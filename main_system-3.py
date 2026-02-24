#!/usr/bin/env python3
"""
Main integration system for Raspberry Pi Zero 2
Receives:
- Own GPS position (via serial GPS module)
- Wind data from anemometer (via GPIO/I2C/Serial)
- Target position via Ethernet (GeoJSON UDP)

Integrates with geojson_receiver.py for file logging
Includes launch ignition control via GPIO

WIRING:
  - Anemometer        : RS485 USB adapter on /dev/ttyUSB1
                        (Modbus RTU, slave addr 2, 9600 baud)
  - Igniter circuit   : GPIO 17 (BCM, physical pin 11) → MOSFET/relay gate
                             5V/12V supply → igniter → MOSFET drain
                             MOSFET source → GND (shared with Pi GND)
  - Add 1N4002 flyback diode across igniter (cathode toward supply +)
"""

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTS & SETUP
# ─────────────────────────────────────────────────────────────────────────────

import socket
import json
import threading
import time
import serial
import numpy as np
import os
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, Tuple
import queue

# Import the geojson_receiver module for file logging
import geojson_receiver

# GPIO output for igniter
try:
    from gpiozero import OutputDevice
    GPIOZERO_AVAILABLE = True
except ImportError:
    print("Warning: gpiozero not found. Ignition functionality disabled.")
    print("  Install with: sudo apt install python3-gpiozero")
    GPIOZERO_AVAILABLE = False

# Try to import GPS library (install with: pip3 install gpsd-py3)
try:
    from gpsd import connect as gpsd_connect, get_current
    GPSD_AVAILABLE = True
except ImportError:
    print("Warning: gpsd module not found. GPS functionality limited.")
    GPSD_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# DATA STRUCTURES
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SystemState:
    """Complete system state"""
    # Own position
    own_lat: float = 0.0
    own_lon: float = 0.0
    own_alt: float = 0.0
    own_timestamp: float = 0.0
    gps_fix: bool = False

    # Wind data
    wind_speed: float = 0.0       # m/s
    wind_direction: float = 0.0   # degrees (0-360)
    wind_pulses: int = 0          # raw pulses in last second (debug)
    wind_timestamp: float = 0.0

    # Target position (from Ethernet)
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_alt: float = 0.0
    target_timestamp: float = 0.0
    target_valid: bool = False

    # Computed values
    range_to_target: float = 0.0    # meters
    bearing_to_target: float = 0.0  # degrees

    # Launch authorization
    launch_authorized: bool = False
    authorization_reasons: list = field(default_factory=list)


@dataclass
class LaunchCriteria:
    """Launch authorization criteria and limits"""
    # GPS requirements
    gps_fix_required: bool = True
    gps_data_timeout: float = 5.0       # seconds

    # Range limits
    min_range: float = 100.0            # meters
    max_range: float = 5000.0           # meters

    # Altitude limits (FAA regulations)
    max_altitude_agl: float = 122.0     # meters (400 feet AGL - FAA Part 101)
    max_altitude_msl: float = 3048.0    # meters (10,000 feet MSL)
    check_altitude: bool = True

    # Wind limits
    max_wind_speed: float = 15.0        # m/s (~54 km/h / 33 mph)
    wind_data_timeout: float = 10.0     # seconds

    # Engagement zone (meters from own position)
    engagement_zone_north: float = 5000.0
    engagement_zone_south: float = 0.0
    engagement_zone_east: float = 5000.0
    engagement_zone_west: float = 0.0
    engagement_zone_min_alt: float = 0.0    # meters AGL
    engagement_zone_max_alt: float = 400.0  # meters AGL

    # Target requirements
    target_lock_required: bool = True
    target_data_timeout: float = 5.0    # seconds

    # Weather conditions
    check_weather: bool = True
    weather_data_timeout: float = 30.0  # seconds


# ─────────────────────────────────────────────────────────────────────────────
# GPS RECEIVER
# ─────────────────────────────────────────────────────────────────────────────

class GPSReceiver:
    """
    GPS receiver interface using gpsd daemon or direct serial NMEA
    """

    def __init__(self, use_gpsd=True, serial_port='/dev/ttyAMA0', baudrate=9600):
        self.use_gpsd = use_gpsd and GPSD_AVAILABLE
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.running = False
        self.thread = None
        self.data_queue = queue.Queue(maxsize=10)
        self.serial_conn = None

    def start(self):
        self.running = True

        if self.use_gpsd:
            print("Starting GPS using gpsd daemon...")
            try:
                gpsd_connect()
                print("✓ Connected to gpsd")
            except Exception as e:
                print(f"✗ Failed to connect to gpsd: {e}")
                print("  Start gpsd with: sudo systemctl start gpsd")
                return False
        else:
            print(f"Starting GPS using serial port {self.serial_port}...")
            try:
                self.serial_conn = serial.Serial(
                    self.serial_port,
                    baudrate=self.baudrate,
                    timeout=1.0
                )
                print("✓ GPS serial port opened")
            except Exception as e:
                print(f"✗ Failed to open GPS serial: {e}")
                return False

        self.thread = threading.Thread(target=self._gps_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial_conn:
            self.serial_conn.close()
        print("GPS receiver stopped")

    def _gps_loop(self):
        while self.running:
            try:
                if self.use_gpsd:
                    packet = get_current()
                    if packet.mode >= 2:
                        data = {
                            'lat': packet.lat,
                            'lon': packet.lon,
                            'alt': packet.alt if packet.mode == 3 else 0.0,
                            'timestamp': time.time(),
                            'fix': True
                        }
                        self._update_queue(data)
                else:
                    line = self.serial_conn.readline().decode('ascii', errors='ignore')
                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        data = self._parse_nmea_gga(line)
                        if data:
                            self._update_queue(data)

                time.sleep(0.1)

            except Exception as e:
                if self.running:
                    print(f"GPS read error: {e}")
                time.sleep(1.0)

    def _parse_nmea_gga(self, nmea_sentence: str) -> Optional[dict]:
        try:
            parts = nmea_sentence.split(',')
            if len(parts) < 10:
                return None

            fix_quality = int(parts[6]) if parts[6] else 0
            if fix_quality == 0:
                return None

            lat_str = parts[2]
            lat_dir = parts[3]
            lat = float(lat_str[:2]) + float(lat_str[2:]) / 60.0
            if lat_dir == 'S':
                lat = -lat

            lon_str = parts[4]
            lon_dir = parts[5]
            lon = float(lon_str[:3]) + float(lon_str[3:]) / 60.0
            if lon_dir == 'W':
                lon = -lon

            alt = float(parts[9]) if parts[9] else 0.0

            return {
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'timestamp': time.time(),
                'fix': True
            }
        except Exception as e:
            print(f"NMEA parse error: {e}")
            return None

    def _update_queue(self, data: dict):
        try:
            self.data_queue.put_nowait(data)
        except queue.Full:
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(data)
            except:
                pass

    def get_position(self) -> Optional[dict]:
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


# ─────────────────────────────────────────────────────────────────────────────
# ANEMOMETER READER  (Modbus RTU over RS485/USB — matches wind_logger.py)
# ─────────────────────────────────────────────────────────────────────────────

class AnemometerReader:
    """
    Reads wind speed from an RS485 Modbus RTU anemometer via USB serial adapter.

    Matches the protocol used in wind_logger.py:
      - Port       : /dev/ttyUSB1  (change if needed)
      - Baud rate  : 9600
      - Slave addr : 2
      - Register   : 0x0000  (raw value = wind speed in 0.1 m/s units)

    Output units: m/s (stored in state) and knots (displayed).
    """

    def __init__(self,
                 port: str = '/dev/ttyUSB1',
                 baudrate: int = 9600,
                 slave_addr: int = 2,
                 sample_interval: float = 1.0):
        self.port            = port
        self.baudrate        = baudrate
        self.slave_addr      = slave_addr
        self.sample_interval = sample_interval
        self.running         = False
        self.thread          = None
        self.data_queue      = queue.Queue(maxsize=10)
        self._serial         = None
        self._gpio_init_ok   = False   # reused flag: True = serial opened OK

    # ── Modbus helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _build_query(self, start_reg: int, num_regs: int) -> bytes:
        query = bytes([
            self.slave_addr, 0x03,
            (start_reg >> 8) & 0xFF, start_reg & 0xFF,
            (num_regs >> 8) & 0xFF,  num_regs & 0xFF
        ])
        crc = self._crc16(query)
        return query + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        self.running = True
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            time.sleep(0.5)
            self._gpio_init_ok = True
            print(f"✓ Anemometer: Modbus RTU on {self.port} "
                  f"(baud={self.baudrate}, slave={self.slave_addr})")
        except Exception as e:
            print(f"✗ Anemometer serial open failed: {e}")
            print(f"  Check that {self.port} exists: ls /dev/ttyUSB*")
            self._gpio_init_ok = False

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True   # keep system running even if serial failed

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        print("Anemometer reader stopped")

    # ── Read loop ─────────────────────────────────────────────────────────────

    def _read_loop(self):
        import struct
        while self.running:
            try:
                if not self._serial or not self._serial.is_open:
                    time.sleep(self.sample_interval)
                    continue

                self._serial.reset_input_buffer()
                query = self._build_query(0x0000, 1)
                self._serial.write(query)
                time.sleep(0.15)

                if self._serial.in_waiting > 0:
                    response = self._serial.read(self._serial.in_waiting)

                    if len(response) >= 7:
                        raw_value    = struct.unpack('>H', response[3:5])[0]
                        wind_speed_ms   = raw_value / 10.0          # m/s
                        wind_speed_kts  = wind_speed_ms * 1.94384   # knots

                        self._update_queue({
                            'speed':     wind_speed_ms,
                            'speed_kts': wind_speed_kts,
                            'direction': 0.0,
                            'raw':       raw_value,
                            'pulses':    raw_value,   # reuse field for display
                            'timestamp': time.time()
                        })
                    else:
                        print(f"[Anemometer] Short response ({len(response)} bytes)")
                else:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"Anemometer: no response from sensor")

            except Exception as e:
                if self.running:
                    print(f"Anemometer read error: {e}")
                time.sleep(1.0)

            time.sleep(self.sample_interval)

    # ── Queue helpers ─────────────────────────────────────────────────────────

    def _update_queue(self, data: dict):
        try:
            self.data_queue.put_nowait(data)
        except queue.Full:
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(data)
            except:
                pass

    def get_wind_data(self) -> Optional[dict]:
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


# ─────────────────────────────────────────────────────────────────────────────
# ETHERNET TARGET RECEIVER
# ─────────────────────────────────────────────────────────────────────────────

class IntegratedEthernetReceiver:
    """
    Integrated Ethernet receiver — uses geojson_receiver.py for file logging
    and provides live target data to the main system
    """

    def __init__(self, host='0.0.0.0', port=5005, buffer_size=65536):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.socket = None
        self.running = False
        self.thread = None
        self.data_queue = queue.Queue(maxsize=10)
        self.save_directory = geojson_receiver.SAVE_DIRECTORY
        self.packet_count = 0

    def start(self):
        self.running = True

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
            print(f"Created directory: {self.save_directory}")

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.settimeout(1.0)
            print(f"✓ Ethernet receiver listening on {self.host}:{self.port}")
            print(f"  Saving GeoJSON files to: {self.save_directory}/")
        except Exception as e:
            print(f"✗ Failed to start Ethernet receiver: {e}")
            return False

        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.socket:
            self.socket.close()
        print("Ethernet receiver stopped")

    def _receive_loop(self):
        while self.running:
            try:
                geojson_bytes, addr = self.socket.recvfrom(self.buffer_size)
                json_str = geojson_bytes.decode('utf-8')
                geojson = json.loads(json_str)
                self.packet_count += 1

                # Save to file
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
                filename = f"geojson_{timestamp}_packet{self.packet_count}.json"
                filepath = os.path.join(self.save_directory, filename)

                with open(filepath, 'w', encoding='utf-8') as f:
                    json.dump(geojson, f, indent=2)

                print(f"\n[Ethernet] Packet #{self.packet_count} saved to: {filename}")
                geojson_receiver.print_geojson_details(geojson)

                coords = self._parse_geojson(geojson)
                if coords:
                    target_data = {
                        'lat': coords[1],
                        'lon': coords[0],
                        'alt': coords[2] if len(coords) > 2 else 0.0,
                        'timestamp': time.time(),
                        'valid': True
                    }
                    self._update_queue(target_data)

            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                print(f"[Ethernet] JSON decode error: {e}")
            except Exception as e:
                if self.running:
                    print(f"[Ethernet] Receive error: {e}")

    def _parse_geojson(self, geojson: dict) -> Optional[list]:
        try:
            if geojson.get('type') == 'FeatureCollection':
                features = geojson.get('features', [])
                if features:
                    geom = features[0].get('geometry', {})
                    return geom.get('coordinates')
            elif geojson.get('type') == 'Feature':
                geom = geojson.get('geometry', {})
                return geom.get('coordinates')
            elif 'coordinates' in geojson:
                return geojson.get('coordinates')
        except:
            pass
        return None

    def _update_queue(self, data: dict):
        try:
            self.data_queue.put_nowait(data)
        except queue.Full:
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(data)
            except:
                pass

    def get_target(self) -> Optional[dict]:
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


# ─────────────────────────────────────────────────────────────────────────────
# LAUNCH AUTHORIZATION SYSTEM
# ─────────────────────────────────────────────────────────────────────────────

class LaunchAuthorizationSystem:
    """Evaluates launch authorization based on multiple criteria"""

    def __init__(self, criteria: LaunchCriteria = None):
        self.criteria = criteria if criteria else LaunchCriteria()

    def check_gps_quality(self, state: SystemState) -> Tuple[bool, str]:
        current_time = time.time()

        if not self.criteria.gps_fix_required:
            return True, "GPS check disabled"

        if not state.gps_fix:
            return False, "No GPS fix"

        if state.own_timestamp == 0:
            return False, "No GPS data received"

        age = current_time - state.own_timestamp
        if age > self.criteria.gps_data_timeout:
            return False, f"GPS data stale ({age:.1f}s old)"

        return True, f"GPS fix valid (age: {age:.1f}s)"

    def check_range_limits(self, state: SystemState) -> Tuple[bool, str]:
        if not state.target_valid:
            return False, "No target lock"

        range_m = state.range_to_target

        if range_m < self.criteria.min_range:
            return False, f"Target too close ({range_m:.0f}m < {self.criteria.min_range:.0f}m)"

        if range_m > self.criteria.max_range:
            return False, f"Target too far ({range_m:.0f}m > {self.criteria.max_range:.0f}m)"

        return True, f"Range acceptable ({range_m:.0f}m)"

    def check_wind_speed(self, state: SystemState) -> Tuple[bool, str]:
        current_time = time.time()

        if state.wind_timestamp == 0:
            return False, "No wind data received"

        age = current_time - state.wind_timestamp
        if age > self.criteria.wind_data_timeout:
            return False, f"Wind data stale ({age:.1f}s old)"

        if state.wind_speed > self.criteria.max_wind_speed:
            return False, f"Wind too high ({state.wind_speed:.1f} m/s > {self.criteria.max_wind_speed:.1f} m/s)"

        return True, f"Wind acceptable ({state.wind_speed:.1f} m/s, {state.wind_speed * 3.6:.1f} km/h)"

    def check_altitude_limits(self, state: SystemState) -> Tuple[bool, str]:
        if not self.criteria.check_altitude:
            return True, "Altitude check disabled"

        if not state.target_valid:
            return False, "No target altitude data"

        target_alt_msl = state.target_alt
        if target_alt_msl > self.criteria.max_altitude_msl:
            return False, f"Target altitude exceeds MSL limit ({target_alt_msl:.0f}m > {self.criteria.max_altitude_msl:.0f}m)"

        if state.gps_fix:
            target_alt_agl = state.target_alt - state.own_alt
            if target_alt_agl > self.criteria.max_altitude_agl:
                return False, f"Target exceeds FAA altitude limit ({target_alt_agl:.0f}m AGL > {self.criteria.max_altitude_agl:.0f}m)"
            return True, f"Altitude within limits ({target_alt_agl:.0f}m AGL, {target_alt_msl:.0f}m MSL)"
        else:
            if target_alt_msl > self.criteria.max_altitude_agl:
                return False, f"Target altitude possibly too high ({target_alt_msl:.0f}m, no AGL reference)"

        return True, f"Altitude acceptable ({target_alt_msl:.0f}m MSL)"

    def check_engagement_zone(self, state: SystemState) -> Tuple[bool, str]:
        if not state.target_valid or not state.gps_fix:
            return False, "Cannot determine engagement zone (no GPS or target)"

        lat_diff = state.target_lat - state.own_lat
        lon_diff = state.target_lon - state.own_lon

        north_offset = lat_diff * 110540
        east_offset = lon_diff * 111320 * np.cos(np.radians(state.own_lat))

        if north_offset < self.criteria.engagement_zone_south:
            return False, f"Target south of engagement zone ({north_offset:.0f}m)"

        if north_offset > self.criteria.engagement_zone_north:
            return False, f"Target north of engagement zone ({north_offset:.0f}m)"

        if east_offset < -self.criteria.engagement_zone_west:
            return False, f"Target west of engagement zone ({east_offset:.0f}m)"

        if east_offset > self.criteria.engagement_zone_east:
            return False, f"Target east of engagement zone ({east_offset:.0f}m)"

        target_alt_agl = state.target_alt - state.own_alt

        if target_alt_agl < self.criteria.engagement_zone_min_alt:
            return False, f"Target below engagement zone ({target_alt_agl:.0f}m AGL < {self.criteria.engagement_zone_min_alt:.0f}m)"

        if target_alt_agl > self.criteria.engagement_zone_max_alt:
            return False, f"Target above engagement zone ({target_alt_agl:.0f}m AGL > {self.criteria.engagement_zone_max_alt:.0f}m)"

        return True, f"Target in engagement zone (N:{north_offset:.0f}m, E:{east_offset:.0f}m, Alt:{target_alt_agl:.0f}m AGL)"

    def check_weather_conditions(self, state: SystemState) -> Tuple[bool, str]:
        if not self.criteria.check_weather:
            return True, "Weather check disabled"

        current_time = time.time()

        if state.wind_timestamp == 0:
            return False, "No weather data available"

        age = current_time - state.wind_timestamp
        if age > self.criteria.weather_data_timeout:
            return False, f"Weather data stale ({age:.1f}s old)"

        return True, "Weather conditions acceptable"

    def evaluate_authorization(self, state: SystemState) -> Tuple[bool, list]:
        """
        Run all checks. Returns (all_passed, list of (name, passed, message)).
        """
        checks = []
        all_passed = True

        for name, fn in [
            ("GPS Fix Quality",     self.check_gps_quality),
            ("FAA Altitude Limits", self.check_altitude_limits),
            ("Range Limits",        self.check_range_limits),
            ("Wind Speed",          self.check_wind_speed),
            ("Engagement Zone",     self.check_engagement_zone),
            ("Weather Conditions",  self.check_weather_conditions),
        ]:
            passed, msg = fn(state)
            checks.append((name, passed, msg))
            all_passed = all_passed and passed

        return all_passed, checks


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance between two GPS coordinates in meters"""
    R = 6371000
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    d_phi = np.radians(lat2 - lat1)
    d_lam = np.radians(lon2 - lon1)
    a = np.sin(d_phi / 2) ** 2 + np.cos(phi1) * np.cos(phi2) * np.sin(d_lam / 2) ** 2
    return R * 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Bearing from point 1 to point 2 in degrees (0-360)"""
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    d_lam = np.radians(lon2 - lon1)
    y = np.sin(d_lam) * np.cos(phi2)
    x = np.cos(phi1) * np.sin(phi2) - np.sin(phi1) * np.cos(phi2) * np.cos(d_lam)
    return (np.degrees(np.arctan2(y, x)) + 360) % 360


# ─────────────────────────────────────────────────────────────────────────────
# MAIN SYSTEM
# ─────────────────────────────────────────────────────────────────────────────

class MainSystem:
    """
    Main integration system with GPS, anemometer, Ethernet target tracking,
    launch authorization, and ignition control.
    """

    # ── Igniter config ────────────────────────────────────────────────────────
    LAUNCH_PIN      = 17    # BCM GPIO pin (physical pin 11)
    SIGNAL_DURATION = 2.0   # Seconds to hold ignition signal HIGH
    COUNTDOWN_SECS  = 5     # Pre-fire countdown

    def __init__(self, launch_criteria: LaunchCriteria = None):
        self.state = SystemState()
        self.gps = GPSReceiver(use_gpsd=False)
        self.anemometer = AnemometerReader(port='/dev/ttyUSB1', baudrate=9600, slave_addr=2)
        self.ethernet = IntegratedEthernetReceiver()
        self.launch_auth = LaunchAuthorizationSystem(launch_criteria)
        self.running = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        print("\n" + "=" * 70)
        print("Starting Main Integration System with File Logging")
        print("=" * 70 + "\n")

        if not self.gps.start():
            print("Warning: GPS failed to start")

        if not self.anemometer.start():
            print("Warning: Anemometer failed to start")

        if not self.ethernet.start():
            print("Warning: Ethernet receiver failed to start")

        self.running = True
        print("\n✓ All systems started")
        print("  - GPS data        : live updates")
        print("  - Wind data       : live updates")
        print(f"  - Ethernet data   : live updates + saving to {self.ethernet.save_directory}/")
        print(f"  - Ignition pin    : GPIO {self.LAUNCH_PIN} (BCM)")
        print()

    def stop(self):
        print("\n\nStopping all systems...")
        self.running = False
        self.gps.stop()
        self.anemometer.stop()
        self.ethernet.stop()
        print("✓ All systems stopped\n")

    # ── State update ──────────────────────────────────────────────────────────

    def update(self):
        """Pull latest data from all sensors and recompute derived values"""
        gps_data = self.gps.get_position()
        if gps_data:
            self.state.own_lat       = gps_data['lat']
            self.state.own_lon       = gps_data['lon']
            self.state.own_alt       = gps_data['alt']
            self.state.own_timestamp = gps_data['timestamp']
            self.state.gps_fix       = gps_data['fix']

        wind_data = self.anemometer.get_wind_data()
        if wind_data:
            self.state.wind_speed     = wind_data['speed']
            self.state.wind_direction = wind_data['direction']
            self.state.wind_pulses    = wind_data.get('pulses', 0)
            self.state.wind_timestamp = wind_data['timestamp']

        target_data = self.ethernet.get_target()
        if target_data:
            self.state.target_lat       = target_data['lat']
            self.state.target_lon       = target_data['lon']
            self.state.target_alt       = target_data['alt']
            self.state.target_timestamp = target_data['timestamp']
            self.state.target_valid     = target_data['valid']

        if self.state.gps_fix and self.state.target_valid:
            self.state.range_to_target = haversine_distance(
                self.state.own_lat, self.state.own_lon,
                self.state.target_lat, self.state.target_lon
            )
            self.state.bearing_to_target = calculate_bearing(
                self.state.own_lat, self.state.own_lon,
                self.state.target_lat, self.state.target_lon
            )

        authorized, checks = self.launch_auth.evaluate_authorization(self.state)
        self.state.launch_authorized    = authorized
        self.state.authorization_reasons = checks

    # ── Status display ────────────────────────────────────────────────────────

    def print_status(self):
        print("\n" + "=" * 70)
        print(f"System Status — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Packets received: {self.ethernet.packet_count}")
        print("=" * 70)

        print("\n📍 Own Position:")
        if self.state.gps_fix:
            print(f"   Lat: {self.state.own_lat:.6f}°")
            print(f"   Lon: {self.state.own_lon:.6f}°")
            print(f"   Alt: {self.state.own_alt:.1f} m")
            print(f"   GPS Fix: ✓")
        else:
            print("   GPS Fix: ✗ (No fix)")

        print("\n💨 Wind Conditions:")
        print(f"   Speed:     {self.state.wind_speed:.2f} m/s  |  "
              f"{self.state.wind_speed * 1.94384:.2f} kts  |  "
              f"{self.state.wind_speed * 3.6:.2f} km/h")
        print(f"   Direction: {self.state.wind_direction:.1f}°")
        serial_status = "Serial OK" if self.anemometer._gpio_init_ok else "Serial FAIL — check /dev/ttyUSB1"
        print(f"   Raw value: {self.state.wind_pulses}  [{serial_status}]")

        print("\n🎯 Target Position:")
        if self.state.target_valid:
            print(f"   Lat:     {self.state.target_lat:.6f}°")
            print(f"   Lon:     {self.state.target_lon:.6f}°")
            print(f"   Alt:     {self.state.target_alt:.1f} m")
            print(f"   Range:   {self.state.range_to_target:.1f} m")
            print(f"   Bearing: {self.state.bearing_to_target:.1f}°")
        else:
            print("   No target data received")

        print("\n" + "=" * 70)
        if self.state.launch_authorized:
            print("🟢 LAUNCH AUTHORIZED")
        else:
            print("🔴 LAUNCH PROHIBITED")
        print("=" * 70)

        print("\nLaunch Criteria Status:")
        for check_name, passed, message in self.state.authorization_reasons:
            status = "✓" if passed else "✗"
            print(f"   {status} {check_name}: {message}")

        if not self.state.launch_authorized:
            failed = [n for n, p, _ in self.state.authorization_reasons if not p]
            print(f"\n⚠️  {len(failed)} check(s) failed: {', '.join(failed)}")

        print("=" * 70)

    # ── Ignition ──────────────────────────────────────────────────────────────

    def fire(self) -> bool:
        """
        Attempt to fire the igniter.
        Blocked unless all launch authorization checks pass.
        Requires operator to type FIRE at the prompt.

        Returns True if ignition signal was sent, False otherwise.
        """
        if not GPIOZERO_AVAILABLE:
            print("🔴 Ignition unavailable — gpiozero not installed.")
            return False

        # Gate on authorization
        if not self.state.launch_authorized:
            print("\n🔴 LAUNCH PROHIBITED — ignition blocked")
            for name, passed, msg in self.state.authorization_reasons:
                if not passed:
                    print(f"   ✗ {name}: {msg}")
            return False

        # Operator confirmation
        print("\n✅ All authorization checks passed.")
        confirm = input("Type FIRE to begin countdown, anything else to abort: ")
        if confirm.strip().upper() != "FIRE":
            print("Launch aborted.")
            return False

        # Countdown
        print()
        for i in range(self.COUNTDOWN_SECS, 0, -1):
            print(f"  T-{i}...")
            time.sleep(1)

        # Fire
        igniter = OutputDevice(self.LAUNCH_PIN, active_high=True, initial_value=False)
        try:
            print("\n🚀 IGNITION!")
            igniter.on()
            time.sleep(self.SIGNAL_DURATION)
            igniter.off()
            print("Signal complete. Igniter circuit open.")
            return True
        finally:
            igniter.off()
            igniter.close()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self, update_interval: float = 2.0, auto_fire: bool = False):
        """
        Main run loop.

        Args:
            update_interval: Seconds between status updates.
            auto_fire: If True, calls fire() automatically when launch is
                       authorized. If False, operator must call fire() manually.
        """
        try:
            while self.running:
                self.update()
                self.print_status()

                if auto_fire and self.state.launch_authorized:
                    fired = self.fire()
                    if fired:
                        break  # Stop loop after successful ignition

                time.sleep(update_interval)
        except KeyboardInterrupt:
            print("\n\nShutdown requested...")
        finally:
            self.stop()


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("=" * 70)
    print("Raspberry Pi Zero 2 — Integrated System with Launch Authorization")
    print("=" * 70)
    print("\nSubsystems:")
    print("  • GPS position      (own coordinates via serial NMEA)")
    print("  • Anemometer        (wind speed via Modbus RTU on /dev/ttyUSB1)")
    print("  • Ethernet UDP      (target GeoJSON data)")
    print("  • Ignition control  (GPIO 17 → MOSFET/relay → igniter)")
    print("\nLaunch Authorization Checks:")
    print("  • GPS fix quality")
    print("  • FAA altitude limits (400 ft / 122m AGL)")
    print("  • Range limits (100m – 5000m)")
    print("  • Wind speed threshold (< 15 m/s)")
    print("  • Target within engagement zone (horizontal & vertical)")
    print("  • Weather conditions acceptable")
    print("\nPress Ctrl+C to stop")
    print("=" * 70)

    custom_criteria = LaunchCriteria(
        max_altitude_agl=122.0,
        max_altitude_msl=3048.0,
        check_altitude=True,
        min_range=100.0,
        max_range=5000.0,
        max_wind_speed=15.0,
        engagement_zone_north=5000.0,
        engagement_zone_south=0.0,
        engagement_zone_east=5000.0,
        engagement_zone_west=0.0,
        engagement_zone_min_alt=0.0,
        engagement_zone_max_alt=400.0,
    )

    system = MainSystem(launch_criteria=custom_criteria)
    system.start()

    # Set auto_fire=True to trigger ignition automatically when all checks pass,
    # or leave False to call system.fire() manually at the right moment.
    system.run(update_interval=2.0, auto_fire=False)
