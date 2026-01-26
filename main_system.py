#!/usr/bin/env python3
"""
Main integration system for Raspberry Pi Zero 2
Receives:
- Own GPS position (via serial GPS module)
- Wind data from anemometer (via GPIO/I2C/Serial)
- Target position via Ethernet (GeoJSON UDP)

Integrates with geojson_receiver.py for file logging
"""

# IMPORTS & SETUP

import socket
import json
import threading
import time
import serial
import numpy as np
import os
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Tuple
import queue

# Import the geojson_receiver module for file logging
import geojson_receiver

# Try to import GPS library (install with: pip3 install gpsd-py3)
try:
    from gpsd import connect as gpsd_connect, get_current
    GPSD_AVAILABLE = True
except ImportError:
    print("Warning: gpsd module not found. GPS functionality limited.")
    GPSD_AVAILABLE = False


# DATA STRUCTURES
# System State - Stores all sensor data

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
    wind_speed: float = 0.0  # m/s
    wind_direction: float = 0.0  # degrees (0-360)
    wind_timestamp: float = 0.0
    
    # Target position (from Ethernet)
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_alt: float = 0.0
    target_timestamp: float = 0.0
    target_valid: bool = False
    
    # Computed values
    range_to_target: float = 0.0  # meters
    bearing_to_target: float = 0.0  # degrees
    
    # Launch authorization
    launch_authorized: bool = False
    authorization_reasons: list = None
    
    def __post_init__(self):
        if self.authorization_reasons is None:
            self.authorization_reasons = []

# LaunchCriteria - Rules for launch authorization

@dataclass
class LaunchCriteria:
    """Launch authorization criteria and limits"""
    # GPS requirements
    gps_fix_required: bool = True
    gps_data_timeout: float = 5.0  # seconds
    
    # Range limits
    min_range: float = 100.0  # meters
    max_range: float = 5000.0  # meters
    
    # Altitude limits (FAA regulations)
    max_altitude_agl: float = 122.0  # meters (400 feet AGL - FAA Part 101)
    max_altitude_msl: float = 3048.0  # meters (10,000 feet MSL - typical airspace limit)
    check_altitude: bool = True
    
    # Wind limits
    max_wind_speed: float = 15.0  # m/s (about 54 km/h or 33 mph)
    wind_data_timeout: float = 10.0  # seconds
    
    # Engagement zone (rectangular bounds in meters from own position)
    engagement_zone_north: float = 5000.0  # meters
    engagement_zone_south: float = 0.0  # meters
    engagement_zone_east: float = 5000.0  # meters
    engagement_zone_west: float = 0.0  # meters
    engagement_zone_min_alt: float = 0.0  # meters AGL
    engagement_zone_max_alt: float = 400.0  # meters AGL (within FAA limit)
    
    # Target requirements
    target_lock_required: bool = True
    target_data_timeout: float = 5.0  # seconds
    
    # Weather conditions
    check_weather: bool = True
    weather_data_timeout: float = 30.0  # seconds

# GPS RECEIVER CLASS

class GPSReceiver:
    """
    GPS receiver interface using gpsd daemon or direct serial
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
        """Start GPS receiver"""
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
        """Stop GPS receiver"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial_conn:
            self.serial_conn.close()
        print("GPS receiver stopped")
    
    def _gps_loop(self):
        """GPS reading loop"""
        while self.running:
            try:
                if self.use_gpsd:
                    # Read from gpsd
                    packet = get_current()
                    if packet.mode >= 2:  # 2D or 3D fix
                        data = {
                            'lat': packet.lat,
                            'lon': packet.lon,
                            'alt': packet.alt if packet.mode == 3 else 0.0,
                            'timestamp': time.time(),
                            'fix': True
                        }
                        self._update_queue(data)
                else:
                    # Read NMEA from serial
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
        """Parse NMEA GGA sentence"""
        try:
            parts = nmea_sentence.split(',')
            if len(parts) < 10:
                return None
            
            # Check fix quality
            fix_quality = int(parts[6]) if parts[6] else 0
            if fix_quality == 0:
                return None
            
            # Parse latitude
            lat_str = parts[2]
            lat_dir = parts[3]
            lat = float(lat_str[:2]) + float(lat_str[2:]) / 60.0
            if lat_dir == 'S':
                lat = -lat
            
            # Parse longitude
            lon_str = parts[4]
            lon_dir = parts[5]
            lon = float(lon_str[:3]) + float(lon_str[3:]) / 60.0
            if lon_dir == 'W':
                lon = -lon
            
            # Parse altitude
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
        """Update data queue, dropping old data if full"""
        try:
            self.data_queue.put_nowait(data)
        except queue.Full:
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(data)
            except:
                pass
    
    def get_position(self) -> Optional[dict]:
        """Get latest GPS position"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


# ANEMOMETER READER CLASS

class AnemometerReader:
    """
    Anemometer interface - supports multiple types:
    - Pulse counting (GPIO)
    - Analog (ADC via I2C/SPI)
    - Serial (RS485/UART)
    """
    
    def __init__(self, interface_type='gpio', gpio_pin=17):
        self.interface_type = interface_type
        self.gpio_pin = gpio_pin
        self.running = False
        self.thread = None
        self.data_queue = queue.Queue(maxsize=10)
        self.pulse_count = 0
        self.last_calc_time = time.time()
        
    def start(self):
        """Start anemometer reading"""
        self.running = True
        
        if self.interface_type == 'gpio':
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                GPIO.add_event_detect(
                    self.gpio_pin, 
                    GPIO.FALLING, 
                    callback=self._gpio_callback
                )
                print(f"✓ Anemometer GPIO initialized on pin {self.gpio_pin}")
            except Exception as e:
                print(f"✗ Failed to initialize anemometer GPIO: {e}")
                return False
        
        self.thread = threading.Thread(target=self._anemometer_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop anemometer reading"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        
        if self.interface_type == 'gpio':
            try:
                import RPi.GPIO as GPIO
                GPIO.cleanup(self.gpio_pin)
            except:
                pass
        
        print("Anemometer reader stopped")
    
    def _gpio_callback(self, channel):
        """GPIO interrupt callback for pulse counting"""
        self.pulse_count += 1
    
    def _anemometer_loop(self):
        """Calculate wind speed from pulses"""
        while self.running:
            try:
                current_time = time.time()
                elapsed = current_time - self.last_calc_time
                
                if elapsed >= 1.0:  # Calculate every second
                    # Convert pulses to wind speed
                    # Typical: 1 pulse per second = 2.4 km/h = 0.667 m/s
                    # Adjust calibration factor for your anemometer
                    wind_speed = (self.pulse_count / elapsed) * 0.667
                    
                    data = {
                        'speed': wind_speed,
                        'direction': 0.0,  # Add wind vane if available
                        'timestamp': current_time
                    }
                    
                    self._update_queue(data)
                    
                    # Reset counters
                    self.pulse_count = 0
                    self.last_calc_time = current_time
                
                time.sleep(0.1)
                
            except Exception as e:
                if self.running:
                    print(f"Anemometer error: {e}")
                time.sleep(1.0)
    
    def _update_queue(self, data: dict):
        """Update data queue"""
        try:
            self.data_queue.put_nowait(data)
        except queue.Full:
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(data)
            except:
                pass
    
    def get_wind_data(self) -> Optional[dict]:
        """Get latest wind data"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None

# ETHERNET TARGET RECEIVER CLASS

class IntegratedEthernetReceiver:
    """
    Integrated Ethernet receiver that uses geojson_receiver.py for file logging
    AND provides live data to the main system
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
        """Start UDP receiver with integrated file saving"""
        self.running = True
        
        # Create save directory if it doesn't exist
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
        """Stop UDP receiver"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.socket:
            self.socket.close()
        print("Ethernet receiver stopped")
    
    def _receive_loop(self):
        """UDP receive loop with file saving"""
        while self.running:
            try:
                # Receive data
                geojson_bytes, addr = self.socket.recvfrom(self.buffer_size)
                json_str = geojson_bytes.decode('utf-8')
                geojson = json.loads(json_str)
                self.packet_count += 1
                
                # Save to file using geojson_receiver logic
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
                filename = f"geojson_{timestamp}_packet{self.packet_count}.json"
                filepath = os.path.join(self.save_directory, filename)
                
                with open(filepath, 'w', encoding='utf-8') as f:
                    json.dump(geojson, f, indent=2)
                
                print(f"\n[Ethernet] Packet #{self.packet_count} saved to: {filename}")
                
                # Also print details using geojson_receiver function
                geojson_receiver.print_geojson_details(geojson)
                
                # Parse coordinates for live system
                coords = self._parse_geojson(geojson)
                if coords:
                    target_data = {
                        'lat': coords[1],  # latitude
                        'lon': coords[0],  # longitude
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
        """Extract coordinates from GeoJSON"""
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
        """Update data queue"""
        try:
            self.data_queue.put_nowait(data)
        except queue.Full:
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(data)
            except:
                pass
    
    def get_target(self) -> Optional[dict]:
        """Get latest target position"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


class LaunchAuthorizationSystem:
    """
    Evaluates launch authorization based on multiple criteria
    """
    
    def __init__(self, criteria: LaunchCriteria = None):
        self.criteria = criteria if criteria else LaunchCriteria()
        
    def check_gps_quality(self, state: SystemState) -> Tuple[bool, str]:
        """Check GPS fix quality"""
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
        """Check if target is within range limits"""
        if not state.target_valid:
            return False, "No target lock"
        
        range_m = state.range_to_target
        
        if range_m < self.criteria.min_range:
            return False, f"Target too close ({range_m:.0f}m < {self.criteria.min_range:.0f}m)"
        
        if range_m > self.criteria.max_range:
            return False, f"Target too far ({range_m:.0f}m > {self.criteria.max_range:.0f}m)"
        
        return True, f"Range acceptable ({range_m:.0f}m)"
    
    def check_wind_speed(self, state: SystemState) -> Tuple[bool, str]:
        """Check wind speed threshold"""
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
        """Check FAA altitude restrictions"""
        if not self.criteria.check_altitude:
            return True, "Altitude check disabled"
        
        if not state.target_valid:
            return False, "No target altitude data"
        
        # Check target altitude MSL (Mean Sea Level)
        target_alt_msl = state.target_alt
        if target_alt_msl > self.criteria.max_altitude_msl:
            return False, f"Target altitude exceeds MSL limit ({target_alt_msl:.0f}m > {self.criteria.max_altitude_msl:.0f}m)"
        
        # Check target altitude AGL (Above Ground Level)
        # Assuming own position altitude is ground level
        if state.gps_fix:
            target_alt_agl = state.target_alt - state.own_alt
            if target_alt_agl > self.criteria.max_altitude_agl:
                return False, f"Target exceeds FAA altitude limit ({target_alt_agl:.0f}m AGL > {self.criteria.max_altitude_agl:.0f}m)"
            
            return True, f"Altitude within limits ({target_alt_agl:.0f}m AGL, {target_alt_msl:.0f}m MSL)"
        else:
            # Can't calculate AGL without own altitude
            if target_alt_msl > self.criteria.max_altitude_agl:
                return False, f"Target altitude possibly too high ({target_alt_msl:.0f}m, no AGL reference)"
        
        return True, f"Altitude acceptable ({target_alt_msl:.0f}m MSL)"
    
    def check_engagement_zone(self, state: SystemState) -> Tuple[bool, str]:
        """Check if target is within engagement zone (horizontal and vertical)"""
        if not state.target_valid or not state.gps_fix:
            return False, "Cannot determine engagement zone (no GPS or target)"
        
        # Calculate relative position (simplified planar approximation)
        # North/South: latitude difference
        # East/West: longitude difference
        lat_diff = state.target_lat - state.own_lat
        lon_diff = state.target_lon - state.own_lon
        
        # Convert to meters (approximate)
        north_offset = lat_diff * 110540  # meters per degree latitude
        east_offset = lon_diff * 111320 * np.cos(np.radians(state.own_lat))  # meters per degree longitude
        
        # Check horizontal bounds
        if north_offset < self.criteria.engagement_zone_south:
            return False, f"Target south of engagement zone ({north_offset:.0f}m)"
        
        if north_offset > self.criteria.engagement_zone_north:
            return False, f"Target north of engagement zone ({north_offset:.0f}m)"
        
        if east_offset < -self.criteria.engagement_zone_west:
            return False, f"Target west of engagement zone ({east_offset:.0f}m)"
        
        if east_offset > self.criteria.engagement_zone_east:
            return False, f"Target east of engagement zone ({east_offset:.0f}m)"
        
        # Check vertical bounds (altitude)
        target_alt_agl = state.target_alt - state.own_alt
        
        if target_alt_agl < self.criteria.engagement_zone_min_alt:
            return False, f"Target below engagement zone ({target_alt_agl:.0f}m AGL < {self.criteria.engagement_zone_min_alt:.0f}m)"
        
        if target_alt_agl > self.criteria.engagement_zone_max_alt:
            return False, f"Target above engagement zone ({target_alt_agl:.0f}m AGL > {self.criteria.engagement_zone_max_alt:.0f}m)"
        
        return True, f"Target in engagement zone (N:{north_offset:.0f}m, E:{east_offset:.0f}m, Alt:{target_alt_agl:.0f}m AGL)"
    
    def check_weather_conditions(self, state: SystemState) -> Tuple[bool, str]:
        """Check overall weather conditions"""
        if not self.criteria.check_weather:
            return True, "Weather check disabled"
        
        current_time = time.time()
        
        # Check if wind data is recent enough
        if state.wind_timestamp == 0:
            return False, "No weather data available"
        
        age = current_time - state.wind_timestamp
        if age > self.criteria.weather_data_timeout:
            return False, f"Weather data stale ({age:.1f}s old)"
        
        # Additional weather checks could include:
        # - Temperature limits
        # - Humidity limits
        # - Precipitation detection
        # For now, we use wind as primary weather indicator
        
        return True, "Weather conditions acceptable"
    
    def evaluate_authorization(self, state: SystemState) -> Tuple[bool, list]:
        """
        Evaluate all launch authorization criteria
        Returns: (authorized, list of (check_name, passed, message))
        """
        checks = []
        all_passed = True
        
        # Run all checks
        passed, msg = self.check_gps_quality(state)
        checks.append(("GPS Fix Quality", passed, msg))
        all_passed = all_passed and passed
        
        passed, msg = self.check_altitude_limits(state)
        checks.append(("FAA Altitude Limits", passed, msg))
        all_passed = all_passed and passed
        
        passed, msg = self.check_range_limits(state)
        checks.append(("Range Limits", passed, msg))
        all_passed = all_passed and passed
        
        passed, msg = self.check_wind_speed(state)
        checks.append(("Wind Speed", passed, msg))
        all_passed = all_passed and passed
        
        passed, msg = self.check_engagement_zone(state)
        checks.append(("Engagement Zone", passed, msg))
        all_passed = all_passed and passed
        
        passed, msg = self.check_weather_conditions(state)
        checks.append(("Weather Conditions", passed, msg))
        all_passed = all_passed and passed
        
        return all_passed, checks


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Earth radius in meters
    
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    delta_phi = np.radians(lat2 - lat1)
    delta_lambda = np.radians(lon2 - lon1)
    
    a = np.sin(delta_phi/2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    
    return R * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate bearing from point 1 to point 2 in degrees"""
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    delta_lambda = np.radians(lon2 - lon1)
    
    y = np.sin(delta_lambda) * np.cos(phi2)
    x = np.cos(phi1) * np.sin(phi2) - np.sin(phi1) * np.cos(phi2) * np.cos(delta_lambda)
    
    bearing = np.degrees(np.arctan2(y, x))
    return (bearing + 360) % 360


class MainSystem:
    """Main integration system with file logging and launch authorization"""
    
    def __init__(self, launch_criteria: LaunchCriteria = None):
        self.state = SystemState()
        self.gps = GPSReceiver(use_gpsd=False)
        self.anemometer = AnemometerReader(interface_type='gpio', gpio_pin=17)
        self.ethernet = IntegratedEthernetReceiver()
        self.launch_auth = LaunchAuthorizationSystem(launch_criteria)
        self.running = False
        
    def start(self):
        """Start all subsystems"""
        print("\n" + "="*70)
        print("Starting Main Integration System with File Logging")
        print("="*70 + "\n")
        
        # Start GPS
        if not self.gps.start():
            print("Warning: GPS failed to start")
        
        # Start anemometer
        if not self.anemometer.start():
            print("Warning: Anemometer failed to start")
        
        # Start integrated Ethernet receiver (with file saving)
        if not self.ethernet.start():
            print("Warning: Ethernet receiver failed to start")
        
        self.running = True
        print("\n✓ All systems started")
        print("  - GPS data: live updates")
        print("  - Wind data: live updates")
        print(f"  - Ethernet data: live updates + saving to {self.ethernet.save_directory}/")
        print("\n")
        
    def stop(self):
        """Stop all subsystems"""
        print("\n\nStopping all systems...")
        self.running = False
        self.gps.stop()
        self.anemometer.stop()
        self.ethernet.stop()
        print("✓ All systems stopped\n")
    
    def update(self):
        """Update system state from all sensors"""
        # Update GPS position
        gps_data = self.gps.get_position()
        if gps_data:
            self.state.own_lat = gps_data['lat']
            self.state.own_lon = gps_data['lon']
            self.state.own_alt = gps_data['alt']
            self.state.own_timestamp = gps_data['timestamp']
            self.state.gps_fix = gps_data['fix']
        
        # Update wind data
        wind_data = self.anemometer.get_wind_data()
        if wind_data:
            self.state.wind_speed = wind_data['speed']
            self.state.wind_direction = wind_data['direction']
            self.state.wind_timestamp = wind_data['timestamp']
        
        # Update target position
        target_data = self.ethernet.get_target()
        if target_data:
            self.state.target_lat = target_data['lat']
            self.state.target_lon = target_data['lon']
            self.state.target_alt = target_data['alt']
            self.state.target_timestamp = target_data['timestamp']
            self.state.target_valid = target_data['valid']
        
        # Calculate range and bearing if both positions valid
        if self.state.gps_fix and self.state.target_valid:
            self.state.range_to_target = haversine_distance(
                self.state.own_lat, self.state.own_lon,
                self.state.target_lat, self.state.target_lon
            )
            self.state.bearing_to_target = calculate_bearing(
                self.state.own_lat, self.state.own_lon,
                self.state.target_lat, self.state.target_lon
            )
        
        # Evaluate launch authorization
        authorized, checks = self.launch_auth.evaluate_authorization(self.state)
        self.state.launch_authorized = authorized
        self.state.authorization_reasons = checks
    
    def print_status(self):
        """Print current system status with launch authorization"""
        print("\n" + "="*70)
        print(f"System Status - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Packets received: {self.ethernet.packet_count}")
        print("="*70)
        
        print("\n📍 Own Position:")
        if self.state.gps_fix:
            print(f"   Lat: {self.state.own_lat:.6f}°")
            print(f"   Lon: {self.state.own_lon:.6f}°")
            print(f"   Alt: {self.state.own_alt:.1f} m")
            print(f"   GPS Fix: ✓")
        else:
            print("   GPS Fix: ✗ (No fix)")
        
        print("\n💨 Wind Conditions:")
        print(f"   Speed: {self.state.wind_speed:.2f} m/s ({self.state.wind_speed * 3.6:.2f} km/h)")
        print(f"   Direction: {self.state.wind_direction:.1f}°")
        
        print("\n🎯 Target Position:")
        if self.state.target_valid:
            print(f"   Lat: {self.state.target_lat:.6f}°")
            print(f"   Lon: {self.state.target_lon:.6f}°")
            print(f"   Alt: {self.state.target_alt:.1f} m")
            print(f"   Range: {self.state.range_to_target:.1f} m")
            print(f"   Bearing: {self.state.bearing_to_target:.1f}°")
        else:
            print("   No target data received")
        
        # Launch Authorization Display
        print("\n" + "="*70)
        if self.state.launch_authorized:
            print("🟢 LAUNCH AUTHORIZED")
        else:
            print("🔴 LAUNCH PROHIBITED")
        print("="*70)
        
        print("\nLaunch Criteria Status:")
        for check_name, passed, message in self.state.authorization_reasons:
            status = "✓" if passed else "✗"
            print(f"   {status} {check_name}: {message}")
        
        if not self.state.launch_authorized:
            failed_checks = [name for name, passed, _ in self.state.authorization_reasons if not passed]
            print(f"\n⚠️  {len(failed_checks)} check(s) failed: {', '.join(failed_checks)}")
        
        print("="*70)
    
    def run(self, update_interval=2.0):
        """Main run loop"""
        try:
            while self.running:
                self.update()
                self.print_status()
                time.sleep(update_interval)
        except KeyboardInterrupt:
            print("\n\nShutdown requested...")
        finally:
            self.stop()


if __name__ == '__main__':
    print("="*70)
    print("Raspberry Pi Zero 2 - Integrated System with Launch Authorization")
    print("="*70)
    print("\nIntegrates:")
    print("  • GPS position (own coordinates)")
    print("  • Anemometer (wind speed/direction)")
    print("  • Ethernet UDP (target GeoJSON data)")
    print("\nLaunch Authorization Checks:")
    print("  • GPS fix quality")
    print("  • FAA altitude limits (400 ft / 122m AGL)")
    print("  • Range limits (100m - 5000m)")
    print("  • Wind speed threshold (< 15 m/s)")
    print("  • Target within engagement zone (horizontal & vertical)")
    print("  • Weather conditions acceptable")
    print("\nFile Logging:")
    print("  • All received GeoJSON packets saved to files")
    print("  • Live data processing simultaneous with logging")
    print("\nPress Ctrl+C to stop")
    print("="*70)
    
    # Optional: Customize launch criteria
    custom_criteria = LaunchCriteria(
        # Altitude limits (FAA Part 101)
        max_altitude_agl=122.0,  # 400 feet AGL
        max_altitude_msl=3048.0,  # 10,000 feet MSL
        check_altitude=True,
        # Range limits
        min_range=100.0,
        max_range=5000.0,
        # Wind limits
        max_wind_speed=15.0,
        # Engagement zone (horizontal)
        engagement_zone_north=5000.0,
        engagement_zone_south=0.0,
        engagement_zone_east=5000.0,
        engagement_zone_west=0.0,
        # Engagement zone (vertical)
        engagement_zone_min_alt=0.0,
        engagement_zone_max_alt=400.0  # meters AGL
    )
    
    system = MainSystem(launch_criteria=custom_criteria)
    system.start()
    system.run(update_interval=2.0)