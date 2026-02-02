#!/usr/bin/env python3
"""
Integrated Main Test System for Raspberry Pi 5
Combines: GPS + Wind Sensor + GeoJSON Receiver

Hardware Setup:
- GPS module on USB (e.g., /dev/ttyUSB0)
- Wind sensor on USB (e.g., /dev/ttyUSB1)
- Ethernet/WiFi for GeoJSON UDP receiver

Usage:
  python3 main_test.py
  python3 main_test.py --gps-port /dev/ttyUSB0 --wind-port /dev/ttyUSB1
"""

import socket
import time
import threading
import serial
import argparse
import json
from datetime import datetime
from typing import Optional, Tuple
from geojson_receiver import receive_and_get_coordinates, HOST, PORT
from wind_logger import wind_monitor_thread

# ============================================================================
# GPS RECEIVER CLASS
# ============================================================================

class GPSReceiver:
    """GPS receiver for real-time position data"""
    
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=9600):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        self.thread = None
        
        # Shared GPS state
        self.gps_lock = threading.Lock()
        self.has_fix = False
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.satellites = 0
        self.last_update = 0
        self.fix_count = 0
        
    def start(self):
        """Start GPS receiver thread"""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            print(f"✓ GPS connected on {self.serial_port}")
            self.running = True
            self.thread = threading.Thread(target=self._gps_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"✗ GPS failed to start on {self.serial_port}: {e}")
            return False
    
    def stop(self):
        """Stop GPS receiver"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial_conn:
            self.serial_conn.close()
    
    def _parse_nmea_gga(self, sentence: str) -> Optional[dict]:
        """Parse NMEA GGA sentence"""
        try:
            parts = sentence.strip().split(',')
            
            if not (sentence.startswith('$GPGGA') or sentence.startswith('$GNGGA')):
                return None
            
            if len(parts) < 15:
                return None
            
            # Check fix quality
            fix_quality = int(parts[6]) if parts[6] else 0
            if fix_quality == 0:
                return {'fix': False}
            
            # Parse latitude
            lat_str = parts[2]
            lat_dir = parts[3]
            if lat_str and lat_dir:
                lat_deg = float(lat_str[:2])
                lat_min = float(lat_str[2:])
                lat = lat_deg + (lat_min / 60.0)
                if lat_dir == 'S':
                    lat = -lat
            else:
                return {'fix': False}
            
            # Parse longitude
            lon_str = parts[4]
            lon_dir = parts[5]
            if lon_str and lon_dir:
                lon_deg = float(lon_str[:3])
                lon_min = float(lon_str[3:])
                lon = lon_deg + (lon_min / 60.0)
                if lon_dir == 'W':
                    lon = -lon
            else:
                return {'fix': False}
            
            # Parse altitude
            alt = float(parts[9]) if parts[9] else 0.0
            
            # Parse satellites
            num_sats = int(parts[7]) if parts[7] else 0
            
            return {
                'fix': True,
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'satellites': num_sats
            }
            
        except Exception:
            return None
    
    def _gps_loop(self):
        """GPS reading loop"""
        print("[GPS] Monitoring thread started")
        while self.running:
            try:
                line = self.serial_conn.readline().decode('ascii', errors='ignore')
                
                if '$GPGGA' in line or '$GNGGA' in line:
                    data = self._parse_nmea_gga(line)
                    
                    if data and data.get('fix'):
                        with self.gps_lock:
                            self.has_fix = True
                            self.latitude = data['lat']
                            self.longitude = data['lon']
                            self.altitude = data['alt']
                            self.satellites = data['satellites']
                            self.last_update = time.time()
                            self.fix_count += 1
                    else:
                        with self.gps_lock:
                            self.has_fix = False
                
                time.sleep(0.01)
                
            except Exception as e:
                if self.running:
                    print(f"[GPS] Read error: {e}")
                time.sleep(1.0)
        
        print("[GPS] Monitoring thread stopped")
    
    def get_position(self) -> Tuple[bool, float, float, float, int]:
        """
        Get current GPS position
        
        Returns:
            (has_fix, latitude, longitude, altitude, satellites)
        """
        with self.gps_lock:
            return (
                self.has_fix,
                self.latitude,
                self.longitude,
                self.altitude,
                self.satellites
            )
    
    def get_status_string(self) -> str:
        """Get formatted GPS status string"""
        has_fix, lat, lon, alt, sats = self.get_position()
        
        if has_fix:
            return f"GPS: FIX ✓ | Lat={lat:.6f}° Lon={lon:.6f}° Alt={alt:.1f}m | Sats={sats}"
        else:
            return "GPS: NO FIX ✗ (waiting for satellites...)"


# ============================================================================
# MAIN INTEGRATED SYSTEM
# ============================================================================

class IntegratedSystem:
    """Main integrated system combining GPS, Wind, and GeoJSON receiver"""
    
    def __init__(self, gps_port='/dev/ttyUSB0', wind_port='/dev/ttyUSB1',
                 geojson_host='0.0.0.0', geojson_port=5005,
                 wind_threshold=20, wind_check_interval=5):
        
        # GPS receiver
        self.gps = GPSReceiver(serial_port=gps_port)
        
        # Wind monitoring (shared state)
        self.wind_is_valid = [False]
        self.current_wind_speed = [None]
        self.wind_lock = threading.Lock()
        self.wind_port = wind_port
        self.wind_threshold = wind_threshold
        self.wind_check_interval = wind_check_interval
        self.wind_thread = None
        
        # GeoJSON receiver
        self.geojson_host = geojson_host
        self.geojson_port = geojson_port
        self.geojson_socket = None
        
        # System state
        self.running = False
        
    def start(self):
        """Start all subsystems"""
        print("="*70)
        print("INTEGRATED TEST SYSTEM")
        print("="*70)
        print(f"Starting at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()
        
        # Start GPS
        print("[1/3] Starting GPS receiver...")
        if self.gps.start():
            print("      GPS thread running")
        else:
            print("      WARNING: GPS not available")
        print()
        
        # Start wind monitor
        print("[2/3] Starting wind monitor...")
        try:
            self.wind_thread = threading.Thread(
                target=wind_monitor_thread,
                args=(self.wind_is_valid, self.current_wind_speed, self.wind_lock,
                      self.wind_check_interval, self.wind_threshold),
                daemon=True
            )
            self.wind_thread.start()
            print(f"      Wind monitor running (port: {self.wind_port})")
            print(f"      Threshold: {self.wind_threshold} knots")
        except Exception as e:
            print(f"      WARNING: Wind monitor failed: {e}")
        print()
        
        # Start GeoJSON receiver
        print("[3/3] Starting GeoJSON receiver...")
        try:
            self.geojson_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.geojson_socket.bind((self.geojson_host, self.geojson_port))
            self.geojson_socket.settimeout(1.0)
            print(f"      Listening on {self.geojson_host}:{self.geojson_port}")
        except Exception as e:
            print(f"      ERROR: Failed to start GeoJSON receiver: {e}")
            return False
        print()
        
        # Give systems time to initialize
        print("Waiting for systems to initialize...")
        time.sleep(2)
        
        print()
        print("="*70)
        print("ALL SYSTEMS READY")
        print("="*70)
        print()
        
        self.running = True
        return True
    
    def stop(self):
        """Stop all subsystems"""
        print("\n\nShutting down all systems...")
        self.running = False
        
        if self.gps:
            self.gps.stop()
            print("✓ GPS stopped")
        
        if self.geojson_socket:
            self.geojson_socket.close()
            print("✓ GeoJSON receiver stopped")
        
        print("✓ Wind monitor stopped (daemon thread)")
        print("\nShutdown complete.")
    
    def print_status(self):
        """Print current system status"""
        print("\n" + "─"*70)
        print(f"System Status - {datetime.now().strftime('%H:%M:%S')}")
        print("─"*70)
        
        # GPS status
        gps_status = self.gps.get_status_string()
        print(f"  {gps_status}")
        
        # Wind status
        with self.wind_lock:
            is_valid = self.wind_is_valid[0]
            wind_speed = self.current_wind_speed[0]
        
        if wind_speed is not None:
            if is_valid:
                print(f"  Wind: SAFE ✓ | Speed={wind_speed:.1f} knots (< {self.wind_threshold} knots)")
            else:
                print(f"  Wind: HIGH ✗ | Speed={wind_speed:.1f} knots (>= {self.wind_threshold} knots)")
        else:
            print(f"  Wind: NO DATA ✗ (sensor error)")
        
        print("─"*70)
    
    def check_launch_conditions(self) -> Tuple[bool, str]:
        """
        Check if conditions are safe for operations
        
        Returns:
            (is_safe, reason)
        """
        # Check GPS
        has_fix, lat, lon, alt, sats = self.gps.get_position()
        if not has_fix:
            return False, "No GPS fix"
        
        # Check wind
        with self.wind_lock:
            is_valid = self.wind_is_valid[0]
            wind_speed = self.current_wind_speed[0]
        
        if wind_speed is None:
            return False, "Wind sensor error"
        
        if not is_valid:
            return False, f"Wind too high ({wind_speed:.1f} knots)"
        
        return True, "All conditions safe"
    
    def run(self):
        """Main run loop"""
        if not self.start():
            return
        
        print("Main loop running... Press Ctrl+C to stop\n")
        
        try:
            packet_count = 0
            
            while self.running:
                try:
                    # Wait for GeoJSON data (with timeout)
                    geojson_bytes, addr = self.geojson_socket.recvfrom(65536)
                    packet_count += 1
                    
                    # Parse GeoJSON
                    json_str = geojson_bytes.decode('utf-8')
                    geojson_data = json.loads(json_str)
                    
                    # Extract coordinates (using your existing function would be better)
                    coordinates = self._extract_coordinates(geojson_data)
                    
                    if coordinates:
                        lon, lat, alt = coordinates[0], coordinates[1], coordinates[2] if len(coordinates) > 2 else 0
                        
                        print("\n" + "="*70)
                        print(f"[PACKET #{packet_count}] Target Received from {addr[0]}:{addr[1]}")
                        print("="*70)
                        print(f"Target Position: Lon={lon:.6f}° Lat={lat:.6f}° Alt={alt:.1f}m")
                        
                        # Print current system status
                        self.print_status()
                        
                        # Check launch conditions
                        is_safe, reason = self.check_launch_conditions()
                        
                        print("\n" + "─"*70)
                        if is_safe:
                            print("✓ LAUNCH AUTHORIZED - All conditions met")
                            print("─"*70)
                            
                            # Get own position
                            has_fix, own_lat, own_lon, own_alt, sats = self.gps.get_position()
                            
                            # Calculate range (simple approximation)
                            if has_fix:
                                lat_diff = lat - own_lat
                                lon_diff = lon - own_lon
                                # Rough distance in meters (1 degree ≈ 111km)
                                distance = ((lat_diff * 111000)**2 + (lon_diff * 111000)**2)**0.5
                                print(f"  Range to target: {distance:.0f} m")
                                print(f"  Altitude difference: {alt - own_alt:.1f} m")
                            
                            print("\n  >>> READY FOR OPERATIONS <<<")
                            
                        else:
                            print(f"✗ LAUNCH PROHIBITED - {reason}")
                            print("─"*70)
                            print("  >>> OPERATIONS ABORTED <<<")
                        
                        print("="*70)
                        
                    else:
                        print(f"[PACKET #{packet_count}] No valid coordinates in GeoJSON")
                    
                except socket.timeout:
                    # No data received, just continue
                    continue
                    
                except json.JSONDecodeError as e:
                    print(f"[ERROR] Invalid JSON received: {e}")
                    
        except KeyboardInterrupt:
            print("\n\nReceived shutdown signal...")
            
        finally:
            self.stop()
    
    def _extract_coordinates(self, geojson: dict) -> Optional[list]:
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


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Integrated GPS + Wind + GeoJSON Test System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s
  %(prog)s --gps-port /dev/ttyUSB0 --wind-port /dev/ttyUSB1
  %(prog)s --wind-threshold 25

Notes:
  - GPS needs clear sky view for fix (go outside)
  - Wind sensor uses Modbus RTU over RS485
  - GeoJSON receiver listens on UDP port 5005
        """
    )
    
    parser.add_argument('--gps-port', default='/dev/ttyUSB0',
                        help='GPS serial port (default: /dev/ttyUSB0)')
    
    parser.add_argument('--wind-port', default='/dev/ttyUSB1',
                        help='Wind sensor serial port (default: /dev/ttyUSB1)')
    
    parser.add_argument('--geojson-host', default='0.0.0.0',
                        help='GeoJSON receiver host (default: 0.0.0.0)')
    
    parser.add_argument('--geojson-port', type=int, default=5005,
                        help='GeoJSON receiver port (default: 5005)')
    
    parser.add_argument('--wind-threshold', type=float, default=20,
                        help='Wind speed threshold in knots (default: 20)')
    
    parser.add_argument('--wind-interval', type=float, default=5,
                        help='Wind check interval in seconds (default: 5)')
    
    args = parser.parse_args()
    
    # Create and run integrated system
    system = IntegratedSystem(
        gps_port=args.gps_port,
        wind_port=args.wind_port,
        geojson_host=args.geojson_host,
        geojson_port=args.geojson_port,
        wind_threshold=args.wind_threshold,
        wind_check_interval=args.wind_interval
    )
    
    system.run()


if __name__ == '__main__':
    main()