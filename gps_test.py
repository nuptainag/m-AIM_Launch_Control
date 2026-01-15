#!/usr/bin/env python3
"""
GPS Receiver Test Script for Raspberry Pi 5
Tests GPS module connection and displays real-time position data

Hardware Setup:
- GPS module connected via UART (default: /dev/ttyAMA0)
- OR using gpsd daemon

Usage:
  python3 gps_test.py                    # Use gpsd daemon
  python3 gps_test.py --serial           # Use direct serial connection
  python3 gps_test.py --serial --port /dev/ttyUSB0  # Custom port
"""

import sys
import time
import serial
import argparse
from datetime import datetime
from typing import Optional

# Try to import gpsd library
try:
    from gpsd import connect as gpsd_connect, get_current
    GPSD_AVAILABLE = True
except ImportError:
    GPSD_AVAILABLE = False
    print("Note: gpsd-py3 not installed. Use --serial for direct serial connection.")
    print("Install with: pip3 install gpsd-py3\n")


class GPSTest:
    """GPS receiver test class"""
    
    def __init__(self, use_gpsd=True, serial_port='/dev/ttyAMA0', baudrate=9600):
        self.use_gpsd = use_gpsd and GPSD_AVAILABLE
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        self.fix_count = 0
        self.sentence_count = 0
        
    def start(self):
        """Initialize GPS connection"""
        print("="*70)
        print("GPS Receiver Test")
        print("="*70)
        
        if self.use_gpsd:
            print(f"\nMode: Using gpsd daemon")
            print("Connecting to gpsd...")
            try:
                gpsd_connect()
                print("✓ Connected to gpsd successfully")
                print("\nIf gpsd is not running, start it with:")
                print("  sudo systemctl start gpsd")
                print("  sudo systemctl enable gpsd")
                return True
            except Exception as e:
                print(f"✗ Failed to connect to gpsd: {e}")
                print("\nTroubleshooting:")
                print("  1. Check if gpsd is running: sudo systemctl status gpsd")
                print("  2. Start gpsd: sudo systemctl start gpsd")
                print("  3. Or try --serial mode for direct connection")
                return False
        else:
            print(f"\nMode: Direct serial connection")
            print(f"Port: {self.serial_port}")
            print(f"Baudrate: {self.baudrate}")
            print("\nOpening serial port...")
            try:
                self.serial_conn = serial.Serial(
                    self.serial_port,
                    baudrate=self.baudrate,
                    timeout=1.0
                )
                print("✓ Serial port opened successfully")
                print("\nIf connection fails, check:")
                print("  1. GPS module is powered")
                print("  2. Correct port (try: ls -l /dev/tty* | grep USB)")
                print("  3. User permissions: sudo usermod -a -G dialout $USER")
                return True
            except Exception as e:
                print(f"✗ Failed to open serial port: {e}")
                print("\nTroubleshooting:")
                print("  1. Check available ports: ls -l /dev/tty*")
                print("  2. Common ports: /dev/ttyAMA0, /dev/ttyUSB0, /dev/ttyACM0")
                print("  3. Check permissions: sudo chmod 666 " + self.serial_port)
                return False
    
    def parse_nmea_gga(self, sentence: str) -> Optional[dict]:
        """Parse NMEA GGA sentence (Global Positioning System Fix Data)"""
        try:
            parts = sentence.strip().split(',')
            
            # Validate sentence
            if not (sentence.startswith('$GPGGA') or sentence.startswith('$GNGGA')):
                return None
            
            if len(parts) < 15:
                return None
            
            # Extract fields
            time_str = parts[1]  # UTC time (hhmmss.ss)
            lat_str = parts[2]   # Latitude (ddmm.mmmm)
            lat_dir = parts[3]   # N/S
            lon_str = parts[4]   # Longitude (dddmm.mmmm)
            lon_dir = parts[5]   # E/W
            fix_quality = parts[6]  # 0=invalid, 1=GPS fix, 2=DGPS fix
            num_sats = parts[7]  # Number of satellites
            hdop = parts[8]      # Horizontal dilution of precision
            altitude = parts[9]  # Altitude above mean sea level
            alt_units = parts[10]  # M = meters
            
            # Parse fix quality
            fix_qual = int(fix_quality) if fix_quality else 0
            if fix_qual == 0:
                return {'fix': False, 'quality': 0}
            
            # Parse latitude (ddmm.mmmm -> decimal degrees)
            if lat_str and lat_dir:
                lat_deg = float(lat_str[:2])
                lat_min = float(lat_str[2:])
                lat = lat_deg + (lat_min / 60.0)
                if lat_dir == 'S':
                    lat = -lat
            else:
                lat = 0.0
            
            # Parse longitude (dddmm.mmmm -> decimal degrees)
            if lon_str and lon_dir:
                lon_deg = float(lon_str[:3])
                lon_min = float(lon_str[3:])
                lon = lon_deg + (lon_min / 60.0)
                if lon_dir == 'W':
                    lon = -lon
            else:
                lon = 0.0
            
            # Parse altitude
            alt = float(altitude) if altitude else 0.0
            
            # Parse time
            if time_str and len(time_str) >= 6:
                hours = int(time_str[0:2])
                minutes = int(time_str[2:4])
                seconds = float(time_str[4:])
                utc_time = f"{hours:02d}:{minutes:02d}:{seconds:05.2f}"
            else:
                utc_time = "N/A"
            
            return {
                'fix': True,
                'quality': fix_qual,
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'satellites': int(num_sats) if num_sats else 0,
                'hdop': float(hdop) if hdop else 0.0,
                'utc_time': utc_time,
                'raw_sentence': sentence.strip()
            }
            
        except Exception as e:
            # Don't print errors for every malformed sentence
            return None
    
    def parse_nmea_rmc(self, sentence: str) -> Optional[dict]:
        """Parse NMEA RMC sentence (Recommended Minimum Navigation Information)"""
        try:
            parts = sentence.strip().split(',')
            
            if not (sentence.startswith('$GPRMC') or sentence.startswith('$GNRMC')):
                return None
            
            if len(parts) < 12:
                return None
            
            status = parts[2]  # A=active, V=void
            if status != 'A':
                return {'fix': False}
            
            # Parse speed and course
            speed_knots = float(parts[7]) if parts[7] else 0.0
            speed_ms = speed_knots * 0.514444  # Convert knots to m/s
            speed_kmh = speed_knots * 1.852    # Convert knots to km/h
            
            course = float(parts[8]) if parts[8] else 0.0
            
            return {
                'fix': True,
                'speed_ms': speed_ms,
                'speed_kmh': speed_kmh,
                'speed_knots': speed_knots,
                'course': course
            }
            
        except Exception as e:
            return None
    
    def read_gpsd(self) -> Optional[dict]:
        """Read GPS data from gpsd daemon"""
        try:
            packet = get_current()
            
            # Check fix mode: 0=no fix, 2=2D fix, 3=3D fix
            if packet.mode < 2:
                return {'fix': False, 'mode': packet.mode}
            
            return {
                'fix': True,
                'mode': packet.mode,
                'lat': packet.lat,
                'lon': packet.lon,
                'alt': packet.alt if packet.mode == 3 else 0.0,
                'speed_ms': packet.hspeed if hasattr(packet, 'hspeed') else 0.0,
                'course': packet.track if hasattr(packet, 'track') else 0.0,
                'satellites': getattr(packet, 'sats', 0),
                'utc_time': packet.time if hasattr(packet, 'time') else 'N/A'
            }
            
        except Exception as e:
            return None
    
    def read_serial(self) -> Optional[dict]:
        """Read GPS data from serial port"""
        try:
            line = self.serial_conn.readline().decode('ascii', errors='ignore')
            self.sentence_count += 1
            
            if not line:
                return None
            
            # Parse different NMEA sentences
            if '$GPGGA' in line or '$GNGGA' in line:
                return self.parse_nmea_gga(line)
            elif '$GPRMC' in line or '$GNRMC' in line:
                return self.parse_nmea_rmc(line)
            
            return None
            
        except Exception as e:
            print(f"\nSerial read error: {e}")
            return None
    
    def display_data(self, data: dict):
        """Display GPS data"""
        if not data:
            return
        
        # Clear screen (optional, comment out if it causes issues)
        # print("\033[2J\033[H", end="")
        
        print("\n" + "="*70)
        print(f"GPS Data - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*70)
        
        if not data.get('fix', False):
            print("\n❌ NO GPS FIX")
            if 'mode' in data:
                mode_str = {0: "No fix", 1: "No fix", 2: "2D fix", 3: "3D fix"}
                print(f"   Mode: {mode_str.get(data['mode'], 'Unknown')}")
            if 'quality' in data:
                print(f"   Quality: {data['quality']}")
            print("\n   Waiting for satellite lock...")
            print("   This may take 30-60 seconds outdoors with clear sky view")
            if self.use_gpsd:
                print(f"\n   Fixes received: {self.fix_count}")
            else:
                print(f"\n   NMEA sentences processed: {self.sentence_count}")
            return
        
        # Fix acquired!
        self.fix_count += 1
        print("\n✓ GPS FIX ACQUIRED")
        
        # Position
        print(f"\n📍 Position:")
        print(f"   Latitude:  {data.get('lat', 0.0):11.6f}°")
        print(f"   Longitude: {data.get('lon', 0.0):11.6f}°")
        print(f"   Altitude:  {data.get('alt', 0.0):8.1f} m")
        
        # Navigation (if available)
        if 'speed_ms' in data or 'course' in data:
            print(f"\n🧭 Navigation:")
            if 'speed_ms' in data:
                speed_ms = data.get('speed_ms', 0.0)
                speed_kmh = data.get('speed_kmh', speed_ms * 3.6)
                print(f"   Speed:     {speed_ms:6.2f} m/s  ({speed_kmh:6.2f} km/h)")
            if 'course' in data:
                print(f"   Course:    {data.get('course', 0.0):6.1f}°")
        
        # Quality indicators
        print(f"\n📊 Quality:")
        if 'satellites' in data:
            print(f"   Satellites: {data.get('satellites', 0)}")
        if 'hdop' in data:
            hdop = data.get('hdop', 0.0)
            hdop_quality = "Excellent" if hdop < 2 else "Good" if hdop < 5 else "Moderate" if hdop < 10 else "Poor"
            print(f"   HDOP:       {hdop:.1f} ({hdop_quality})")
        if 'mode' in data:
            mode_str = {2: "2D fix", 3: "3D fix"}
            print(f"   Fix mode:   {mode_str.get(data['mode'], 'Unknown')}")
        if 'quality' in data:
            qual_str = {1: "GPS fix", 2: "DGPS fix", 4: "RTK fix", 5: "Float RTK"}
            print(f"   Quality:    {qual_str.get(data['quality'], 'Unknown')}")
        
        # Time
        if 'utc_time' in data:
            print(f"\n🕐 UTC Time:  {data['utc_time']}")
        
        # Statistics
        print(f"\n📈 Statistics:")
        print(f"   Fixes received: {self.fix_count}")
        if not self.use_gpsd:
            print(f"   NMEA sentences: {self.sentence_count}")
        
        # Raw data (optional)
        if 'raw_sentence' in data and False:  # Set to True to see raw NMEA
            print(f"\n📝 Raw NMEA:")
            print(f"   {data['raw_sentence']}")
        
        print("="*70)
    
    def run(self):
        """Main test loop"""
        if not self.start():
            return
        
        print("\n" + "="*70)
        print("GPS TEST RUNNING")
        print("="*70)
        print("\nPress Ctrl+C to stop")
        print("\nWaiting for GPS data...")
        if not self.use_gpsd:
            print("Listening for NMEA sentences...")
        print()
        
        self.running = True
        last_display = 0
        
        try:
            while self.running:
                # Read GPS data
                if self.use_gpsd:
                    data = self.read_gpsd()
                else:
                    data = self.read_serial()
                
                # Display update (limit to once per second)
                current_time = time.time()
                if data and (current_time - last_display) >= 1.0:
                    self.display_data(data)
                    last_display = current_time
                
                # Small delay
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\nTest stopped by user")
        except Exception as e:
            print(f"\n\nError: {e}")
        finally:
            self.stop()
    
    def stop(self):
        """Clean up"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        print("\n✓ GPS test completed")
        print(f"✓ Total fixes received: {self.fix_count}\n")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Test GPS receiver on Raspberry Pi 5',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Use gpsd daemon (default)
  %(prog)s --serial                 # Direct serial connection
  %(prog)s --serial --port /dev/ttyUSB0  # Custom serial port
  %(prog)s --serial --baud 115200   # Custom baud rate

Notes:
  - GPS needs clear view of sky for initial fix (30-60 seconds)
  - For serial mode, ensure user is in dialout group: sudo usermod -a -G dialout $USER
  - For gpsd mode, ensure gpsd is running: sudo systemctl start gpsd
        """
    )
    
    parser.add_argument(
        '--serial',
        action='store_true',
        help='Use direct serial connection instead of gpsd'
    )
    
    parser.add_argument(
        '--port',
        default='/dev/ttyAMA0',
        help='Serial port (default: /dev/ttyAMA0)'
    )
    
    parser.add_argument(
        '--baud',
        type=int,
        default=9600,
        help='Baud rate (default: 9600)'
    )
    
    args = parser.parse_args()
    
    # Create and run GPS test
    use_gpsd = not args.serial
    
    if use_gpsd and not GPSD_AVAILABLE:
        print("Error: gpsd-py3 module not installed")
        print("Install with: pip3 install gpsd-py3")
        print("Or use --serial for direct connection")
        sys.exit(1)
    
    gps_test = GPSTest(
        use_gpsd=use_gpsd,
        serial_port=args.port,
        baudrate=args.baud
    )
    
    gps_test.run()


if __name__ == '__main__':
    main()