import serial
import time
import struct
from datetime import datetime
import csv
import os

def calculate_crc(data):
    """Calculate Modbus CRC16"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def create_modbus_query(slave_addr, function_code, start_reg, num_regs):
    """Create Modbus RTU query with CRC"""
    query = bytes([slave_addr, function_code, 
                   (start_reg >> 8) & 0xFF, start_reg & 0xFF,
                   (num_regs >> 8) & 0xFF, num_regs & 0xFF])
    crc = calculate_crc(query)
    query += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return query

# Configuration
port = '/dev/ttyUSB0'
baudrate = 9600
slave_addr = 2
log_file = 'wind_speed_log.csv'
sample_interval = 1  # seconds

# Statistics tracking
readings = []
max_history = 100  # Keep last 100 readings for statistics

try:
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    
    # Clear any existing data in buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.5)
    
    print(f"Connected to {port}")
    print(f"Logging data to: {log_file}")
    print(f"Sample interval: {sample_interval} second(s)")
    print("-" * 60)
    
    # Create CSV file with headers if it doesn't exist
    file_exists = os.path.isfile(log_file)
    csv_file = open(log_file, 'a', newline='')
    csv_writer = csv.writer(csv_file)
    
    if not file_exists:
        csv_writer.writerow(['Timestamp', 'Wind Speed (knots)', 'Wind Speed (m/s)', 'Raw Value'])
    
    reading_count = 0
    
    while True:
        # Clear buffers before query
        ser.reset_input_buffer()
        
        # Query wind speed register
        query = create_modbus_query(slave_addr, 0x03, 0x0000, 1)
        ser.write(query)
        time.sleep(0.15)  # Increased wait time for response
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            
            if len(response) >= 7:
                # Parse response
                raw_value = struct.unpack('>H', response[3:5])[0]
                wind_speed_ms = raw_value / 10.0
                wind_speed = wind_speed_ms * 1.94384  # Convert m/s to knots
                timestamp = datetime.now()
                
                # Store reading
                readings.append(wind_speed)
                if len(readings) > max_history:
                    readings.pop(0)
                
                # Log to CSV
                csv_writer.writerow([
                    timestamp.strftime('%Y-%m-%d %H:%M:%S'),
                    wind_speed,
                    wind_speed_ms,
                    raw_value
                ])
                csv_file.flush()
                
                reading_count += 1
                
                # Calculate statistics
                current_avg = sum(readings) / len(readings)
                current_max = max(readings)
                current_min = min(readings)
                
                # Display current reading and stats
                print(f"[{timestamp.strftime('%H:%M:%S')}] "
                      f"Wind Speed: {wind_speed:5.1f} kts | "
                      f"Avg: {current_avg:5.1f} | "
                      f"Min: {current_min:5.1f} | "
                      f"Max: {current_max:5.1f} | "
                      f"Samples: {reading_count}")
                
                # Alert for high wind speeds (>20 knots is strong breeze)
                if wind_speed > 20.0:
                    print(f"  ⚠️  HIGH WIND ALERT: {wind_speed:.1f} knots")
                
        else:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] No response from sensor")
        
        time.sleep(sample_interval)

except KeyboardInterrupt:
    print("\n" + "-" * 60)
    print("Stopping data collection...")
    
    if readings:
        print(f"\nSession Statistics:")
        print(f"  Total readings: {reading_count}")
        print(f"  Average wind speed: {sum(readings)/len(readings):.2f} knots")
        print(f"  Maximum wind speed: {max(readings):.2f} knots")
        print(f"  Minimum wind speed: {min(readings):.2f} knots")
    
except Exception as e:
    print(f"Error: {e}")
    
finally:
    if 'csv_file' in locals():
        csv_file.close()
    if 'ser' in locals() and ser.is_open:
        ser.close()
    print(f"\nData saved to: {log_file}")
    print("Serial port closed")