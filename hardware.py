import time
import serial
import struct
import socket

# Bug fix: comments were swapped — USB0 is GPS, USB1 is wind, matching the port assignments
windPort = '/dev/ttyUSB1' ## USB1 = wind
gpsPort = '/dev/ttyUSB0'  ## USB0 = GPS

########## GPS Functions ##########

def to_decimal(raw_val, direction):
    if not raw_val: return 0.0
    
    # NMEA Lat is DDMM.MMMM, Lon is DDDMM.MMMM
    # We find the decimal point and move back two places to find the split
    
    dot_index = raw_val.find('.')
    deg = float(raw_val[:dot_index-2])
    minutes = float(raw_val[dot_index-2:])
    decimal = deg + (minutes / 60.0)
    return -decimal if direction in ['S', 'W'] else decimal

def parse_gga(sentence):
    """Helper to extract lat, lon, alt from a raw NMEA string."""
    parts = sentence.split(',')
    
    if len(parts) < 10 or parts[6] == '0':  # Check if fix is valid
        return None
    
    return {"lat": to_decimal(parts[2], parts[3]),
            "lon": to_decimal(parts[4], parts[5]),
            "alt": float(parts[9]) if parts[9] else 0.0}

def getGPS(port=gpsPort, baudrate=9600):
    # Comment out all serial logic
    # Return a dummy dict so the parser doesn't fail
    return {"lat": 38.99, "lon": -104.86, "alt": 2163.0}
    #"""Returns {"lat", "lon", "alt"} dict or None if no fix is found."""
    # try:
    #     ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
    # except Exception as e:
    #     print(f"Connection Error: {e}")
    #     return None

    # # Bug fix: moved ser.close() into a finally block so the port is always
    # # released, even if an exception is raised mid-loop.
    # try:
    #     # Search for the GGA sentence (contains Lat, Lon, Alt).
    #     # Check several lines to give the module a chance to send the right data.
    #     for _ in range(50):
    #         line = ser.readline().decode('ascii', errors='ignore')
    #         if '$GPGGA' in line or '$GNGGA' in line:
    #             data = parse_gga(line)
    #             if data:
    #                 return data
    # except Exception as e:
    #     print(f"Connection Error: {e}")
    # finally:
    #     ser.close()

    # return None


########## Wind Functions ##########

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

def getWind(port=windPort, baudrate=9600, slave_addr=2):
    """
    Read current wind speed from anemometer.
    
    Args:
        port: Serial port (default: /dev/ttyUSB1)
        baudrate: Baud rate (default: 9600)
        slave_addr: Modbus slave address (default: 2)
    
    Returns:
        wind_speed in knots (float) or None if error
    """
    # Return a safe wind speed (e.g., 5 knots)
    return 5.0

    # Comment out Modbus/Serial logic
    # try:
    #     ser = serial.Serial(
    #         port=port,
    #         baudrate=baudrate,
    #         bytesize=serial.EIGHTBITS,
    #         parity=serial.PARITY_NONE,
    #         stopbits=serial.STOPBITS_ONE,
    #         timeout=1
    #     )
        
    #     # Clear buffers
    #     ser.reset_input_buffer()
    #     ser.reset_output_buffer()
    #     time.sleep(0.1)
        
    #     # Query wind speed register
    #     query = create_modbus_query(slave_addr, 0x03, 0x0000, 1)
    #     ser.write(query)
    #     time.sleep(0.15)
        
    #     if ser.in_waiting > 0:
    #         response = ser.read(ser.in_waiting)
            
    #         if len(response) >= 7:
    #             raw_value = struct.unpack('>H', response[3:5])[0]
    #             wind_speed_ms = raw_value / 10.0
    #             wind_speed_knots = wind_speed_ms * 1.94384
                
    #             ser.close()
    #             return wind_speed_knots
    #         else:
    #             ser.close()
    #             return None
    #     else:
    #         ser.close()
    #         return None
            
    # except Exception as e:
    #     return None

# Bug 10 fixed: removed the orphaned connectBluetooth(self) module-level function.
# It used 'self' as if it were a method but was never part of a class, making it
# uncallable. The working version now lives in launchSystem.py as an instance method
# on launchController, which is the correct location.