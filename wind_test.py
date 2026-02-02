import serial
import time
import struct

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

port = '/dev/ttyUSB0'
baudrate = 9600

try:
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    
    print(f"Connected to {port}")
    print("Reading wind anemometer data...\n")
    
    slave_addr = 2  # Default address from your documentation
    
    while True:
        # Query register 0x0000, read 1 register
        query = create_modbus_query(slave_addr, 0x03, 0x0000, 1)
        
        ser.write(query)
        time.sleep(0.1)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"Response: {response.hex(' ')}")
            
            if len(response) >= 7:
                # Parse response: [addr][func][byte_count][data_high][data_low][crc_low][crc_high]
                raw_value = struct.unpack('>H', response[3:5])[0]  # Unsigned 16-bit
                actual_value = raw_value / 10.0
                
                print(f"Raw value: 0x{raw_value:04X} ({raw_value})")
                print(f"Wind measurement: {actual_value} (degrees or m/s)")
                print()
        else:
            print("No response - checking connection...\n")
        
        time.sleep(1)  # Read every second

except KeyboardInterrupt:
    print("\nStopping...")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")