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

def get_wind_speed(port='/dev/ttyUSB0', baudrate=9600, slave_addr=2):
    """
    Read current wind speed from anemometer.
    
    Args:
        port: Serial port (default: /dev/ttyUSB0)
        baudrate: Baud rate (default: 9600)
        slave_addr: Modbus slave address (default: 2)
    
    Returns:
        wind_speed in knots (float) or None if error
    """
    try:
        # Open serial connection
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)
        
        # Query wind speed register
        query = create_modbus_query(slave_addr, 0x03, 0x0000, 1)
        ser.write(query)
        time.sleep(0.15)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            
            if len(response) >= 7:
                # Parse response
                raw_value = struct.unpack('>H', response[3:5])[0]
                wind_speed_ms = raw_value / 10.0
                wind_speed_knots = wind_speed_ms * 1.94384
                
                ser.close()
                return wind_speed_knots
            else:
                ser.close()
                return None
        else:
            ser.close()
            return None
            
    except Exception as e:
        return None

def wind_valid(threshold_knots=20, port='/dev/ttyUSB0', baudrate=9600, slave_addr=2):
    """
    Check if wind speed is valid (safe) for operations.
    
    Args:
        threshold_knots: Wind speed threshold in knots (default: 20)
        port: Serial port (default: /dev/ttyUSB0)
        baudrate: Baud rate (default: 9600)
        slave_addr: Modbus slave address (default: 2)
    
    Returns:
        True if wind speed < threshold_knots (safe)
        False if wind speed >= threshold_knots (too windy) or sensor error
    """
    wind_speed = get_wind_speed(port=port, baudrate=baudrate, slave_addr=slave_addr)
    
    if wind_speed is None:
        # Sensor error - return False for safety
        return False
    
    # Return True only if wind is below threshold
    return wind_speed < threshold_knots

def wind_monitor_thread(wind_is_valid_ref, current_wind_speed_ref, wind_lock, check_interval=5, threshold_knots=20):
    """
    Continuously monitor wind conditions in background.
    Updates shared wind_is_valid and current_wind_speed.
    
    Args:
        wind_is_valid_ref: Shared list [bool] for validity status
        current_wind_speed_ref: Shared list [float] for wind speed
        wind_lock: threading.Lock() for thread safety
        check_interval: Seconds between checks (default: 5)
        threshold_knots: Wind threshold in knots (default: 20)
    """
    print("[Wind Monitor] Starting wind monitoring thread...")
    
    while True:
        try:
            # Get wind speed
            speed = get_wind_speed()
            is_valid = wind_valid(threshold_knots=threshold_knots)
            
            # Update shared state
            with wind_lock:
                current_wind_speed_ref[0] = speed
                wind_is_valid_ref[0] = is_valid
            
            # Print status
            if speed is not None:
                status = "SAFE" if is_valid else "NO-GO"
                print(f"[Wind Monitor]{status}")
            else:
                print("[Wind Monitor] Sensor error")
            
        except Exception as e:
            print(f"[Wind Monitor] Error: {e}")
        
        time.sleep(check_interval)