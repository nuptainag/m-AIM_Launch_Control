# main_test.py
import socket
import time
import threading
from geojson_receiver import receive_and_get_coordinates, HOST, PORT
from wind_logger import wind_monitor_thread

def main():
    # Shared state between threads (using lists for mutability)
    wind_is_valid = [False]
    current_wind_speed = [None]
    wind_lock = threading.Lock()
    
    # Start wind monitoring thread
    wind_thread = threading.Thread(
        target=wind_monitor_thread, 
        args=(wind_is_valid, current_wind_speed, wind_lock, 5, 20),
        daemon=True
    )
    wind_thread.start()
    
    # Give wind monitor a moment to get first reading
    time.sleep(1)
    
    # Create socket for GeoJSON receiver
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    
    print("Main program starting...")
    print(f"Listening on {HOST}:{PORT}\n")
    
    try:
        while True:
            # Get coordinates from receiver
            coordinates, geojson_data, receive_time, addr = receive_and_get_coordinates(sock, verbose=False)
            
            if coordinates:
                lon, lat, alt = coordinates[0], coordinates[1], coordinates[2] if len(coordinates) > 2 else 0
                print(f"\n[GeoJSON] Got coordinates: Lon={lon}, Lat={lat}, Alt={alt}")
                
                # Check current wind conditions
                with wind_lock:
                    is_valid = wind_is_valid[0]
                    wind_speed = current_wind_speed[0]
                
                if is_valid:
                    print(f"[Main] Wind is safe ({wind_speed:.1f} knots) - proceeding with operations")
                    # Do your operations here
                    
                else:
                    if wind_speed is not None:
                        print(f"[Main] Wind too high ({wind_speed:.1f} knots) - aborting operations")
                    else:
                        print("[Main] Wind sensor error - aborting operations")
            else:
                print("[GeoJSON] No coordinates received")
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        sock.close()
        print("Socket closed.")

if __name__ == '__main__':
    main()