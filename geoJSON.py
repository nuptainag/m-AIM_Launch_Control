import socket
import json
import threading

# --- Shared State & Locks ---
latest_coords = None
data_lock = threading.Lock()

def extract_coordinates(geojson_data):
    """
    Extract coordinates from GeoJSON data
    Returns: [longitude, latitude, altitude] or None if not found
    """
    try:
        if geojson_data.get('type') == 'FeatureCollection':
            features = geojson_data.get('features', [])
            if features:
                geometry = features[0].get('geometry', {})
                coordinates = geometry.get('coordinates', [])
                if coordinates:
                    return coordinates
                    
        elif geojson_data.get('type') == 'Feature':
            geometry = geojson_data.get('geometry', {})
            coordinates = geometry.get('coordinates', [])
            if coordinates:
                return coordinates
                
        elif geojson_data.get('type') in ['Point', 'LineString', 'Polygon', 'MultiPoint', 'MultiLineString', 'MultiPolygon']:
            coordinates = geojson_data.get('coordinates', [])
            if coordinates:
                return coordinates
                
        return None
        
    except Exception as e:
        print(f"Error extracting coordinates: {e}")
        return None

def _dataListen(ip, port):
    """
    Background thread task. Loops continuously waiting for UDP packets.
    """
    global latest_coords
    
    BUFFER_SIZE = 65536
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    
    print(f"[GeoJSON Listener] Running in background on {ip}:{port}")
    
    while True:
        try:
            # recvfrom is blocking. The thread sleeps here until data arrives!
            geojson_bytes, addr = sock.recvfrom(BUFFER_SIZE)
            
            geojson_string = geojson_bytes.decode('utf-8')
            geojson_data = json.loads(geojson_string)
            
            coordinates = extract_coordinates(geojson_data)
            
            if coordinates:
                # Safely update the shared variable using the lock
                with data_lock:
                    latest_coords = coordinates
                    
        except json.JSONDecodeError:
            pass # Ignore malformed packets silently to keep the thread alive
        except Exception as e:
            print(f"[GeoJSON Listener] Error: {e}")

def start_listener(ip='0.0.0.0', port=5005):
    """
    Starts the daemon listener thread. Called once by launchSystem.
    """
    # daemon=True ensures this thread dies instantly when the main loop exits
    thread = threading.Thread(target=_dataListen, args=(ip, port), daemon=True)
    thread.start()
    return thread

def getDroneCoords():
    """
    Called by the main loop. Safely retrieves the most recent coordinates.
    """
    with data_lock:
        # Return current coordinates (will be None if no packets received yet)
        return latest_coords