import socket
import json
import os
from datetime import datetime

# --- Configuration ---
HOST = '0.0.0.0'  # Listen on all available interfaces (Ethernet, Wi-Fi)
PORT = 5005
BUFFER_SIZE = 65536  # Maximum size for a UDP datagram (set generously)
SAVE_DIRECTORY = 'received_geojson'  # Directory to save incoming files

def extract_coordinates(geojson_data):
    """
    Extract coordinates from GeoJSON data
    Returns: [longitude, latitude, altitude] or None if not found
    """
    try:
        # Handle FeatureCollection
        if geojson_data.get('type') == 'FeatureCollection':
            features = geojson_data.get('features', [])
            if features:
                geometry = features[0].get('geometry', {})
                coordinates = geometry.get('coordinates', [])
                if coordinates:
                    return coordinates
        
        # Handle single Feature
        elif geojson_data.get('type') == 'Feature':
            geometry = geojson_data.get('geometry', {})
            coordinates = geometry.get('coordinates', [])
            if coordinates:
                return coordinates
        
        # Handle direct geometry object
        elif geojson_data.get('type') in ['Point', 'LineString', 'Polygon', 'MultiPoint', 'MultiLineString', 'MultiPolygon']:
            coordinates = geojson_data.get('coordinates', [])
            if coordinates:
                return coordinates
        
        return None
        
    except Exception as e:
        print(f"Error extracting coordinates: {e}")
        return None

def print_geojson_details(geojson_data):
    """
    Read and display detailed information from GeoJSON data
    """
    print(f"GeoJSON Type: {geojson_data.get('type', 'N/A')}")
    
    # Handle different GeoJSON types
    if geojson_data.get('type') == 'FeatureCollection':
        features = geojson_data.get('features', [])
        print(f"Number of Features: {len(features)}")
        
        # Display each feature
        for idx, feature in enumerate(features, 1):
            print(f"\n  Feature {idx}:")
            print(f"    Type: {feature.get('type', 'N/A')}")
            
            # Geometry information
            geometry = feature.get('geometry', {})
            if geometry:
                print(f"    Geometry Type: {geometry.get('type', 'N/A')}")
                coordinates = geometry.get('coordinates', [])
                print(f"    Coordinates: {coordinates}")
            
            # Properties information
            properties = feature.get('properties', {})
            if properties:
                print(f"    Properties:")
                for key, value in properties.items():
                    print(f"      {key}: {value}")
    
    elif geojson_data.get('type') == 'Feature':
        print(f"Feature Type: {geojson_data.get('type', 'N/A')}")
        
        # Geometry information
        geometry = geojson_data.get('geometry', {})
        if geometry:
            print(f"Geometry Type: {geometry.get('type', 'N/A')}")
            coordinates = geometry.get('coordinates', [])
            print(f"Coordinates: {coordinates}")
        
        # Properties information
        properties = geojson_data.get('properties', {})
        if properties:
            print("Properties:")
            for key, value in properties.items():
                print(f"  {key}: {value}")
    
    elif geojson_data.get('type') in ['Point', 'LineString', 'Polygon', 'MultiPoint', 'MultiLineString', 'MultiPolygon']:
        # Direct geometry object
        print(f"Geometry Type: {geojson_data.get('type', 'N/A')}")
        coordinates = geojson_data.get('coordinates', [])
        print(f"Coordinates: {coordinates}")
    
    # Display any additional top-level properties
    other_props = {k: v for k, v in geojson_data.items() 
                   if k not in ['type', 'features', 'geometry', 'properties', 'coordinates']}
    if other_props:
        print("\nAdditional Properties:")
        for key, value in other_props.items():
            print(f"  {key}: {value}")

def receive_and_get_coordinates(sock=None, verbose=True):
    """
    Main function to be called from external script.
    Receives one GeoJSON packet and returns the coordinates.
    
    Args:
        sock: Optional existing socket. If None, creates a new one.
        verbose: If True, prints detailed information. If False, silent mode.
    
    Returns:
        tuple: (coordinates, geojson_data, receive_time, addr)
               coordinates: [lon, lat, alt] or None
               geojson_data: full GeoJSON dict
               receive_time: datetime object
               addr: (ip, port) tuple of sender
    """
    # Create socket if not provided
    close_socket = False
    if sock is None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((HOST, PORT))
        close_socket = True
        if verbose:
            print(f"Listening for UDP datagrams on {HOST}:{PORT}...")
    
    try:
        # Receive data and the address of the sender
        geojson_bytes, addr = sock.recvfrom(BUFFER_SIZE)
        receive_time = datetime.now()
        
        # Decode the received bytes
        geojson_string = geojson_bytes.decode('utf-8')
        geojson_data = json.loads(geojson_string)
        
        # Extract coordinates
        coordinates = extract_coordinates(geojson_data)
        
        if verbose:
            # Save to file
            if not os.path.exists(SAVE_DIRECTORY):
                os.makedirs(SAVE_DIRECTORY)
            
            timestamp = receive_time.strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f"geojson_{timestamp}.json"
            filepath = os.path.join(SAVE_DIRECTORY, filename)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(geojson_data, f, indent=2)
            
            # Display information
            print("=" * 60)
            print(f"Received - {receive_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            
            # Extract and display timestamp from GeoJSON if it exists
            geojson_timestamp = None
            if 'timestamp' in geojson_data:
                geojson_timestamp = geojson_data['timestamp']
            elif geojson_data.get('type') == 'FeatureCollection':
                features = geojson_data.get('features', [])
                if features and 'properties' in features[0]:
                    geojson_timestamp = features[0]['properties'].get('timestamp')
            elif geojson_data.get('type') == 'Feature':
                properties = geojson_data.get('properties', {})
                geojson_timestamp = properties.get('timestamp')
            
            if geojson_timestamp:
                geojson_time = datetime.fromtimestamp(geojson_timestamp)
                geojson_time_str = geojson_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                time_diff = (receive_time - geojson_time).total_seconds()
                print(f"GeoJSON Timestamp: {geojson_time_str} ({geojson_timestamp})")
                print(f"Time Difference: {time_diff:.6f} seconds ({time_diff*1000:.3f} ms)")
            
            print(f"Received from: {addr[0]}:{addr[1]}")
            print(f"Size: {len(geojson_bytes)} bytes")
            print(f"Saved to: {filepath}")
            
            if coordinates:
                print(f"Coordinates: Lon={coordinates[0]}, Lat={coordinates[1]}, Alt={coordinates[2] if len(coordinates) > 2 else 'N/A'}")
            else:
                print("No coordinates found")
            
            print("-" * 60)
            print_geojson_details(geojson_data)
            print("=" * 60)
            print()
        
        return coordinates, geojson_data, receive_time, addr
        
    except json.JSONDecodeError as e:
        if verbose:
            print(f"Error: Received invalid GeoJSON data - {e}")
        return None, None, None, None
        
    except UnicodeDecodeError as e:
        if verbose:
            print(f"Error: Unable to decode data as UTF-8 - {e}")
        return None, None, None, None
        
    except Exception as e:
        if verbose:
            print(f"An error occurred: {e}")
        return None, None, None, None
        
    finally:
        if close_socket:
            sock.close()

def receive_data_udp():
    """
    Standalone receiver that runs continuously (for testing)
    """
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    
    # Create directory for saving files if it doesn't exist
    if not os.path.exists(SAVE_DIRECTORY):
        os.makedirs(SAVE_DIRECTORY)
        print(f"Created directory: {SAVE_DIRECTORY}")
    
    print(f"Listening for UDP datagrams on {HOST}:{PORT}...")
    print(f"Saving files to: {SAVE_DIRECTORY}/")
    print("Press Ctrl+C to stop\n")
    
    packet_count = 0
    
    while True:
        try:
            packet_count += 1
            coordinates, geojson_data, receive_time, addr = receive_and_get_coordinates(sock, verbose=True)
            
            if coordinates:
                print(f"Main program received coordinates: {coordinates}\n")
            
        except KeyboardInterrupt:
            print("\nShutting down...")
            break
    
    sock.close()
    print("Socket closed.")

if __name__ == '__main__':
    receive_data_udp()