import socket
import json
import os
from datetime import datetime

# --- Configuration ---
HOST = '0.0.0.0'  # Listen on all available interfaces (Ethernet, Wi-Fi)
PORT = 5005
BUFFER_SIZE = 65536  # Maximum size for a UDP datagram (set generously)
SAVE_DIRECTORY = 'received_geojson'  # Directory to save incoming files

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

def receive_data_udp():
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
            # Receive data and the address of the sender
            geojson_bytes, addr = sock.recvfrom(BUFFER_SIZE)
            packet_count += 1
            
            # Decode the received bytes
            geojson_string = geojson_bytes.decode('utf-8')
            geojson_data = json.loads(geojson_string)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]  # milliseconds
            receive_time = datetime.now()  # Store the receive time as datetime object
            filename = f"geojson_{timestamp}_packet{packet_count}.json"
            filepath = os.path.join(SAVE_DIRECTORY, filename)
            
            # Save to file
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(geojson_data, f, indent=2)
            
            # Display header
            print("=" * 60)
            print(f"Packet #{packet_count} - {receive_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            
            # Extract and display timestamp from GeoJSON if it exists
            geojson_timestamp = None
            
            # Check top level
            if 'timestamp' in geojson_data:
                geojson_timestamp = geojson_data['timestamp']
            # Check inside FeatureCollection features
            elif geojson_data.get('type') == 'FeatureCollection':
                features = geojson_data.get('features', [])
                if features and 'properties' in features[0]:
                    geojson_timestamp = features[0]['properties'].get('timestamp')
            # Check inside single Feature
            elif geojson_data.get('type') == 'Feature':
                properties = geojson_data.get('properties', {})
                geojson_timestamp = properties.get('timestamp')
            
            # Display timestamp comparison if found
            if geojson_timestamp:
                geojson_time = datetime.fromtimestamp(geojson_timestamp)
                geojson_time_str = geojson_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                
                # Calculate the difference
                time_diff = (receive_time - geojson_time).total_seconds()
                
                print(f"GeoJSON Timestamp: {geojson_time_str} ({geojson_timestamp})")
                print(f"Time Difference: {time_diff:.6f} seconds ({time_diff*1000:.3f} ms)")
            
            print(f"Received from: {addr[0]}:{addr[1]}")
            print(f"Size: {len(geojson_bytes)} bytes")
            print(f"Saved to: {filepath}")
            print("-" * 60)
            
            # Process and display GeoJSON data
            print_geojson_details(geojson_data)
            
            print("=" * 60)
            print()
            
        except json.JSONDecodeError as e:
            print(f"Error: Received invalid GeoJSON data - {e}")
            print(f"Raw data: {geojson_bytes[:200]}...")  # Show first 200 bytes
            print()
            
        except UnicodeDecodeError as e:
            print(f"Error: Unable to decode data as UTF-8 - {e}")
            print()
            
        except KeyboardInterrupt:
            print("\nShutting down...")
            break
            
        except Exception as e:
            print(f"An error occurred: {e}")
            print()
    
    sock.close()
    print("Socket closed.")

if __name__ == '__main__':
    receive_data_udp()