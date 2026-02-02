import socket
import time

# CONFIGURATION
# ---------------------------------------------------------
# Replace this with your Grove Bluetooth MAC address
target_mac_address = "00:0E:EA:CF:7C:DE" 
# Standard RFCOMM port is usually 1 for these modules
port = 1 
# ---------------------------------------------------------

def receive_messages():
    """Connect to the Grove Bluetooth module and receive messages."""
    
    print(f"Attempting to connect to {target_mac_address}...")
    
    try:
        # Create a Bluetooth RFCOMM socket
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        # Connect to the device
        sock.connect((target_mac_address, port))
        print("Connected!")
        print("Listening for messages... (Press Ctrl+C to stop)\n")
        
        # Continuously receive data
        buffer = ""
        while True:
            try:
                # Receive data in small chunks
                data = sock.recv(1024)
                
                if not data:
                    print("\nConnection closed by device.")
                    break
                
                # Decode and add to buffer
                buffer += data.decode('utf-8')
                
                # Process complete messages (separated by newlines)
                while '\n' in buffer:
                    message, buffer = buffer.split('\n', 1)
                    if message.strip():
                        print(f"Received: {message.strip()}")
                
            except socket.timeout:
                continue
            except UnicodeDecodeError:
                print("Warning: Received non-text data")
                continue
                
    except KeyboardInterrupt:
        print("\n\nStopping receiver...")
    
    except OSError as e:
        print(f"Error: Could not connect. Make sure the device is paired and on.")
        print(f"System details: {e}")
    
    finally:
        try:
            sock.close()
        except:
            pass
        print("Connection closed.")

if __name__ == "__main__":
    print("=== Bluetooth Message Receiver ===\n")
    receive_messages()