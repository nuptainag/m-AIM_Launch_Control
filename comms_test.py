import socket
import time

# CONFIGURATION
# ---------------------------------------------------------
# Replace this with your Grove Bluetooth MAC address
target_mac_address = "00:0E:EA:CF:7C:DE" 
# Standard RFCOMM port is usually 1 for these modules
port = 1 
# ---------------------------------------------------------

def send_message(message):
    print(f"Attempting to connect to {target_mac_address}...")
    
    try:
        # Create a Bluetooth RFCOMM socket
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        # Connect to the device
        sock.connect((target_mac_address, port))
        print("Connected!")

        # Send the data (must be encoded to bytes)
        # Adding a newline character (\n) is good practice for text protocols
        full_message = message + "\n"
        sock.send(full_message.encode('utf-8'))
        print(f"Sent: {message}")

        # Wait a moment to ensure transmission before closing
        time.sleep(1)
        
        # Close the connection
        sock.close()
        print("Connection closed.")

    except OSError as e:
        print(f"Error: Could not connect. Make sure the device is paired and on.")
        print(f"System details: {e}")

if __name__ == "__main__":
    text_to_send = input("Enter message to send to Teensy: ")
    send_message(text_to_send)