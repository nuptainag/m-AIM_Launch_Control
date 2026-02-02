import socket
 
# CONFIGURATION

# ---------------------------------------------------------

target_mac_address = "00:0E:EA:CF:7C:DE" 

port = 1

# ---------------------------------------------------------
 
def start_initiator():

    print(f"Connecting to {target_mac_address}...")

    try:

        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        sock.connect((target_mac_address, port))

        print("Connected! You can now send messages to the Teensy.\n")
 
        while True:

            # 1. SEND: The Pi initiates the conversation

            msg = input("Enter message to send: ")

            # (Optional: Allow clean exit)

            if msg.lower() == 'exit':

                break
 
            sock.send(msg.encode('utf-8'))

            print(" -> Sent. Waiting for acknowledgement...")
 
            # 2. WAIT: The script pauses here until Teensy replies

            data = sock.recv(1024)

            if not data:

                print("Connection closed by Teensy.")

                break

            # 3. CONFIRM: Print the acknowledgement

            response = data.decode('utf-8').strip()

            print(f" <- Received from Teensy: '{response}'\n")
 
    except OSError as e:

        print(f"Error: {e}")

    finally:

        sock.close()

        print("Connection closed.")
 
if __name__ == "__main__":

    start_initiator()