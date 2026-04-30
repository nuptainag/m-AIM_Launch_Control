import rocket
import geoJSON
import hardware
import time
import socket

class launchController:
    
    def __init__(self):
        
        self.maxWind = 20  # knots
        
        self.launchCoords = None
        self.GPSready = False
        self.launchAngle_deg = 45
        self.launched = False
        
        self.rocketParameters = {
            "initialMass": 1.06477,
            "propellantMass": 0.070,
            "burnTime": 1.1,
            "avgThrust": 127.4,
            "Cdo": 0.07717,
            "area_m2": 0.0552
        }
        
        self.macAddress = "00:0E:EA:CF:7C:DE"
        self.bluetoothPort = 1
        self.bt_sock = None
        self.btConnected = False
        
        self.connectBluetooth()        # Bug 1 fixed — calls instance method now
        
        self.ip = "0.0.0.0"
        self.port = 5005
        self.thread = geoJSON.start_listener(self.ip, self.port)
    
    def connectBluetooth(self):        # Bug 1 fixed — moved into class
        print("Bluetooth connection bypassed (Simulation Mode).")
        self.btConnected = True
        # print(f"Attempting to connect to Teensy at {self.macAddress}...")
        # try:
        #     self.bt_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        #     self.bt_sock.connect((self.macAddress, self.bluetoothPort))
        #     self.bt_sock.setblocking(False)
        #     self.btConnected = True
        #     print("Bluetooth connected successfully.")
        # except OSError as e:
        #     print(f"Failed to connect Bluetooth: {e}")
        #     self.btConnected = False

    def _waitForGPSlock(self):
        print("Bypassing GPS lock... using hardcoded coordinates.")
        # Provide manual USAFA coordinates [lat, lon, alt]
        self.launchCoords = [38.99, -104.86, 2163] 
        self.GPSready = True
        # while self.launchCoords is None:
        #     self.launchCoords = hardware.getGPS()
        #     if self.launchCoords is None:
        #         print("Retrying GPS lock.")
        #         time.sleep(1)
        #     else:
        #         self.GPSready = True

    def _windInLimits(self):
        currentWind = hardware.getWind()
        return currentWind < self.maxWind

    def inRange(self):
        
        targetCoords = geoJSON.getDroneCoords()
        if targetCoords is None:
            return False
        launchDirection_deg = rocket.calculateLaunchDirection(self.launchCoords, targetCoords)
        inRange = rocket.checkRange(self.rocketParameters, self.launchCoords, targetCoords, self.launchAngle_deg, launchDirection_deg)
        return inRange

    def sendCoordinates(self, targetCoords):
        # Comment out the socket send and just print to console
        print(f"[Sim Mode] Would send to Teensy: {targetCoords}")
        return True
        # if not self.btConnected:                    # Bug 2 fixed
        #     print("Cannot send coords. Bluetooth not connected.")
        #     return False
        # try:
        #     csv_string = f"{targetCoords[0]},{targetCoords[1]},{targetCoords[2]}\n"
        #     self.bt_sock.send(csv_string.encode('utf-8'))
        #     print(f"Sent via BT: {csv_string.strip()}")
        #     return True
        # except OSError as e:
        #     print(f"Bluetooth connection lost during send: {e}")
        #     self.btConnected = False                # Bug 2 fixed
        #     self.bt_sock.close()
        #     return False

    def closeBluetooth(self):
        if self.btConnected and self.bt_sock:
            self.bt_sock.close()
            print("Bluetooth connection closed cleanly.")

    def executeLaunch(self):
        print("Launch successful.")
        self.launched = True

    def launchReady(self):
        return True if self._windInLimits() and self.GPSready and self.btConnected else False