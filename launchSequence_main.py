import geoJSON
from launchSystem import launchController
import time

# Initialize system
system = launchController()

# --- 1. BYPASS GPS LOCK ---
# system._waitForGPSlock() # Comment this out to prevent the script from hanging while waiting for a signal.
system.GPSready = True     # Manually set this to True so system.launchReady() passes.
system.btConnected = True  # Manually set this so the launch check passes without a Teensy.

authorized = False

def main():
    
    if system.launchReady(): # This will now return True due to the manual flags above.
        
        inRange = system.inRange()
        authorized = False

        trackPredictionOutlook_s = 10 
        totalTime_s = 0
        currentDroneTime_s = 0 
        
        ### Rocket Timing Parameters
        interceptTime_s = 5  
        rocketFlightTime_s = interceptTime_s
        delayTime_s = 0

        while not authorized:
            loopStart = time.perf_counter() 
            
            if not inRange or rocketFlightTime_s > trackPredictionOutlook_s:
                time.sleep(1)
                inRange = system.inRange()
                loopEnd = time.perf_counter()
                totalTime_s += (loopEnd - loopStart)
                continue

            else: 
                # --- 2. BYPASS BLUETOOTH SEND ---
                # The sendCoordinates call can be commented out or left as-is 
                # if you modified launchSystem.py to just print instead of using sockets.
                if not system.sendCoordinates(geoJSON.getDroneCoords()):
                    print("[Sim] Send coordinates bypassed.") #
                
                loopEnd = time.perf_counter()
                currentDroneTime_s = totalTime_s + (loopEnd - loopStart) 
                
                if (trackPredictionOutlook_s - currentDroneTime_s) > rocketFlightTime_s: 
                    delayTime_s = (trackPredictionOutlook_s - currentDroneTime_s) - rocketFlightTime_s 
                    print("\nRocket is", delayTime_s, "seconds away from launch.")
                    time.sleep(delayTime_s) 
                
                    if system.inRange(): 
                        print("Rocket has launched.\n")
                        authorized = True
                        system.executeLaunch()
                    else:
                        continue
                else:
                    print("Rocket has launched.")
                    authorized = True
                    system.executeLaunch()
    else:
        print("Unsuccesful launch.")
             
if __name__ == "__main__":
    try:
        print("System initialized. Press Ctrl+C to abort and exit.")
        while not system.launched:
            main()
    except KeyboardInterrupt:
        print("\n[System] Manual abort detected. Shutting down...")
    finally:
        print("[System] Cleaning up connections...")
        # --- 3. DISABLE BLUETOOTH CLEANUP ---
        # system.closeBluetooth() # Comment this out if no socket was ever opened.
        print("[System] Offline.")