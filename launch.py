#!/usr/bin/env python3
"""
Bottle Rocket Launch Ignition Script - Raspberry Pi 5
Sends a 5V HIGH signal on a GPIO pin to trigger an igniter/e-match.

WIRING:
  - GPIO pin (default: GPIO 17, physical pin 11) → relay/MOSFET gate or directly to igniter circuit
  - GND (physical pin 6 or 9) → igniter circuit ground

SAFETY NOTES:
  - The Pi 5 GPIO outputs 3.3V logic. To switch a true 5V signal to the igniter,
    use a logic-level MOSFET (e.g. 2N7000 / IRLZ44N) or a relay module.
  - Never connect an e-match or igniter directly to a GPIO pin.
  - Keep the igniter circuit physically disconnected until ready to arm.
  - Always point the rocket in a safe direction before running this script.
"""

import time
import sys

try:
    from gpiozero import OutputDevice
except ImportError:
    print("gpiozero not found. Install with: sudo apt install python3-gpiozero")
    sys.exit(1)

# --- Configuration ---
LAUNCH_PIN      = 17      # BCM GPIO pin number
SIGNAL_DURATION = 2.0     # Seconds to hold the signal HIGH (adjust for your igniter)
COUNTDOWN_SECS  = 5       # Pre-launch countdown

def countdown(seconds: int):
    for i in range(seconds, 0, -1):
        print(f"  T-{i}...")
        time.sleep(1)

def launch(pin: int = LAUNCH_PIN, duration: float = SIGNAL_DURATION):
    igniter = OutputDevice(pin, active_high=True, initial_value=False)

    try:
        print("\n=== BOTTLE ROCKET LAUNCH CONTROLLER ===")
        print(f"  Launch pin  : GPIO {pin}")
        print(f"  Signal time : {duration}s")
        confirm = input("\nType FIRE to begin countdown, anything else to abort: ")

        if confirm.strip().upper() != "FIRE":
            print("Launch aborted.")
            return

        print("\nArming...")
        countdown(COUNTDOWN_SECS)

        print("\n🚀 IGNITION!")
        igniter.on()
        time.sleep(duration)
        igniter.off()
        print("Signal complete. Igniter circuit open.")

    finally:
        igniter.off()
        igniter.close()

if __name__ == "__main__":
    launch()