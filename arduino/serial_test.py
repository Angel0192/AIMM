#!/usr/bin/env python3
import serial
import time


try:
    # Change Serial if program is not registering any commands
    arduino = serial.Serial("/dev/ttyACM2", 9600, timeout=1)
    time.sleep(2)  # wait for Arduino to reset
    print("Connected to Arduino. Type commands to send. Ctrl+C to exit.")
    def send_command(cmd):
        arduino.write((cmd + '\n').encode())
        time.sleep(1)
        while arduino.in_waiting:
            print(arduino.readline().decode().strip())
    # Now enter the interactive loop
    while True:
        # Optionally, you can run initial commands first
        send_command("Forward")
        time.sleep(2)
        send_command("Neutral")
        time.sleep(2)
        send_command("Backward")
        time.sleep(2)

except KeyboardInterrupt:
    send_command("Neutral")
    print("\nProgram interrupted by user. Exiting...")

finally:
    if arduino.is_open:
        arduino.close()
