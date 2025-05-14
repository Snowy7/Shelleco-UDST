import serial
import time

# Find the correct port - typically /dev/ttyTHS0 or similar for Jetson hardware UART
# Use 'ls /dev/tty*' to list available ports
serial_port = '/dev/ttyTHS1'  # Change this to match your Jetson's UART port
baud_rate = 9600

try:
    # Open serial port
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(f"Connected to {serial_port} at {baud_rate} baud")
    
    while True:
        try:
            # Take angle input from the user
            angle = input("Enter an angle (0-180): ")
            
            # Validate the input
            if not angle.isdigit():
                print("Invalid input. Please enter a number between 0 and 180.")
                continue
            
            angle = int(angle)
            if angle < 0 or angle > 180:
                print("Angle out of range. Please enter a number between 0 and 180.")
                continue
            
            # Send the angle to Arduino
            ser.write((str(angle) + '\n').encode())
            print(f"Sent: {angle}")
            
            # Read data from Arduino
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(f"Received: {line}")
        
        except ValueError:
            print("Invalid input. Please enter a valid number.")
                
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
except KeyboardInterrupt:
    print("Program terminated by user")
finally:
    # Close serial port if it's open
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
