import serial


ser = serial.Serial('/dev/ttyACM0', 9600)
while True:
    
    val = input("Input distance (in cm):")
    ser.write(str.encode(val))
    
    
