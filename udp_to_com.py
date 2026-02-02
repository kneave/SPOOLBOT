import socket
import serial

UDP_PORT = 9000
COM_PORT = "COM31"      # the SECOND port of your pair
BAUD = 115200           # irrelevant for virtual COM, but required

ser = serial.Serial(COM_PORT, BAUD, timeout=0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))

print(f"UDP :{UDP_PORT}  -->  {COM_PORT}")
print("Press Ctrl+C to stop.")

try:
    while True:
        data, addr = sock.recvfrom(4096)
#        print("RX", addr, data[:50])  
        ser.write(data)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    sock.close()
