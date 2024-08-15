#Speaker Sound Producing code with DFROBOT #mp3 player

import serial
# Initialize serial communication with DFPlayer Mini
try:
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    ser.flush()
    print("Serial port opened successfully.")
except Exception as e:
    print(f"Error opening serial port: {e}")
def send_command(command, param1=0, param2=0):
    '''Send a command to the DFPlayer Mini'''
    start_byte = 0x7E
    version = 0xFF
    length = 0x06
    end_byte = 0xEF
    data = [start_byte, version, length, command, param1, param2]
    checksum = -(sum(data[1:6])) & 0xFF
    data.append(checksum)
    data.append(end_byte)
    ser.write(bytearray(data))
    print(f"Sent command: {[hex(x) for x in data]}")
def play_sound():
    '''Play sound on the DFPlayer Mini'''
    send_command(0x03, 0x00, 0x01)  # Play track 1

