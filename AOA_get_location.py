import serial
import time

def AOA_get_location():
    print("Reading anchor data")

    serial_con = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
    print(serial_con);

    serial_con.write("AT\r".encode()) #attention
    for i in range(3):
        line = serial_con.readline()
        print(line)


    serial_con.write("AT+GMM\r".encode()) #model ID
    for i in range(3):
        line = serial_con.readline()
        print(line)

    serial_con.write("AT+UMLA=1\r".encode()) #local address
    for i in range(3):
        line = serial_con.readline()
        print(line)

    serial_con.write("ATI9\r".encode()) #software ID
    for i in range(3):
        line = serial_con.readline()
        print(line)

    serial_con.write("AT+UDFENABLE=1\r".encode()) #Direction finding enable (track / angle calc)
    for i in range(3):
        line = serial_con.readline()
        print(line)

    serial_con.write("+UDFFILT\r".encode()) #read response from Direction finding filter
    for i in range(50):
        line = serial_con.readline()
        print(line)

    a = 1
    while a == 1:
        line = serial_con.readline()
        print(line)
        time.sleep(0)

    
    

    






AOA_get_location()
