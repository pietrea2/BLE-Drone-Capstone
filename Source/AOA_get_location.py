import serial
import time
import serial.tools.list_ports
import triangulation



#com_conneciton = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1)

def AOA_get_location():
    print("starting1")
    print("Reading anchor data")

    foundPorts = get_ports()        
    connectPort = findArduino(foundPorts)
    serial_connections = []

    #print(foundPorts)
    print(connectPort)

    for COM_port in connectPort:
        serial_con = serial.Serial(COM_port, 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
        serial_connections.append(serial_con)

    #Connect to correct COM port for uBlox AOA
    #Adam's COM port :^)
    #serial_con = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);

    #Mohammad's COM port :-)
    #serial_con = serial.Serial('COM3', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
    #print(serial_con);

    for index, con in enumerate(serial_connections):
        anchor_setup_ver3(index, con)


    a = 1
    while a == 1:

        #ancho1_line = serial_connections[0].readline()

        anchor_read_lines = []
        for serial_con_read in serial_connections:
            anchor_read_lines.append( serial_con_read.readline() )
        
        #line = serial_con.readline()
        #print(line)
        #time.sleep(0)



        #split_data format:
        #split_data = line.decode('utf8').strip('\r\n').split(',')
        #[0] = <ed_instance_id>           ---> 6-byte Eddystone instance id
        #[1] = <rssi>                     ---> RSSI = Received Signal Strength Indicator,
        #[2] = <angle_azimuth>            ---> Azimuth angle in range -90 to 90°
        #[3] = <angle_elevation>          ---> Elevation angle in range -90 to 90°
        #[4] = <notused>
        #[5] = <channel>                  ---> Channel from which the packet angle was calculated
        #[6] = <anchor_id>                ---> The value set by +UDFCFG param_tag 4
        #[7] = <user_defined_str>         ---> The value set by +UDFCFG param_tag 2
        #[8] = <timestamp_ms>             ---> Time since boot in milliseconds
        #[9] = <periodic_event_counter>   ---> Periodic event counter value of the periodic advertising packet sent from a tag
        

        anchor_azimuths = []
        anchor_elevations = []
        for line in anchor_read_lines:
            split_data = line.decode('utf8').strip('\r\n').split(',')

            #print(split_data)
            #print(len(split_data))
            if( len(split_data) == 9 ): #check for empty lines of \n\r
                #print(split_data)
                #print(split_data[0]) #6-byte Eddystone instance id of tag
                #print(split_data[2]) #RSSI of 1st polarization

                #2 calculated angles, between -90 and 90 degrees
               

                azimuth = float(split_data[2])
                elevation = float(split_data[3])
                #print("Azimuth:   " + split_data[2] + "Elevation:  " + split_data[3])
                anchor_azimuths.append(azimuth)
                anchor_elevations.append(elevation)
                time.sleep(0.01)

    
                #print(split_data)

        if len(anchor_azimuths) != 0:
            #print("Anchor 0 Azimuth: " + anchor_azimuths[0] + "   Elevation:  " + anchor_elevations[0] + " Anchor 1 Azimuth:   " + anchor_azimuths[1] + "Elevation:  " + anchor_elevations[1])
            
            #TESTING SUN OUTPUT
            #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Elevation: " + str(anchor_elevations[0]));


            #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Elevation: " + str(anchor_elevations[0]) + "           2: Azimuth: " + str(anchor_azimuths[1]) + " Elevation: " + str(anchor_elevations[1]))

            
            
            #print("Anchor 0 Azimuth: " + anchor_azimuths[0] + " Anchor 1 Azimuth:   " + anchor_azimuths[1] + " Anchor 0 Elevation:  " + anchor_elevations[0] + " Anchor 1 Elevation:  " + anchor_elevations[1])    
            #print("Anchor 0 Azimuth: " + anchor_azimuths[0] + " Anchor 1 Azimuth:   " + anchor_azimuths[1] + " Anchor 0 Elevation:  " + anchor_elevations[0] + " Anchor 1 Elevation:  " + anchor_elevations[1])    
        
            


            #TODO:
            #Double check if Elevation angles are the same/similar between all anchors
            #def triangulation(num_of_anchors, pos_anchor_1, pos_anchor_2, azimuth1, azimuth2, elevation):

            #TODO:
            #Re-write triangulate function for 2+ anchors

            
            x, y, z, x1, y2, z3 = triangulation.triangulation(len(serial_connections), [1.5, 0.0], [0.0, 0.0], anchor_azimuths[0], anchor_azimuths[1], anchor_elevations[0])
            #x_fuck, y_fuck, z_fuck, x1_fuck, y2_fuck, z3_fuck = triangulation.triangulation(len(serial_connections), [0.0, 0.0], [1.0, 0.0], anchor_azimuths[1], anchor_azimuths[0], anchor_elevations[0])

            #Rounded location values
            number_of_decimals = 5
            x_round, y_round, z_round = round(x, number_of_decimals), round(y, number_of_decimals), round(z, number_of_decimals)
            x1_round, y1_round, z1_round = round(x1, number_of_decimals), round(y2, number_of_decimals), round(z3, number_of_decimals)
            
            #y_fucking_rounded = round(y_fuck, number_of_decimals)

            #print("Calculated cartesian coordinates Method1: " , x, y, z)
            #print("Calculated cartesian coordinates Method2: " , x1, y2, z3, "\n")

            #PRINT THE METHOD 2 VALS:
            #print("Calculated cartesian coordinates Method1: " , x_round, y_fucking_rounded, z_round)
            print("Calculated cartesian coordinates Method1: " , x_round, y_round, z_round)
            #time.sleep(0.001)


            #print("Calculated cartesian coordinates Method2: " , x1_round, y1_round, z1_round, "\n")




def anchor_setup(index, serial_con):

    print("ublox Anchor " + str(index) + " configuration:====================")
    print(serial_con.port)
    #Send initial config commands to AOA board:
    serial_con.write("AT\r".encode()) #attention

    line = serial_con.readline()
    while b'AT\r\r\n' not in line:
        line = serial_con.readline()
        print(line)
        
    while b'OK\r\n' not in line:
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
    for i in range(3):
        line = serial_con.readline()
        print(line)

        if not b'ERROR' in line:
            print("Done Configuration===============================\n")
    
    return


def anchor_setup_ver2(serial_con):

    #Send initial config commands to AOA board:
    serial_con.write("AT\r".encode()) #attention
    line = serial_con.read(50)
    print(line)

    serial_con.write("AT+GMM\r".encode()) #model ID
    line = serial_con.read(50)
    print(line)

    serial_con.write("AT+UMLA=1\r".encode()) #local address
    line = serial_con.read(50)
    print(line)

    serial_con.write("ATI9\r".encode()) #software ID
    line = serial_con.read(50)
    print(line)

    serial_con.write("AT+UDFENABLE=1\r".encode()) #Direction finding enable (track / angle calc)
    line = serial_con.read(50)
    print(line)

    serial_con.write("+UDFFILT\r".encode()) #read response from Direction finding filter
    line = serial_con.read(50)
    print(line)
    
    return



def anchor_setup_ver3(index, serial_con):

    print("ublox Anchor " + str(index) + " configuration:====================")
    print(serial_con.port)
    #Send initial config commands to AOA board:
    serial_con.write("AT\r".encode()) #attention

    line = serial_con.readline()
    while b'AT\r\r\n' not in line:
        line = serial_con.readline()
    print(line)
        
    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)
    



    serial_con.write("AT+GMM\r".encode())
    while b'AT+GMM\r' not in line:
        line = serial_con.readline()
    print(line)

    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)




    serial_con.write("AT+UMLA=1\r".encode())
    while b'AT+UMLA=1\r' not in line:
        line = serial_con.readline()
    print(line)
    
    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)



    serial_con.write("ATI9\r".encode())
    while b'ATI9\r' not in line:
        line = serial_con.readline()
    print(line)

    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)




    serial_con.write("AT+UDFENABLE=1\r".encode())
    while b'AT+UDFENABLE=1\r' not in line:
        line = serial_con.readline()
    print(line)

    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)



    serial_con.write("+UDFFILT\r".encode())
    while b'+UDFFILT\r' not in line:
        line = serial_con.readline()
    print(line)
    while b'ERROR' not in line:
        line = serial_con.readline()
    print("Done Configuration===============================\n")


    return





def AOA_get_location_2(init):
    
    if(init):
        serial_con = com_conneciton
        print(serial_con);

        #Send initial config commands to AOA board:
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



    line = com_conneciton.readline()
    #print(line)
    time.sleep(0)

    #split_data format:
    split_data = line.decode('utf8').strip('\r\n').split(',')
    #[0] = <ed_instance_id>           ---> 6-byte Eddystone instance id
    #[1] = <rssi>                     ---> RSSI = Received Signal Strength Indicator,
    #[2] = <angle_azimuth>            ---> Azimuth angle in range -90 to 90 °C
    #[3] = <angle_elevation>          ---> Elevation angle in range -90 to 90 °C
    #[4] = <notused>
    #[5] = <channel>                  ---> Channel from which the packet angle was calculated
    #[6] = <anchor_id>                ---> The value set by +UDFCFG param_tag 4
    #[7] = <user_defined_str>         ---> The value set by +UDFCFG param_tag 2
    #[8] = <timestamp_ms>             ---> Time since boot in milliseconds
    #[9] = <periodic_event_counter>   ---> Periodic event counter value of the periodic advertising packet sent from a tag
    

    if( len(split_data) == 1 ): #check for empty lines of \n\r
        #read another line!!!
        line = com_conneciton.readline()
        split_data = line.decode('utf8').strip('\r\n').split(',')
    

    #2 calculated angles, between -90 and 90 degrees
    azimuth = split_data[2]
    elevation = split_data[3]
    #print("Azimuth:   " + split_data[3] + "  Elevation:  " + split_data[4])

    return azimuth, elevation
    


#https://github.com/WaveShapePlay/ArduinoPyserialComConnect/blob/master/findArduino.py 
def get_ports():

    ports = serial.tools.list_ports.comports()
    
    return ports

def findArduino(portsFound):
    
    commPort = []
    numConnection = len(portsFound)
    
    for i in range(0,numConnection):
        port = portsFound[i]
        strPort = str(port)
        
        #print(strPort)

        if 'USB Serial Port' in strPort: 
            splitPort = strPort.split(' ')
            commPort.append( (splitPort[0]) )

    return commPort







#-------------------------------TESTING---------------------------
AOA_get_location()
#example = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
#azimuth, elevation, com_con = AOA_get_location_2(True, example)
