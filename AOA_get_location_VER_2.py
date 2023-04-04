import serial
import time
import serial.tools.list_ports
import triangulation
from statistics import mean





def AOA_get_location():



    #==========POSITION OF ANCHORS============
    #[X, Y] in METERS
    anchor_1_pos = [1.29, 0.0]
    anchor_2_pos = [0.0, 0.0]
    anchor_3_pos = [0.0, 1.5]








    print("Reading anchor data")

    foundPorts = get_ports()        
    connectPort = findArduino(foundPorts)
    serial_connections = []

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


    AVERAGE_COUNTER = 5
    average_pos_counter = 0
    drone_x_positions = []
    drone_y_positions = []
    drone_z_positions = []
    a = 1
    while a == 1:

        anchor_read_lines = []
        for serial_con_read in serial_connections:
            anchor_read_lines.append( serial_con_read.readline() )
        

        anchor_azimuths = []
        anchor_elevations = []
        for line in anchor_read_lines:
            split_data = line.decode('utf8').strip('\r\n').split(',')

            
            if( len(split_data) == 9 ): #check for empty lines of \n\r

                azimuth = float(split_data[2])
                elevation = float(split_data[3])
                
                anchor_azimuths.append(azimuth)
                anchor_elevations.append(elevation)
                time.sleep(0.01)



        if len(anchor_azimuths) != 0:
            
            #Counter for caluclating the average
            average_pos_counter += 1

            #TESTING SUN OUTPUT
            #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Elevation: " + str(anchor_elevations[0]));



            #TODO:
            #Double check if Elevation angles are the same/similar between all anchors
            #def triangulation(num_of_anchors, pos_anchor_1, pos_anchor_2, azimuth1, azimuth2, elevation):

            #TODO:
            #Re-write triangulate function for 2+ anchors (3!)
            anchor_1_pos = [129, 0.0]
            anchor_2_pos = [0.0, 0.0]
            anchor_3_pos = [0.0, 150]
            #x_3_anchors, y_3_anchors, z_3_anchors = triangulation.triangulation_3_anchors(anchor_1_pos, anchor_2_pos, anchor_3_pos, anchor_azimuths[0], anchor_azimuths[1], anchor_azimuths[2], anchor_elevations[0])



            #COM12 - black cable (0, 0)
            #COM3  - white cable (x, 0)
            

            #REGULAR 2 ANCHOR CALCULATION=======================================================================
            x, y, z, x1, y2, z3 = triangulation.triangulation(len(serial_connections), anchor_1_pos, anchor_2_pos, anchor_azimuths[0], anchor_azimuths[1], anchor_elevations[0])
            #x_fuck, y_fuck, z_fuck, x1_fuck, y2_fuck, z3_fuck = triangulation.triangulation(len(serial_connections), [0.0, 0.0], [1.0, 0.0], anchor_azimuths[1], anchor_azimuths[0], anchor_elevations[0])

            #Rounded location values
            number_of_decimals = 4
            x_round, y_round, z_round = round(x, number_of_decimals), round(y, number_of_decimals), round(z, number_of_decimals)
            x1_round, y1_round, z1_round = round(x1, number_of_decimals), round(y2, number_of_decimals), round(z3, number_of_decimals)
            #y_fucking_rounded = round(y_fuck, number_of_decimals)

            #print("Calculated cartesian coordinates Method1: " , x1_round, y1_round, z1_round, "\n")

            #PRINT THE METHOD 2 VALS: MORE ACCURATE
            #print("Calculated cartesian coordinates Method 2: " , x_round, y_fucking_rounded, z_round)

            #FINAL
            #print("Calculated cartesian coordinates Method 2: " , x_round, y_round, z_round)
            #time.sleep(0.001)


            #TAKING AVERAGE OF CALC POSITIONS=======================================================================================
            #Poll 5 times, take average of 5 x, y, z positions
            #Print this out:
            drone_x_positions.append(x_round)
            drone_y_positions.append(y_round)
            drone_z_positions.append(z_round)

            #drone_x_positions.append(x1_round)
            #drone_y_positions.append(y1_round)
            #drone_z_positions.append(z1_round)

            if(average_pos_counter == AVERAGE_COUNTER):
                average_x_pos = round(mean(drone_x_positions), number_of_decimals)
                average_y_pos = round(mean(drone_y_positions), number_of_decimals)
                average_z_pos = round(mean(drone_z_positions), number_of_decimals)

                drone_x_positions.clear()
                drone_y_positions.clear()
                drone_z_positions.clear()

                average_pos_counter = 0

                print("Rolling average coordinates: (" , average_x_pos, average_y_pos, average_z_pos, " )")


            





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
