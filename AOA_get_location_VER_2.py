import serial
import time
import serial.tools.list_ports
import triangulation
from statistics import mean
import threading as t





def AOA_get_location():
    global drone_coord
    drone_coord = [0.0, 0.0, 0.0]

    global lock
    lock = t.Lock()

    print("Reading anchor data")

    foundPorts = get_ports()        
    connectPort = findArduino(foundPorts)
    serial_connections = []

    print(connectPort)

    for COM_port in connectPort:
        serial_con = serial.Serial(COM_port, 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1)
        serial_connections.append(serial_con)

    #Connect to correct COM port for uBlox AOA
    #Adam's COM port :^)
    #serial_con = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);

    #Mohammad's COM port :-)
    #serial_con = serial.Serial('COM3', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
    #print(serial_con);

    for index, con in enumerate(serial_connections):
        anchor_setup_ver3(index, con)


    AVERAGE_COUNTER = 10
    average_pos_counter = 0
    drone_x_positions = []
    drone_y_positions = []
    drone_z_positions = []

    anchor_read_lines = []

    #For 1st averaging method (average of cartesian coordinates)
    anchor_azimuths = []
    anchor_elevations = []

    #For 2nd averaging method (average of anchor angles)
    anchor_counter = 0
    anchor_1_azimuths = []
    anchor_2_azimuths = []
    anchor_1_elevations = []
    anchor_2_elevations = []

    average_cartesian_coordinates_method = True
    #average_anchor_angles_method = True

    #==========POSITION OF ANCHORS============
    #[X, Y] in CENTIMETERS
    anchor_1_pos = [0.0, 0.0]
    anchor_2_pos = [129.0, 0.0]
    anchor_3_pos = [0.0, 150.0]

    #For rounding
    number_of_decimals = 4

    a = 1
    while a == 1:

        #STEP 1: Poll through all anchors & Get list of azimuths and elevations

        for serial_con_read in serial_connections:
            
            line = serial_con_read.readline()
            anchor_read_lines.append( line )

            split_data = line.decode('utf8').strip('\r\n').split(',')
            if( len(split_data) == 9 ): #check for empty lines of \n\r

                azimuth = float(split_data[2])
                elevation = float(split_data[3])
                
                anchor_azimuths.append(azimuth)
                anchor_elevations.append(elevation)
                #time.sleep(0.01)

                #Method 2: anchor angle average
                if(anchor_counter == 0):
                    anchor_1_azimuths.append(azimuth)
                    anchor_1_elevations.append(elevation)
                    anchor_counter += 1
                elif(anchor_counter == 1):
                    anchor_2_azimuths.append(azimuth)
                    anchor_2_elevations.append(elevation)
                    anchor_counter = 0





        if len(anchor_azimuths) != 0:
            
            
            #Triangulating all angles recieved, THEN averaging cartesian coordinates
            if(average_cartesian_coordinates_method):

                #TESTING SUN OUTPUT
                #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Elevation: " + str(anchor_elevations[0]));

                #TODO:
                #Double check if Elevation angles are the same/similar between all anchors
                #Taking average of all elevations:
                average_elevation = round(mean(anchor_elevations), 4)

                #TODO:
                #Re-write triangulate function for 2+ anchors (3!)
                #3 ANCHOR CODE======================================================================================
                anchor_1_pos = [0.0, 0.0]
                anchor_2_pos = [129, 0.0]
                anchor_3_pos = [0.0, 150]
                #x_3_anchors, y_3_anchors, z_3_anchors = triangulation.triangulation_3_anchors(anchor_1_pos, anchor_2_pos, anchor_3_pos, anchor_azimuths[0], anchor_azimuths[1], anchor_azimuths[2], anchor_elevations[0])



                #COM12 - black cable (0, 0)
                #COM3  - white cable (x, 0)
                

                #REGULAR 2 ANCHOR CALCULATION=======================================================================
                x_m2, y_m2, z_m2, x1, y2, z3 = triangulation.triangulation(len(serial_connections), anchor_1_pos, anchor_2_pos, anchor_azimuths[1], anchor_azimuths[0], average_elevation)
                #x_fuck, y_fuck, z_fuck, x1_fuck, y2_fuck, z3_fuck = triangulation.triangulation(len(serial_connections), [0.0, 0.0], [1.0, 0.0], anchor_azimuths[1], anchor_azimuths[0], anchor_elevations[0])

                




                #y_fucking_rounded = round(y_fuck, number_of_decimals)

                #print("Calculated cartesian coordinates Method1: " , x1_round, y1_round, z1_round, "\n")

                #PRINT THE METHOD 2 VALS: MORE ACCURATE
                #print("Calculated cartesian coordinates Method 2: " , x_round, y_fucking_rounded, z_round)

                #FINAL
                #print("Calculated cartesian coordinates Method 2: " , x_m2, y_m2, z_m2)
                #time.sleep(0.001)


                #TAKING AVERAGE OF CALC POSITIONS=======================================================================================
                #Poll 5 times, take average of 5 x, y, z positions
                #Print this out:
                drone_x_positions.append(x_m2)
                drone_y_positions.append(y_m2)
                drone_z_positions.append(z_m2)

                anchor_azimuths.clear()
                anchor_elevations.clear()

                #drone_x_positions.append(x1_round)
                #drone_y_positions.append(y1_round)
                #drone_z_positions.append(z1_round)

                #Counter for caluclating the average
                average_pos_counter += 1

                if(average_pos_counter == AVERAGE_COUNTER):
                    
                    average_x_pos = round(mean(drone_x_positions), number_of_decimals)
                    average_y_pos = round(mean(drone_y_positions), number_of_decimals)
                    average_z_pos = round(mean(drone_z_positions), number_of_decimals)

                    drone_x_positions.clear()
                    drone_y_positions.clear()
                    drone_z_positions.clear()

                    average_pos_counter = 0

                    #print("Rolling average coordinates: (" , average_x_pos, average_y_pos, average_z_pos, " )")

                    #lock.acquire
                    lock.acquire()
                    drone_coord = [average_x_pos, average_y_pos, average_z_pos]
                    # print(drone_coord)
                    #lock.release
                    lock.release()
                    # print("killing thread")
                    # return
                    #return average_x_pos, average_y_pos, average_z_pos





            #NEW: averaging the anchor angles, THEN calculating triangulation
            else:

                #Counter for caluclating the average
                average_pos_counter += 1

                if(average_pos_counter == AVERAGE_COUNTER):
                    
                    #Average the azimuth and elevation angle arrays
                    average_anchor1_azimuths = round(mean(anchor_1_azimuths), number_of_decimals)
                    average_anchor1_elevations = round(mean(anchor_1_elevations), number_of_decimals)
                    anchor_1_azimuths.clear()
                    anchor_1_elevations.clear()

                    average_anchor2_azimuths = round(mean(anchor_2_azimuths), number_of_decimals)
                    average_anchor2_elevations = round(mean(anchor_2_elevations), number_of_decimals)
                    anchor_2_azimuths.clear()
                    anchor_2_elevations.clear()


                    #Average out the elevations between anchors:
                    average_elevation = (average_anchor1_elevations + average_anchor2_elevations) / 2.0

                    #REGULAR 2 ANCHOR CALCULATION=======================================================================
                    x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles, x_m1_from_ave_angles, y_m1_from_ave_angles, z_m1_from_ave_angles = \
                        triangulation.triangulation(len(serial_connections), anchor_1_pos, anchor_2_pos, average_anchor1_azimuths, average_anchor2_azimuths, average_elevation)

                    average_pos_counter = 0
                    #print("Rolling average coordinates: (" , x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles, " )")

                    #return x_m2_from_ave_angles, x_m2_from_ave_angles, x_m2_from_ave_angles

                    #lock.acquire
                    lock.acquire()
                    drone_coord = [x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles]
                    print(drone_coord)

                    #lock.release
                    lock.release()





                    




            





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
#AOA_get_location()
#example = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
#azimuth, elevation, com_con = AOA_get_location_2(True, example)
