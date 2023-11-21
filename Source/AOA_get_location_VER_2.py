import serial
import time
import serial.tools.list_ports
import triangulation
from statistics import mean
import threading as t
import random
import sys





def AOA_get_location():

    df = open('Drone Testing Coords x.csv','w')
    df.writelines(f"x, y, z\n")

    #Drone Coordinate array for GUI
    global drone_coord
    drone_coord = [0.0, 0.0, 0.0]

    global lock
    lock = t.Lock()

    #User variable for averaging method
    global average_method
    average_method = 2 #Default
    #average_method = 0 -> Average of x, y, z data
    #average_method = 1 -> Average of angle data
    #average_method = 2 -> Rolling Average of x, y, z data
    #average_method = 3 -> Rolling Average of angle data

    # while(1):
    #     time.sleep(1)
    #     rand_x = random.randrange(0,200,1)
    #     rand_y = random.randrange(0,200,1)
    #     rand_z = random.randrange(0,200,1)
    #     lock.acquire()
    #     drone_coord =[rand_x,rand_y,rand_z]
    #     lock.release()
        
    #====================ANCHORS INIT CONNECTION===========================================
    #======================================================================================
    print("Reading Anchor Init Data")
    foundPorts = get_ports() 
    # print("ports:", foundPorts) 
    # 
    # 
    # 
    print(foundPorts)
    connectPort = findArduino(foundPorts)     #['COM12', 'COM3', 'COM8'])#foundPorts)
    #connectPort = ["COM12", "COM3", "COM8"]
    print(connectPort)

    #print(connectPort)

    serial_connections = []
    for COM_port in connectPort:
        serial_con = serial.Serial(COM_port, 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1)
        serial_connections.append(serial_con)

    for index, con in enumerate(serial_connections):
        anchor_setup_ver3(index, con)
    #======================================================================================
    #======================================================================================

    

    #============SETUP VARIABLES===========================================================
    #======================================================================================
    AVERAGE_COUNTER = 5
    average_pos_counter = 0

    #==========POSITION OF ANCHORS============
    #[X, Y] in CENTIMETERS
    anchor_1_pos = [0.0, 0.0]
    anchor_2_pos = [62.0, 0.0]
    anchor_3_pos = [0.0, 48.0]
    #=========================================

    #METHOD OF CALCULATION
    average_cartesian_coordinates_method = True
    #average_anchor_angles_method = True

    #Arrays for triangulated x, y, z vals: used for averages of x, y, z
    drone_x_positions = []
    drone_y_positions = []
    drone_z_positions = []

    #For 1st averaging method (average of cartesian coordinates)
    anchor_azimuths = []
    anchor_elevations = []

    #Fill with first average, then update every other N times
    garbage_detection_method_0_average = []

    #For 2nd averaging method (average of anchor angles)
    anchor_counter = 0
    num_of_anchors = len(serial_connections)

    anchor_1_azimuths = []
    anchor_1_elevations = []
    anchor_2_azimuths = []
    anchor_2_elevations = []
    anchor_3_azimuths = []
    anchor_3_elevations = []

    #For rounding
    number_of_decimals = 4
    #For debugging: full messages polled from anchors
    anchor_read_lines = []

    printing_every_n = 0
    #======================================================================================
    #======================================================================================
    
    a = 1
    while a:

        anchor_azimuths = ['n','n','n']
        anchor_elevations = ['n','n','n']
        samples = 0
        #STEP 1: Poll through all anchors & Get list of azimuths and elevations
        for serial_con_read in serial_connections:
            # print(serial_con_read.port)
            line = serial_con_read.readline()
            anchor_read_lines.append( line )

            split_data = line.decode('utf8').strip('\r\n').split(',')
            #split_data format:
            #[0] = <ed_instance_id>           ---> 6-byte Eddystone instance id
            #[1] = <rssi>                     ---> RSSI = Received Signal Strength Indicator,
            #[2] = <angle_azimuth>            ---> A    zimuth angle in range -90 to 90°
            #[3] = <angle_elevation>          ---> Elevation angle in range -90 to 90°
            #[4] = <notused>
            #[5] = <channel>                  ---> Channel from which the packet angle was calculated
            #[6] = <anchor_id>                ---> The value set by +UDFCFG param_tag 4
            #[7] = <user_defined_str>         ---> The value set by +UDFCFG param_tag 2
            #[8] = <timestamp_ms>             ---> Time since boot in milliseconds
            #[9] = <periodic_event_counter>   ---> Periodic event counter value of the periodic advertising packet sent from a tag

            if( len(split_data) == 9 ): #check for empty lines of \n\r, disregard them
                
                azimuth = float(split_data[2]) #Extract azimuth
                elevation = float(split_data[3]) #Extract elevation

                #For Method 1: average of triangulated data
                
                anchor_num = 0
                if (serial_con_read.port == "COM12"): # Anchor 1
                    # print("port: ",   serial_con_read.port)
                    anchor_num = 0    

                elif (serial_con_read.port == "COM3"): # Anchor 2
                    anchor_num = 1
                    # print("port: ",   serial_con_read.port)

                elif (serial_con_read.port == "COM8"):
                    anchor_num = 2
                    # print("port: ",   serial_con_read.port)

                anchor_azimuths[anchor_num] = azimuth
                anchor_elevations[anchor_num] = elevation
                samples += 1


                #For Method 2: anchor angle average
                #Store anchor angles in their seperate lists
                if(anchor_counter == 0):
                    anchor_1_azimuths.append(azimuth)
                    anchor_1_elevations.append(elevation)
                    anchor_counter += 1

                elif(anchor_counter == 2):
                    anchor_3_azimuths.append(azimuth)
                    anchor_3_elevations.append(elevation)
                    anchor_counter = 0

                elif(anchor_counter == 1):
                    anchor_2_azimuths.append(azimuth)
                    anchor_2_elevations.append(elevation)

                    if(num_of_anchors == 3):
                        anchor_counter += 1
                    else:
                        anchor_counter = 0
                        
                
            

        if samples == 3:
            samples = 0
            #TRIANGULATION TEST
            if(average_method == 4):

                #TESTING SUN OUTPUT
                #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Elevation: " + str(anchor_elevations[0]));
                #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Azimuth 2: " + str(anchor_azimuths[1]));

                #Double check if Elevation angles are the same/similar between all anchors
                #Taking average of all elevations:
                average_elevation = round(mean(anchor_elevations), 4)
                #print("Elevations: ", anchor_elevations)
                #print("Average elvation:" , average_elevation)
                #anchor_elevations.clear()

                #COM12 - black cable (0, 0)
                #COM3  - white cable (x, 0)
                #REGULAR 2 ANCHOR CALCULATION=======================================================================
                if(num_of_anchors == 2):
                    x_m2, y_m2, z_m2= \
                        triangulation.triangulation(anchor_1_pos, anchor_2_pos, anchor_azimuths[0], anchor_azimuths[1], average_elevation)
                    # x_m2, y_m2, z_m2= \
                    #     triangulation.triangulation_hogulation(anchor_1_pos, anchor_2_pos, anchor_azimuths[0], anchor_azimuths[1], average_elevation)
                
                #3 ANCHOR CALCULATION===============================================================================
                #TODO:
                    #Double check the order of anchor azimuths in anchor_azimuths list!!!!
                    #Need to pass correctly to function
                elif(num_of_anchors == 3):
                    x_m2, y_m2, z_m2 = \
                        triangulation.triangulation_3_anchors(anchor_1_pos, anchor_2_pos, anchor_3_pos, anchor_azimuths[0], anchor_azimuths[1], anchor_azimuths[2], average_elevation)
                
                # print("Calculated cartesian coordinates Method 2: " , x_m2, y_m2, z_m2)

                #print("Calculated cartesian coordinates Method 2: " , x_m2+17.0, y_m2+17.0, z_m2)

                anchor_elevations.clear()
                anchor_azimuths.clear()

                #MAKE SURE:
                anchor_1_azimuths.clear()
                anchor_1_elevations.clear()
                anchor_2_azimuths.clear()
                anchor_2_elevations.clear()
                anchor_3_azimuths.clear()
                anchor_3_elevations.clear()
                

            #Averaging Method 1:
            #Average of every N (x, y, z) data
            #Triangulating all angles recieved, THEN averaging cartesian coordinates
            if(average_method == 0):

                #TESTING SUN OUTPUT
                #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Elevation: " + str(anchor_elevations[0]));
                #print("1: Azimuth: " + str(anchor_azimuths[0]) + " Azimuth 2: " + str(anchor_azimuths[1]));

                #Double check if Elevation angles are the same/similar between all anchors
                #Taking average of all elevations:
                average_elevation = round(mean(anchor_elevations), 4)
                #print("Elevations: ", anchor_elevations)
                #print("Average elvation:" , average_elevation)

                #COM12 - black cable (0, 0)
                #COM3  - white cable (x, 0)
                #REGULAR 2 ANCHOR CALCULATION=======================================================================
                if(num_of_anchors == 2):
                    x_m2, y_m2, z_m2= \
                        triangulation.triangulation(anchor_1_pos, anchor_2_pos, anchor_azimuths[0], anchor_azimuths[1], average_elevation)
                
                #3 ANCHOR CALCULATION===============================================================================
                #TODO:
                    #Double check the order of anchor azimuths in anchor_azimuths list!!!!
                    #Need to pass correctly to function
                elif(num_of_anchors == 3):
                    x_m2, y_m2, z_m2 = \
                        triangulation.triangulation_3_anchors(anchor_1_pos, anchor_2_pos, anchor_3_pos, anchor_azimuths[0], anchor_azimuths[1], anchor_azimuths[2], average_elevation)
                
                #print("Calculated cartesian coordinates Method 2: " , x_m2, y_m2, z_m2)

                #TAKING AVERAGE OF CALC POSITIONS=======================================================================================
                #Poll N times, take average of N x, y, z positions
                #Print this out:
                drone_x_positions.append(x_m2)
                drone_y_positions.append(y_m2)
                drone_z_positions.append(z_m2)

                # #GARBAGE DETECTION:
                # if(len(garbage_detection_method_0_average) == 0):
                #     drone_x_positions.append(x_m2)
                #     drone_y_positions.append(y_m2)
                #     drone_z_positions.append(z_m2)
                # else:
                #     x_off = abs( (garbage_detection_method_0_average[0] - x_m2)/garbage_detection_method_0_average[0] ) * 100
                #     y_off = abs( (garbage_detection_method_0_average[1] - y_m2)/garbage_detection_method_0_average[1] ) * 100
                #     z_off = abs( (garbage_detection_method_0_average[2] - z_m2)/garbage_detection_method_0_average[2] ) * 100

                #     if( (x_off > 90) or (y_off > 90) or (z_off > 90) ):
                #         adam = 0
                #     else:
                #         drone_x_positions.append(x_m2)
                #         drone_y_positions.append(y_m2)
                #         drone_z_positions.append(z_m2)

                #Clear these lists, will collect new pair of angles from 2 Anchors next loop
                anchor_azimuths.clear()
                anchor_elevations.clear()

                #Used to use counter average_pos_counter
                if(len(drone_x_positions) >= AVERAGE_COUNTER):
                    
                    average_x_pos = round(mean(drone_x_positions), number_of_decimals)
                    average_y_pos = round(mean(drone_y_positions), number_of_decimals)
                    average_z_pos = round(mean(drone_z_positions), number_of_decimals)

                    drone_x_positions.clear()
                    drone_y_positions.clear()
                    drone_z_positions.clear()


                    # garbage_detection_method_0_average.clear()

                    # garbage_detection_method_0_average.append(average_x_pos)
                    # garbage_detection_method_0_average.append(average_y_pos)
                    # garbage_detection_method_0_average.append(average_z_pos)

                    #print("Rolling average coordinates: (" , average_x_pos, average_y_pos, average_z_pos, " )")

                    lock.acquire()
                    drone_coord = [average_x_pos, average_y_pos, average_z_pos]
                    # print(drone_coord)
                    lock.release()
                    # print("killing thread")

                #MAKE SURE:
                anchor_1_azimuths.clear()
                anchor_1_elevations.clear()
                anchor_2_azimuths.clear()
                anchor_2_elevations.clear()
                anchor_3_azimuths.clear()
                anchor_3_elevations.clear()


            #Averaging Method 3:
            #ROLLING AVERAGE of x, y, z data
            elif(average_method == 2):

                #Fill up array of x, y, z data up to average_pos_counter == AVERAGE_COUNTER--------------------------------------------------------------------

                #Taking average of all elevations:
                average_elevation = round(mean(anchor_elevations), 4)
                #REGULAR 2 ANCHOR CALCULATION=======================================================================
                if(num_of_anchors == 2):
                    x_m2, y_m2, z_m2 = \
                        triangulation.triangulation(anchor_1_pos, anchor_2_pos, anchor_azimuths[0], anchor_azimuths[1], average_elevation)
                
                #3 ANCHOR CALCULATION===============================================================================
                #TODO:
                    #Double check the order of anchor azimuths in anchor_azimuths list!!!!
                    #Need to pass correctly to function
                elif(num_of_anchors == 3):
                    x_m2, y_m2, z_m2 = \
                        triangulation.triangulation_3_anchors(anchor_1_pos, anchor_2_pos, anchor_3_pos, anchor_azimuths[0], anchor_azimuths[1], anchor_azimuths[2], average_elevation)


                #Add current x, y, z into arrays
                drone_x_positions.append(x_m2)
                drone_y_positions.append(y_m2)
                drone_z_positions.append(z_m2)

                #Clear these lists, will collect new pair of angles from 2 Anchors next loop
                anchor_azimuths.clear()
                anchor_elevations.clear()
                #----------------------------------------------------------------------------------------------------------------------------------------------

                #Take average of x, y, z array (with rolling average window length)
                #Same as average_pos_counter == AVERAGE_COUNTER
                if(len(drone_x_positions) == AVERAGE_COUNTER):

                    average_x_pos = round(mean(drone_x_positions), number_of_decimals)
                    average_y_pos = round(mean(drone_y_positions), number_of_decimals)
                    average_z_pos = round(mean(drone_z_positions), number_of_decimals)
                    #print("Rolling average coordinates: (" , average_x_pos, average_y_pos, average_z_pos, " )")

                    lock.acquire()
                    drone_coord = [average_x_pos, average_y_pos, average_z_pos]
                    # print(drone_coord)

                    
                    # with open('Drone Test.txt', 'w') as f:
                    #     f.write('(')
                    #     for coord in drone_coord:
                    #         f.write(str(coord))
                    #         f.write(', ')
                    #     f.write(')')
                    df.writelines(f"{drone_coord[0]},{drone_coord[1]},{drone_coord[2]}\n")
                    # for coord in drone_coord:
                    #         df.write(str(coord))
                    #         df.write(', ')
                    # df.write(')')

                    if(a == 400):
                        # df.close()
                        pass


                    lock.release()

                    # print("killing thread")

                    #Remove First (oldest) data point from lists!!!
                    drone_x_positions.pop(0)
                    drone_y_positions.pop(0)
                    drone_z_positions.pop(0)

                if(len(drone_x_positions) > AVERAGE_COUNTER):
                    drone_x_positions.clear()
                    drone_y_positions.clear()
                    drone_z_positions.clear()
                
                #MAKE SURE:
                anchor_1_azimuths.clear()
                anchor_1_elevations.clear()
                anchor_2_azimuths.clear()
                anchor_2_elevations.clear()
                anchor_3_azimuths.clear()
                anchor_3_elevations.clear()




            
            #Averaging Method #2
            #Averaging the anchor angles, THEN calculating triangulation
            elif(average_method == 1):
                
                #Used to use average_pos_counter
                if(len(anchor_1_azimuths) >= AVERAGE_COUNTER):
                    
                    #Average the azimuth and elevation angle arrays
                    average_anchor1_azimuths = round(mean(anchor_1_azimuths), number_of_decimals)
                    average_anchor1_elevations = round(mean(anchor_1_elevations), number_of_decimals)
                    anchor_1_azimuths.clear()
                    anchor_1_elevations.clear()

                    average_anchor2_azimuths = round(mean(anchor_2_azimuths), number_of_decimals)
                    average_anchor2_elevations = round(mean(anchor_2_elevations), number_of_decimals)
                    anchor_2_azimuths.clear()
                    anchor_2_elevations.clear()

                    if(num_of_anchors == 3):
                        average_anchor3_azimuths = round(mean(anchor_3_azimuths), number_of_decimals)
                        average_anchor3_elevations = round(mean(anchor_3_elevations), number_of_decimals)
                        anchor_3_azimuths.clear()
                        anchor_3_elevations.clear()

                    #Average out the elevations between anchors:
                    if(num_of_anchors == 2):
                        average_elevation = (average_anchor1_elevations + average_anchor2_elevations) / 2.0
                    else:
                        average_elevation = (average_anchor1_elevations + average_anchor2_elevations + average_anchor3_elevations) / 3.0      

                    #REGULAR 2 ANCHOR CALCULATION=======================================================================
                    if(num_of_anchors == 2):
                        x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles= \
                            triangulation.triangulation(anchor_1_pos, anchor_2_pos, average_anchor1_azimuths, average_anchor2_azimuths, average_elevation)

                    #3 ANCHOR CALCULATION===============================================================================
                    elif(num_of_anchors == 3):
                        x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles = \
                            triangulation.triangulation_3_anchors(anchor_1_pos, anchor_2_pos, anchor_3_pos, average_anchor1_azimuths, average_anchor2_azimuths, average_anchor3_azimuths, average_elevation)
                    
                    #print("Rolling average coordinates: (" , x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles, " )")

                    lock.acquire()
                    drone_coord = [x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles]
                    # print(drone_coord)
                    lock.release()

                anchor_azimuths.clear()
                anchor_elevations.clear()
                



            
            #Averaging Method #4
            #ROLLING AVERAGE of angle data THEN calculating triangulation
            elif(average_method == 3):
                
                #Collect AVERAGE_COUNTER number of angles per anchor (the averaging window)
                #Angle arrays filled above
                if(len(anchor_1_azimuths) == AVERAGE_COUNTER):
                    
                    #Average the azimuth and elevation angle arrays
                    average_anchor1_azimuths = round(mean(anchor_1_azimuths), number_of_decimals)
                    average_anchor1_elevations = round(mean(anchor_1_elevations), number_of_decimals)
                    

                    average_anchor2_azimuths = round(mean(anchor_2_azimuths), number_of_decimals)
                    average_anchor2_elevations = round(mean(anchor_2_elevations), number_of_decimals)
                    

                    if(num_of_anchors == 3):
                        average_anchor3_azimuths = round(mean(anchor_3_azimuths), number_of_decimals)
                        average_anchor3_elevations = round(mean(anchor_3_elevations), number_of_decimals)
                        

                    #Average out the elevations between anchors:
                    if(num_of_anchors == 3):
                        average_elevation = (average_anchor1_elevations + average_anchor2_elevations + average_anchor3_elevations) / 3.0      
                    else:
                        average_elevation = (average_anchor1_elevations + average_anchor2_elevations) / 2.0


                    #REGULAR 2 ANCHOR CALCULATION=======================================================================
                    if(num_of_anchors == 2):
                        x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles = \
                            triangulation.triangulation(anchor_1_pos, anchor_2_pos, average_anchor1_azimuths, average_anchor2_azimuths, average_elevation)

                    #3 ANCHOR CALCULATION===============================================================================
                    elif(num_of_anchors == 3):
                        x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles = \
                            triangulation.triangulation_3_anchors(anchor_1_pos, anchor_2_pos, anchor_3_pos, average_anchor1_azimuths, average_anchor2_azimuths, average_anchor3_azimuths, average_elevation)
                    
                    #print("Rolling average coordinates: (" , x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles, " )")

                    lock.acquire()
                    drone_coord = [x_m2_from_ave_angles, y_m2_from_ave_angles, z_m2_from_ave_angles]
                    # print(drone_coord)
                    lock.release()

                    # if(printing_every_n % AVERAGE_COUNTER):
                    #     print(drone_coord)
                    #     printing_every_n = 1


                    #REMOVE OLDEST DATA FROM LISTS!
                    anchor_1_azimuths.pop(0)
                    anchor_1_elevations.pop(0)
                    anchor_2_azimuths.pop(0)
                    anchor_2_elevations.pop(0)
                    
                    if(num_of_anchors == 3):
                        anchor_3_azimuths.pop(0)
                        anchor_3_elevations.pop(0)

                
                #JUST IN CASE
                elif(len(anchor_1_azimuths) > AVERAGE_COUNTER):
                    anchor_1_azimuths.clear()
                    anchor_1_elevations.clear()
                    anchor_2_azimuths.clear()
                    anchor_2_elevations.clear()
                    anchor_3_azimuths.clear()
                    anchor_3_elevations.clear()

                anchor_azimuths.clear()
                anchor_elevations.clear()

        
            
        a += 1


            
            









def anchor_setup_ver3(index, serial_con):

    print("ublox Anchor " + str(index) + " configuration:====================")
    print(serial_con.port)
    #Send initial config commands to AOA board:
    serial_con.write("AT\r".encode()) #attention

    #Send and confirm AT command:-----------------------------
    line = serial_con.readline()
    while b'AT\r\r\n' not in line:
        line = serial_con.readline()
    print(line)
        
    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)
    #---------------------------------------------------------


    #Send and confirm AT+GMM command:-------------------------
    serial_con.write("AT+GMM\r".encode())
    while b'AT+GMM\r' not in line:
        line = serial_con.readline()
    print(line)

    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)
    #---------------------------------------------------------


    #Send and confirm AT+UMLA=1 command:----------------------
    serial_con.write("AT+UMLA=1\r".encode())
    while b'AT+UMLA=1\r' not in line:
        line = serial_con.readline()
    print(line)
    
    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)
    #---------------------------------------------------------


    #Send and confirm ATI9 command:---------------------------
    serial_con.write("ATI9\r".encode())
    while b'ATI9\r' not in line:
        line = serial_con.readline()
    print(line)

    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)
    #---------------------------------------------------------


    #Send and confirm AT+UDFENABLE=1 command:-----------------
    serial_con.write("AT+UDFENABLE=1\r".encode())
    while b'AT+UDFENABLE=1\r' not in line:
        line = serial_con.readline()
    print(line)

    while b'OK\r\n' not in line:
        line = serial_con.readline()
    print(line)
    #---------------------------------------------------------


    #Send and confirm +UDFFILT command:-----------------------
    serial_con.write("+UDFFILT\r".encode())
    while b'+UDFFILT\r' not in line:
        line = serial_con.readline()
    print(line)
    while b'ERROR' not in line:
        line = serial_con.readline()
    #---------------------------------------------------------
    print("Done Configuration===============================\n")

    return

#https://github.com/WaveShapePlay/ArduinoPyserialComConnect/blob/master/findArduino.py 
def get_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    # if sys.platform.startswith('win'):
    #     ports = ['COM%s' % (i + 1) for i in range(256)]
    # elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
    #     # this excludes your current terminal "/dev/tty"
    #     ports = glob.glob('/dev/tty[A-Za-z]*')
    # elif sys.platform.startswith('darwin'):
    #     ports = glob.glob('/dev/tty.*')
    # else:
    #     raise EnvironmentError('Unsupported platform')

    # result = []
    # for port in ports:
    #     try:
    #         s = serial.Serial(port)
    #         s.close()
    #         result.append(port)
    #     except (OSError, serial.SerialException):
    #         pass
    # return result
    ports = serial.tools.list_ports.comports()
    return ports

def findArduino(portsFound: list):
    
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
# AOA_get_location()
# anchor_3_azimuths = []
# #print(mean(anchor_3_azimuths))

# length = 1
# print(float(length))