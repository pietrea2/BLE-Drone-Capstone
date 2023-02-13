import serial
import time

com_conneciton = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1)

def AOA_get_location():
    print("Reading anchor data")

    #Connect to correct COM port for uBlox AOA
    serial_con = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
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



    a = 1
    while a == 1:


        line = serial_con.readline()
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
        

        if( len(split_data) != 1 ): #check for empty lines of \n\r
            #print(split_data)
            #print(split_data[0]) #6-byte Eddystone instance id of tag
            #print(split_data[2]) #RSSI of 1st polarization

            #2 calculated angles, between -90 and 90 degrees
            azimuth = split_data[3]
            elevation = split_data[4]
            print("Azimuth:   " + split_data[3] + "Elevation:  " + split_data[4])





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
    



#-------------------------------TESTING---------------------------
#AOA_get_location()
#example = serial.Serial('COM12', 115200, parity=serial.PARITY_NONE, rtscts=1, timeout=1, stopbits=1);
#azimuth, elevation, com_con = AOA_get_location_2(True, example)
