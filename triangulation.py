import math as m
from statistics import mean





def triangulation(pos_anchor_1, pos_anchor_2, azimuth1, azimuth2, elevation):
    #print("Triangulating the location of the drones to cartesian:\n\n")

    #[x, y] coordinate on floor (in meters)
    #pos_anchor_1 = [0.0, 0.0]
    x1 = pos_anchor_1[0]
    y1 = pos_anchor_1[1]
    
    #pos_anchor_2 = [1.5, 0.0]
    x2 = pos_anchor_2[0]
    y2 = pos_anchor_2[1]

    #print(f"Azimuth1: {azimuth1} | Azimuth2: {azimuth2}")
    #print(elevation)

    phi = 90.0 - elevation
    theta_1 = 90 - azimuth1
    theta_2 = 90 - azimuth2

    #tan1 = float(m.tan(m.radians(theta_1)))
    #tan2 = float(m.tan(m.radians(theta_2)))

    if(theta_1 == 0):
        theta_1 = 0.0001
        
    elif(theta_2 == 0):
        theta_2 = 0.0001
    
    new_tan_1 = float(m.sin(m.radians(theta_1))) / float(m.cos(m.radians(theta_1)))
    new_tan_2 = float(m.sin(m.radians(theta_2))) / float(m.cos(m.radians(theta_2)))
    #print("Tans:", new_tan_1, new_tan_1)

    # x_m2 = ( (y1 - y2) + x2*tan2 - x1*tan1 ) / ( tan2 - tan1 )      
    # y_m2 = ( y1*tan2 - y2*tan1 - (x2 - x1)*tan2*tan1 ) / ( tan2 - tan1 )
    # z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) / m.tan(m.radians(phi))

    x_m2 = ( (y1 - y2) + x2*new_tan_2 - x1*new_tan_1 ) / ( new_tan_2 - new_tan_1 )      
    y_m2 = ( y1*new_tan_2 - y2*new_tan_1 + (x2 - x1)*new_tan_2*new_tan_1 ) / ( new_tan_2 - new_tan_1 )

    denom = m.tan(m.radians(phi))
    z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) / denom
    #z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) * m.cos(m.radians(phi))

    #Round all calculated vals:
    number_of_decimals = 4
    x_m2_round, y_m2_round, z_m2_round = round(x_m2, number_of_decimals), round(y_m2, number_of_decimals), round(z_m2, number_of_decimals)
    # print(x_m2_round)
    return x_m2_round, y_m2_round, z_m2_round






def triangulation_hogulation(pos_anchor_1, pos_anchor_2, azimuth1, azimuth2, elevation):
    #print("Triangulating the location of the drones to cartesian:\n\n")

    #[x, y] coordinate on floor (in meters)
    #pos_anchor_1 = [0.0, 0.0]
    x1 = pos_anchor_1[0]
    y1 = pos_anchor_1[1]
    
    #pos_anchor_2 = [1.5, 0.0]
    x2 = pos_anchor_2[0]
    y2 = pos_anchor_2[1]

    #print(f"Azimuth1: {azimuth1} | Azimuth2: {azimuth2}")
    #print(elevation)

    phi = 90.0 - elevation
    theta_1 = 90 - azimuth1
    theta_2 = 90 + azimuth2

    #tan1 = float(m.tan(m.radians(theta_1)))
    #tan2 = float(m.tan(m.radians(theta_2)))

    if(theta_1 == 0):
        theta_1 = 0.0001
        
    elif(theta_2 == 0):
        theta_2 = 0.0001
    
    new_tan_1 = float(m.sin(m.radians(theta_1))) / float(m.cos(m.radians(theta_1)))
    new_tan_2 = float(m.sin(m.radians(theta_2))) / float(m.cos(m.radians(theta_2)))
    #print("Tans:", new_tan_1, new_tan_1)

    x_m2 = ( x2*new_tan_2 ) / ( new_tan_2 + new_tan_1 ) 

    #Ver1  
    y_m2 = x_m2 * new_tan_1
    #Ver2
    #y_m2 = (x2 - x_m2) * new_tan_2

    z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) / m.tan(m.radians(phi))
    #z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) * m.cos(m.radians(phi))

    #Round all calculated vals:
    number_of_decimals = 4
    x_m2_round, y_m2_round, z_m2_round = round(x_m2, number_of_decimals), round(y_m2, number_of_decimals), round(z_m2, number_of_decimals)
    # print(x_m2_round)
    return x_m2_round, y_m2_round, z_m2_round






def triangulation_old(num_of_anchors, pos_anchor_1, pos_anchor_2, azimuth1, azimuth2, elevation):
    #print("Triangulating the location of the drones to cartesian:\n\n")



    #[x, y] coordinate on floor (in meters)
    #pos_anchor_1 = [0.0, 0.0]
    x1 = pos_anchor_1[0]
    y1 = pos_anchor_1[1]
    
    #pos_anchor_2 = [1.5, 0.0]
    x2 = pos_anchor_2[0]
    y2 = pos_anchor_2[1]

    print(f"Azimuth1: {azimuth1} | Azimuth2: {azimuth2}")
    print(elevation)
    
    
    
    #print(y1)
    #print(m.cos(m.radians(0)))
    #print(m.tan(m.radians(90)))

    # #Used to calculate rho (Method 1)
    # x_baseline_distance = pos_anchor_2[0] - pos_anchor_1[0]
    # azimuth_2 = 180 - azimuth2
    # x_baseline_angle = azimuth2 - azimuth1


    # #Variables needed for calculus coordinate calcs
    # denominator = (m.sin(m.radians(x_baseline_angle)))
    # if(denominator != 0):
    #     rho = (m.sin(m.radians(azimuth_2))*x_baseline_distance) / denominator
    # else:
    #     rho = 0
        
    # theta = azimuth1
    phi = 90.0 - elevation

    #print("uBLox Anchor positions: (" , x1, ", " , y1 , ") (" , x2 , ", " , y2 , ")\n")

    # if(num_of_anchors == 2):

    #     method_1 = False
    #     if(True):
            
    #         #----------Law of Sines Method-------------------------------------------------
    #         #Will calculate the location of drone relative to anchor #1
    #         #Have:
    #         #   azimuth1 = theta
    #         #   90 - elevation = phi

    #         #Need to calculate rho:
    #         if( rho != 0):
    #             x_m1 = rho * m.sin( m.radians(phi) ) * m.cos( m.radians(theta) )
    #             y_m1 = rho * m.sin( m.radians(phi) ) * m.sin( m.radians(theta) )
    #             z_m1 = rho * m.cos( m.radians(phi) )
    #         else:
    #             x_m1 = m.nan
    #             y_m1 = m.nan
    #             z_m1 = m.nan 
    
    #         #return x, y, z


    theta_1 = 90 - azimuth1
    theta_2 = 90 - azimuth2

    if(True):
        #-----------Method #2----------------------------------------------------------
        #Source:
        #https://www.omnicalculator.com/m/triangulation

        #x = ( (0 - pos_anchor_2[1]) + pos_anchor_2[0]*m.tan(m.radians(azimuth2)) )



        tan1 = float(m.tan(m.radians(theta_1)))
        tan2 = float(m.tan(m.radians(theta_2)))

        if(theta_1 == 0):
            theta_1 = 0.0001
            
        elif(theta_2 == 0):
            theta_2 = 0.0001
        
        else:
            new_tan_1 = float(m.sin(m.radians(theta_1))) / float(m.cos(m.radians(theta_1)))
            new_tan_2 = float(m.sin(m.radians(theta_2))) / float(m.cos(m.radians(theta_2)))

            print("Tans:", new_tan_1, new_tan_1)

        # x_m2 = ( (y1 - y2) + x2*tan2 - x1*tan1 ) / ( tan2 - tan1 )      
        # y_m2 = ( y1*tan2 - y2*tan1 - (x2 - x1)*tan2*tan1 ) / ( tan2 - tan1 )
        # z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) / m.tan(m.radians(phi))

        x_m2 = ( (y1 - y2) + x2*new_tan_2 - x1*new_tan_1 ) / ( new_tan_2 - new_tan_1 )      
        y_m2 = ( y1*new_tan_2 - y2*new_tan_1 + (x2 - x1)*new_tan_2*new_tan_1 ) / ( new_tan_2 - new_tan_1 )
        #z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) * m.cos(m.radians(phi))
        z_m2 =  m.sqrt( m.pow(x_m2, 2) + m.pow(y_m2, 2) ) / m.tan(m.radians(phi))






    #Round all calculated vals:
    number_of_decimals = 4
    #x_m1_round, y_m1_round, z_m1_round = round(x_m1, number_of_decimals), round(y_m1, number_of_decimals), round(z_m1, number_of_decimals)
    x_m2_round, y_m2_round, z_m2_round = round(x_m2, number_of_decimals), round(y_m2, number_of_decimals), round(z_m2, number_of_decimals)
    # print(x_m2_round)
    return x_m2_round, y_m2_round, z_m2_round#, x_m1_round, y_m1_round, z_m1_round





def triangulation_3_anchors(pos_anchor_1, pos_anchor_2, pos_anchor_3, azimuth1, azimuth2, azimuth3, elevation):
    
    #[x, y] coordinate on floor (in meters)
    #pos_anchor_1 = [0.0, 0.0]
    x1 = pos_anchor_1[0]
    y1 = pos_anchor_1[1]

    #pos_anchor_2 = [1.5, 0.0]
    x2 = pos_anchor_2[0]
    y2 = pos_anchor_2[1]

    #pos_anchor_3 = [0.0, 1.5]
    x3 = pos_anchor_3[0]
    y3 = pos_anchor_3[1]

    #Define theta and phi angles using azimuths
    phi = 90 - elevation
    theta_1 = 90 - azimuth1
    theta_2 = 90 - azimuth2

    x_positions = []
    y_positions = []
    z_positions = []


    if(theta_1 == 0):
        theta_1 = 0.0001
        
    elif(theta_2 == 0):
        theta_2 = 0.0001
    
    new_tan_1 = float(m.sin(m.radians(theta_1))) / float(m.cos(m.radians(theta_1)))
    new_tan_2 = float(m.sin(m.radians(theta_2))) / float(m.cos(m.radians(theta_2)))



    #-----------Method #2----------------------------------------------------------
    #Source:
    #https://www.omnicalculator.com/m/triangulation

    #Will take the average of the triangulated position each pair of anchors calculates
    #Since we have 3 anchors, 3 positions will be calculated using:
        #A) Anchor 1 & Anchor 2 (Default)
        #B) Anchor 1 & Anchor 3
        #C) Anchor 2 & Anchor 3

    #A) Anchor 1 & 2----------------------------------------------------------------------
    x_A = ( (y1 - y2) + x2*new_tan_2 - x1*new_tan_1 ) / ( new_tan_2 - new_tan_1 )      
    y_A = ( y1*new_tan_2 - y2*new_tan_1 + (x2 - x1)*new_tan_2*new_tan_1 ) / ( new_tan_2 - new_tan_1 )

    denom = m.tan(m.radians(phi))
    z_A =  m.sqrt( m.pow(x_A, 2) + m.pow(y_A, 2) ) / denom

    number_of_decimals = 4
    x_A_round, y_A_round, z_A_round = round(x_A, number_of_decimals), round(y_A, number_of_decimals), round(z_A, number_of_decimals)

    x_positions.append(x_A_round)
    y_positions.append(y_A_round)
    z_positions.append(z_A_round)



    #B) Anchor 1 & 3----------------------------------------------------------------------
    #Setup new vars:
    x_3_temp = y3
    y_3_temp = x3

    theta_3 = 90 - azimuth3
    theta_1_prime = 180 - azimuth1

    new_tan_3 = float(m.sin(m.radians(theta_3))) / float(m.cos(m.radians(theta_3)))
    new_tan_1_prime = float(m.tan(m.radians(theta_1_prime))) #float(m.sin(m.radians(theta_1_prime))) / float(m.cos(m.radians(theta_1_prime)))

    denominator = ( new_tan_1_prime - new_tan_3 )
    if(denominator):

        x_B = ( (y1 - y_3_temp) + x_3_temp*new_tan_1_prime - x1*new_tan_3 ) / denominator
        y_B = ( y1*new_tan_1_prime - y_3_temp*new_tan_3 + (x_3_temp - x1)*new_tan_1_prime*new_tan_3 ) / ( new_tan_1_prime - new_tan_3 )

        denom = m.tan(m.radians(phi))
        z_B =  m.sqrt( m.pow(x_B, 2) + m.pow(y_B, 2) ) / denom

        #Convert calculated x, y into the room x, y coordinates (Using Anchor 1 as origin)
        x_B_final = y_B
        y_B_final = y3 - x_B

        x_B_round, y_B_round, z_B_round = round(x_B_final, number_of_decimals), round(y_B_final, number_of_decimals), round(z_B, number_of_decimals)

        x_positions.append(x_B_round)
        y_positions.append(y_B_round)
        z_positions.append(z_B_round)


    
    #C) Anchor 2 & 3----------------------------------------------------------------------
    #Setup new vars:
    tan_theta_3 = m.tan(m.radians(azimuth3))
    tan_theta_2 = m.tan(m.radians(-azimuth2))

    y_C = (y3 - x2 * tan_theta_3) / (1 - tan_theta_2 * tan_theta_3)

    x_C = x2 - y_C * tan_theta_2

    z_C =  m.sqrt( m.pow(x_C, 2) + m.pow(y_C, 2) ) / m.tan(m.radians(phi))

    x_C_round, y_C_round, z_C_round = round(x_C, number_of_decimals), round(y_C, number_of_decimals), round(z_C, number_of_decimals)

    x_positions.append(x_C_round)
    y_positions.append(y_C_round)
    z_positions.append(z_C_round)

    

    #Compute the average of anchor pair calculations:
    average_x_pos = round(mean(x_positions), number_of_decimals)
    average_y_pos = round(mean(y_positions), number_of_decimals)
    average_z_pos = round(mean(z_positions), number_of_decimals)

    return average_x_pos, average_y_pos, average_z_pos



#x, y, z, x1, y2, z3 = triangulation(2, [0, 0], [3, 0], 60, 150, 30)
#print("Calculated cartesian coordinates Method1: " , x, y, z, "\n")
#print("Calculated cartesian coordinates Method2: " , x1, y2, z3, "\n")


