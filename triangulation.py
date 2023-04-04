import math as m
from statistics import mean


def triangulation(num_of_anchors, pos_anchor_1, pos_anchor_2, azimuth1, azimuth2, elevation):
    #print("Triangulating the location of the drones to cartesian:\n\n")



    #[x, y] coordinate on floor (in meters)
    #pos_anchor_1 = [0.0, 0.0]
    x1 = pos_anchor_1[0]
    y1 = pos_anchor_1[1]

    #pos_anchor_2 = [1.5, 0.0]
    x2 = pos_anchor_2[0]
    y2 = pos_anchor_2[1]

    #Used to calculate rho (Method 1)
    x_baseline_distance = pos_anchor_2[0] - pos_anchor_1[0]
    azimuth_2 = 180 - azimuth2
    x_baseline_angle = azimuth2 - azimuth1


    #Variables needed for calculus coordinate calcs
    denominator = (m.sin(m.radians(x_baseline_angle)))
    if(denominator != 0):
        rho = (m.sin(m.radians(azimuth_2))*x_baseline_distance) / denominator
    else:
        rho = 0
        
    theta = azimuth1
    phi = 90 - elevation

    #print("uBLox Anchor positions: (" , x1, ", " , y1 , ") (" , x2 , ", " , y2 , ")\n")

    if(num_of_anchors == 2):

        method_1 = False
        if(True):
            
            #----------Law of Sines Method-------------------------------------------------
            #Will calculate the location of drone relative to anchor #1
            #Have:
            #   azimuth1 = theta
            #   90 - elevation = phi

            #Need to calculate rho:
            if( rho != 0):
                x_m = rho * m.sin( m.radians(phi) ) * m.cos( m.radians(theta) )
                y_m = rho * m.sin( m.radians(phi) ) * m.sin( m.radians(theta) )
                z_m = rho * m.cos( m.radians(phi) )
            else:
                x_m = m.nan
                y_m = m.nan
                z_m = m.nan 
    
            #return x, y, z

        if(True):
            #-----------Method #2----------------------------------------------------------
            #Source:
            #https://www.omnicalculator.com/m/triangulation

            #x = ( (0 - pos_anchor_2[1]) + pos_anchor_2[0]*m.tan(m.radians(azimuth2)) )
            x = ( (y1 - y2) + x2*m.tan(m.radians(90 - azimuth2)) - x1*m.tan(m.radians(90 - azimuth1)) ) / \
                                ( m.tan(m.radians(90 - azimuth2)) - m.tan(m.radians(90 - azimuth1)) )
                    
            y = ( y1*m.tan(m.radians(90 - azimuth2)) - y2*m.tan(m.radians(90 - azimuth1)) - (x2 - x1)*m.tan(m.radians(90 - azimuth2))*m.tan(m.radians(90 - azimuth1)) ) / \
                                                                                                    ( m.tan(m.radians(90 - azimuth2)) - m.tan(m.radians(90 - azimuth1)) )

            z =  m.sqrt( m.pow(x, 2) + m.pow(y, 2) ) / m.tan(m.radians(phi))


        return x, y, z, x_m, y_m, z_m





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
    theta_3 = 90 - azimuth3

    x_positions = []
    y_positions = []
    z_positions = []

    #-----------Method #2----------------------------------------------------------
    #Source:
    #https://www.omnicalculator.com/m/triangulation

    #Will take the average of the triangulated position each pair of anchors calculates
    #Since we have 3 anchors, 3 positions will be calculated using:
        #A) Anchor 1 & Anchor 2 (Default)
        #B) Anchor 1 & Anchor 3
        #C) Anchor 2 & Anchor 3

    #A) Anchor 1 & 2----------------------------------------------------------------------
    x_A = ( (y1 - y2) + x2*m.tan(m.radians(theta_2)) - x1*m.tan(m.radians(theta_1)) ) / \
                        ( m.tan(m.radians(theta_2)) - m.tan(m.radians(theta_1)) )
            
    y_A = ( y1*m.tan(m.radians(theta_2)) - y2*m.tan(m.radians(theta_1)) - (x2 - x1)*m.tan(m.radians(theta_2))*m.tan(m.radians(theta_1)) ) / \
                                                                                            ( m.tan(m.radians(theta_2)) - m.tan(m.radians(theta_1)) )

    z_A =  m.sqrt( m.pow(x_A, 2) + m.pow(y_A, 2) ) / m.tan(m.radians(phi))

    number_of_decimals = 4
    x_A_round, y_A_round, z_A_round = round(x_A, number_of_decimals), round(y_A, number_of_decimals), round(z_A, number_of_decimals)

    x_positions.append(x_A_round)
    y_positions.append(y_A_round)
    z_positions.append(z_A_round)

    




    #B) Anchor 1 & 3----------------------------------------------------------------------
    x_B = ( (y3 - y1) + x1*m.tan(m.radians(theta_1)) - x3*m.tan(m.radians(theta_3)) ) / \
                        ( m.tan(m.radians(theta_1)) - m.tan(m.radians(theta_3)) )
            
    y_B = ( y3*m.tan(m.radians(theta_1)) - y1*m.tan(m.radians(theta_3)) - (x1 - x3)*m.tan(m.radians(theta_1))*m.tan(m.radians(theta_3)) ) / \
                                                                                            ( m.tan(m.radians(theta_1)) - m.tan(m.radians(theta_3)) )

    z_B =  m.sqrt( m.pow(x_B, 2) + m.pow(y_B, 2) ) / m.tan(m.radians(phi))

    #Convert calculated x, y into the room x, y coordinates (Using Anchor 1 as origin)
    x_B_final = y_B
    y_B_final = y3 - x_B

    x_B_round, y_B_round, z_B_round = round(x_B_final, number_of_decimals), round(y_B_final, number_of_decimals), round(z_B, number_of_decimals)

    x_positions.append(x_B_round)
    y_positions.append(y_B_round)
    z_positions.append(z_B_round)




    '''
    #C) Anchor 2 & 3----------------------------------------------------------------------
    x_A = ( (y1 - y2) + x2*m.tan(m.radians(theta_2)) - x1*m.tan(m.radians(theta_1)) ) / \
                        ( m.tan(m.radians(theta_2)) - m.tan(m.radians(theta_1)) )
            
    y_A = ( y1*m.tan(m.radians(theta_2)) - y2*m.tan(m.radians(theta_1)) - (x2 - x1)*m.tan(m.radians(theta_2))*m.tan(m.radians(theta_1)) ) / \
                                                                                            ( m.tan(m.radians(theta_2)) - m.tan(m.radians(theta_1)) )

    z_A =  m.sqrt( m.pow(x_A, 2) + m.pow(y_A, 2) ) / m.tan(m.radians(phi))
    '''





    #Compute the average of anchor pair calculations:
    average_x_pos = round(mean(x_positions), number_of_decimals)
    average_y_pos = round(mean(y_positions), number_of_decimals)
    average_z_pos = round(mean(z_positions), number_of_decimals)


    return average_x_pos, average_y_pos, average_z_pos



#x, y, z, x1, y2, z3 = triangulation(2, [0, 0], [3, 0], 60, 150, 30)
#print("Calculated cartesian coordinates Method1: " , x, y, z, "\n")
#print("Calculated cartesian coordinates Method2: " , x1, y2, z3, "\n")


