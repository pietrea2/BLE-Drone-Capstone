import math




def triangulation(num_of_anchors, pos_anchor_1, pos_anchor_2, azimuth1, azimuth2, elevation):
    print("Triangulating the location of the drones to cartesian:\n\n")

    #[x, y] coordinate on floor (in meters)
    pos_anchor_1 = [0, 0]
    x1 = pos_anchor_1[0]
    y1 = pos_anchor_1[1]

    pos_anchor_2 = [3, 0]
    x2 = pos_anchor_2[0]
    y2 = pos_anchor_2[1]

    theta = azimuth1
    phi = 90 - elevation

    print("uBLox Anchor positions: (" , x1, ", " , y1 , ") (" , x2 , ", " , y2 , ")\n")

    if(num_of_anchors == 2):

        method_1 = False
        if(method_1):
            #----------Law of Sines Method-------------------------------------------------
            #Will calculate the location of drone relative to anchor #1
            #Have:
            #   azimuth1 = theta
            #   90 - elevation = phi

            #Need to calculate rho:
            rho = 0            
            


            x = rho * math.sin( math.radians(phi) ) * math.cos( math.radians(theta) )
            y = rho * math.sin( math.radians(phi) ) * math.sin( math.radians(theta) )
            z = rho * math.cos( math.radians(phi) )


        else:
            #-----------Method #2----------------------------------------------------------
            #Source:
            #https://www.omnicalculator.com/math/triangulation

            #x = ( (0 - pos_anchor_2[1]) + pos_anchor_2[0]*math.tan(math.radians(azimuth2)) )
            x = ( (y1 - y2) + x2*math.tan(math.radians(azimuth2)) - x1*math.tan(math.radians(azimuth1)) ) / \
                                ( math.tan(math.radians(azimuth2)) - math.tan(math.radians(azimuth1)) )

            y = ( y1*math.tan(math.radians(azimuth2)) - y2*math.tan(math.radians(azimuth1)) - (x1 - x2)*math.tan(math.radians(azimuth2))*math.tan(math.radians(azimuth1)) ) / \
                                                                                                    ( math.tan(math.radians(azimuth2)) - math.tan(math.radians(azimuth1)) )

            z =  math.sqrt( math.pow(x, 2) + math.pow(y, 2) ) / math.tan(math.radians(phi))


        return x, y, z



x, y, z = triangulation(2, [0, 0], [3, 0], 60, 150, 30)
print("Calculated cartesian coordinates: " , x, y, z, "\n")