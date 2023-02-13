import math



def triangulation(num_of_anchors, azimuth1, azimuth2, elevation):
    print("triangulating the location of the drones to cartesian...")


    if(num_of_anchors == 2):


        #----------Law of Sines Method-------------------------------------------------
        #Will calculate the location of drone relative to anchor #1
        #Have:
        #   azimuth1 = theta
        #   90 - elevation = phi

        #Need to calculate rho:
        rho = 0
        
        theta = azimuth1
        phi = 90 - elevation


        x = rho * math.sin( math.radians(phi) ) * math.cos( math.radians(theta) )
        y = rho * math.sin( math.radians(phi) ) * math.sin( math.radians(theta) )
        z = rho * math.cos( math.radians(phi) )



        #-----------Method #2----------------------------------------------------------
        #Source:
        #https://www.omnicalculator.com/math/triangulation



