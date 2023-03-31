
# Main file for interfacing with all the other modules of the project

# AOA_get_location.py: get the location of the drone using PySerial
# triangulation.py: use the angular information from the AOA and convert to cartesian coordinates
# control_interface.py: terminal or graphical interface for controlling the drones (arming & moving around)
# control.py: main entry point for the control system for the drones

import AOA_get_location as AOA
from control_interface import *
import control
import triangulation as TRI
import sys



def main():
    print("Running main\n")

    #initialize the uBLox anchor connection 1st loop
    init_com_connection = True

    #while(1):

     #   azimuth, elevation = AOA.AOA_get_location_2(init_com_connection)
      #  init_com_connection = False

      #  x, y, z, x1, y2, z3 = TRI.triangulation(2, [0, 0], [3, 0], 90, int(azimuth), int(elevation))
      #  print("Azimuth: ", azimuth, "  Elevation: ", elevation, " ---> Calculated cartesian coordinates Method1: " , x, y, z, "\n")
        #print("Calculated cartesian coordinates Method2: " , x1, y2, z3, "\n")




    # Iniaialize the application
    app = QApplication([])

    # Start the main window of the PyQt 
    interface_window = MainWindow()
    interface_window.start_interface()
    #interface_window.InitWindow()
    #interface_window.paintEvent()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()