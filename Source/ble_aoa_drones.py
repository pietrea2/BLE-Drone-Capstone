
# Main file for interfacing with all the other modules of the project

# AOA_get_location.py: get the location of the drone using PySerial
# triangulation.py: use the angular information from the AOA and convert to cartesian coordinates
# control_interface.py: terminal or graphical interface for controlling the drones (arming & moving around)
# control.py: main entry point for the control system for the drones

# import AOA_get_location as AOA
import AOA_get_location_VER_2 as AOA_V2
from control_interface import *
import control
import triangulation as TRI
import sys
import threading
import time 

# [x, y, z]
# AOA_V2.drone_coord
#AOA_V2.lock


def main():
    print("Running Main\n")

    
    #Run AOA location calc program
    thread = threading.Thread(target=AOA_V2.AOA_get_location, args=())
    thread.daemon = True
    thread.start()
    print("started thread")
    # thread.join()


    # while 1:
    #     AOA_V2.lock.acquire()
    #     print("acquired lock")
    #     print(AOA_V2.drone_coord)
    #     AOA_V2.lock.release()
    #     time.sleep(1)

    




    # Iniaialize the application

    app = QApplication(sys.argv)

    # Start the main window of the PyQt 
    interface_window = MainWindow()
    #interface_window.InitWindow()
    #interface_window.paintEvent()
    interface_window.show()
    sys.exit(app.exec())

    #thread.kill()


if __name__ == "__main__":
    main()