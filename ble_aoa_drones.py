
# Main file for interfacing with all the other modules of the project

# AOA_get_location.py: get the location of the drone using PySerial
# triangulation.py: use the angular information from the AOA and convert to cartesian coordinates
# control_interface.py: terminal or graphical interface for controlling the drones (arming & moving around)
# control.py: main entry point for the control system for the drones

import AOA_get_location
import control_interface
import control
import triangulation



def main():
    print("doing stuff")



if __name__ == "__main__":
    main()