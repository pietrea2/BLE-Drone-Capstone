
# Main file for interfacing with all the other modules of the project

# AOA_get_location.py: get the location of the drone using PySerial
# triangulation.py: use the angular information from the AOA and convert to cartesian coordinates
# control_interface.py: terminal or graphical interface for controlling the drones (arming & moving around)
# control.py: main entry point for the control system for the drones

import AOA_get_location
from control_interface import *
import control
import triangulation
import threading
import sys


def main():
    print("doing stuff")

    # Iniaialize the application
    app = QApplication([])

    # Start the main window of the PyQt 
    interface_window = MainWindow()
    interface_window.start_interface()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()