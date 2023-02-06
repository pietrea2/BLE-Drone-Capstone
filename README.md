# BLE-AOA-Drone-Swarms
Using BLE AOA to locate and control a bunch of drones!


## AOA_get_location.py
Connect to the anchor using COM serial port.
Using AT commands found in https://content.u-blox.com/sites/default/files/XPLR-AOA-Explorer-kits_UserGuide_UBX-21004616.pdf, we will poll (with a loop, timer, etc) the location of each drone to use for the control system.
The communication will be handled with PySerial.

## control_interface.py
The drone will be facing only one direction to simplify control commands
Main GUI/Text interface for sending commands to the drone. The commands will be likely as follows:
* Arm/Disarm drone
* Liftoff/Land
* Climb <# meters>
* Descend <# meters>
* Move left/right/forward/backward <# meters>
* Move left <# meters>

## control.py


## triangulation.py
