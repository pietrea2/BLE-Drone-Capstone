# from control_interface import *
# from control_interface import CMD
from tello import *

from enum import Enum

# All available command types
class CMD(Enum):
        ARM = 0
        DISARM = 1
        LIFTOFF = 2
        LAND = 3
        CLIMB = 4
        DESCEND = 5
        LEFT = 6
        RIGHT = 7
        FORWARD = 8
        BACKWARD = 9

def handle_control(command, val):
    # print("figuring out how to control the drone...")

    match command:
        case CMD.ARM:
            print("strating drone")
            start()
        case CMD.DISARM:
            disarm()
        case CMD.LIFTOFF:
            takeoff()
        case CMD.LAND:
            land()
        case CMD.CLIMB:
            up()
        case CMD.DESCEND:
            down()
        case CMD.LEFT:
            left()
        case CMD.RIGHT:
            right()
        case CMD.FORWARD:
            forward()
        case CMD.BACKWARD:
            backward()

         