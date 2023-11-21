from enum import Enum
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtCore import Qt, QTimer
from control import *

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

import numpy as np

import AOA_get_location_VER_2 as aoa
import threading

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

import numpy as np

import AOA_get_location_VER_2 as aoa
import threading


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


# Class containing the control command to be send to the drone eventually
class Command():
    def __str__(self):
        if self.type in [CMD.ARM, CMD.DISARM]:
            return self.type
        else:
            return (f"{self.type}: {self.value}")

    # each command includes the command type (enum) and the value of the command (in metres)
    def __init__(self, command: CMD, value):
        self.type = command
        self.value = value


# def control(droneID: int, command: CMD, val: int = 0):
#     print("Command: ", command)
#     handle_control(command, val)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # fence: [[x_min, x_max],[y_min, y_max],[z_min, z_max]] (in cm)
        self.fence = [[0, 100], [0, 100], [-50, 50]]
        self.flying = False

        # Initialize all widgets
        # self.lim = QLineEdit(self)
        # self.droneID = QLabel("DroneID", self)
        # self.drones = QComboBox(self)
        self.ARMButton = QPushButton("ARM", self)
        self.ARMButton.clicked.connect(self.control_clicked)
        self.DISARMButton = QPushButton("DISARM", self)
        self.DISARMButton.clicked.connect(self.control_clicked)
        self.LIFTOFFButton = QPushButton("LIFTOFF")
        self.LIFTOFFButton.clicked.connect(self.control_clicked)
        # self.LIFTOFFVal = QLineEdit(self)
        self.LANDButton = QPushButton("LAND", self)
        self.LANDButton.clicked.connect(self.control_clicked)
        self.FBButton = QPushButton("FWD/BACK", self)
        self.FBButton.clicked.connect(self.control_clicked)
        self.FBVal = QLineEdit(self)
        self.LRButton = QPushButton("LEFT/RIGHT", self)
        self.LRButton.clicked.connect(self.control_clicked)
        self.LRVal = QLineEdit(self)
        self.CDButton = QPushButton("CLIMB/DSND", self)
        self.CDButton.clicked.connect(self.control_clicked)
        self.CDVal = QLineEdit()
        self.droneLoc = QLabel("Location: ", self)
        self.batt = QLabel("Battery: ", self)

        self.forw = QPushButton("F", self)
        self.forw.pressed.connect(lambda: self.rc_ble(0, 100, 0, 0))
        self.forw.released.connect(lambda: rc(0, 0, 0, 0))
        self.back = QPushButton("B", self)
        self.back.pressed.connect(lambda: self.rc_ble(0, -100, 0, 0))
        self.back.released.connect(lambda: rc(0, 0, 0, 0))
        
        self.right = QPushButton("R", self)
        self.right.pressed.connect(lambda: self.rc_ble(100, 0, 0, 0))
        self.right.released.connect(lambda: rc(0, 0, 0, 0))
        self.left = QPushButton("L", self)
        self.left.pressed.connect(lambda: self.rc_ble(-100, 0, 0, 0))
        self.left.released.connect(lambda: rc(0, 0, 0, 0))

        self.up = QPushButton("U", self)
        self.up.pressed.connect(lambda: self.rc_ble(0, 0, 100, 0))
        self.up.released.connect(lambda: rc(0, 0, 0, 0))
        self.down = QPushButton("D", self)
        self.down.pressed.connect(lambda: self.rc_ble(0, 0, -100, 0))
        self.down.released.connect(lambda: rc(0, 0, 0, 0))

        self.yawl = QPushButton("YL", self)
        self.yawl.pressed.connect(lambda: self.rc_ble(0, 0, 0, -100))
        self.yawl.released.connect(lambda: rc(0, 0, 0, 0))
        self.yawr = QPushButton("YR", self)
        self.yawr.pressed.connect(lambda: self.rc_ble(0, 0, 0, 100))
        self.yawr.released.connect(lambda: rc(0, 0, 0, 0))

        self.stop = QPushButton("STOP", self)
        self.stop.clicked.connect(stop)

        # Add widgets to layout
        self.window = QWidget()
        self.setCentralWidget(self.window)
        self.layout = QGridLayout(self.window)
        # self.layout.addWidget(self.droneID, 0, 0, alignment=Qt.AlignCenter)
        # self.layout.addWidget(self.drones, 0, 1, 1, 1)
        self.layout.addWidget(self.ARMButton, 1, 0)
        self.layout.addWidget(self.DISARMButton, 2, 0)
        self.layout.addWidget(self.LIFTOFFButton, 3, 0)
        # self.layout.addWidget(self.LIFTOFFVal, 3, 1)
        self.layout.addWidget(self.LANDButton, 4, 0)
        self.layout.addWidget(self.FBButton, 5, 0)
        self.layout.addWidget(self.FBVal, 5, 1)
        self.layout.addWidget(self.LRButton, 6, 0)
        self.layout.addWidget(self.LRVal, 6, 1)
        self.layout.addWidget(self.CDButton, 7, 0)
        self.layout.addWidget(self.CDVal, 7, 1)
        self.layout.addWidget(self.droneLoc, 8, 0, 1, 3, alignment=Qt.AlignCenter)
        self.layout.addWidget(self.forw, 9,1)
        self.layout.addWidget(self.back, 10,1)
        self.layout.addWidget(self.left, 10,0)
        self.layout.addWidget(self.right, 10,2)
        self.layout.addWidget(self.up, 9,3)
        self.layout.addWidget(self.down, 10,3)
        self.layout.addWidget(self.yawl, 9,0)
        self.layout.addWidget(self.yawr, 9,2)
        self.layout.addWidget(self.batt, 11,0,1,2)
        self.layout.addWidget(self.stop, 12,0,1,4)

        
        # Add plot
        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        self.axes = self.fig.add_subplot(211, projection='3d')
        self.axes.set_xlim(self.fence[0])
        self.axes.set_ylim(self.fence[1])
        self.axes.set_zlim(self.fence[2])
        self.layout.addWidget(self.canvas, 0, 4, 40, 2)

        # self.fig2d = Figure()
        # self.canvas2d = FigureCanvas(self.fig2d)
        self.axes2d = self.fig.add_subplot(212)
        self.axes2d.set_xlim(self.fence[0])
        self.axes2d.set_xlim(self.fence[1])
        
        self.window.setLayout(self.layout)

        self.start_interface()


    # def keyPressEvent(self, event):
    #     print("smth happened")
    #     if event.key() == Qt.Key_Space:
    #         self.test_method()

    # def test_method(self):
    #     print('Space key pressed')
        
    # def keyPressEvent(self, eventQKeyEvent):
    #     key = eventQKeyEvent.key()    
    #     print("Keypress")
    #     if key == Qt.Key_Up:
    #         print("Up pressed")
    #     elif key == Qt.Key_Left:
    #         print("Left pressed") 
    #     elif key == Qt.Key_Right:
    #         print("Right pressed")
    #     elif key == Qt.Key_Down:
    #         print("Down pressed") 
    

    # Show the interface
    def start_interface(self):
        print("Getting the control information from the user to send to the drones...")
        self.setWindowTitle("Drone Control Interface")

        self.timer = QTimer()
        self.location_update()
        self.timer.timeout.connect(self.location_update)
        self.timer.start(500)
        # self.window.show()
        print("setup done")


    # Update drone location
    def location_update(self):
        aoa.lock.acquire()
        coord = aoa.drone_coord
        aoa.lock.release()
        # print("coordinates: ", coord)
        
        self.droneLoc.setText(f"Drone Location: {coord}")
        self.axes.cla()
        self.axes.set_xlim(self.fence[0])
        self.axes.set_ylim(self.fence[1])
        self.axes.set_zlim(self.fence[2])
        self.axes.plot(coord[0],coord[1],coord[2], marker='x')

        self.axes2d.cla()
        self.axes2d.set_xlim(self.fence[0])
        self.axes2d.set_ylim(self.fence[1])
        self.axes2d.plot(coord[0],coord[1], marker='x')
        self.canvas.draw()

        # Fix location if the drone is outside the geofence
        if self.flying:
            target = self.fix_location(coord)
            self.batt.setText(f"Battery: {get_battery()}")
            need_fix = False
            for coord in target:
                if abs(coord) >= 20: 
                    need_fix = True
                    break
            if need_fix:
                print("Need drone location to go to: ", target)
                # go(target)
            else:
                stop()


    # Returns the coordinate change needed to fix the drone's location 
    def fix_location(self, coord, target=None):
        # Need to go to specific target
        if target:
            for i in range(3):
                target[i] = int(target[i] - coord[i])
            return target
        
        # Need to stay within geofence
        target = [0,0,0]
        for i in range(3):
            if coord[i] <= self.fence[i][0]:
                target[i] = int(self.fence[i][0] - coord[i] + 20)
            elif coord[i] >= self.fence[i][1]:
                target[i] = int(self.fence[i][1] - coord[i] - 20)

            if abs(target[i]) >= 1000: target[i] = 0

        return target

    def rc_ble(self, roll, pitch, throttle, yaw):
        # if roll != 0:
        # elif pitch
        rc(roll, pitch, throttle, yaw)


    # Button click controller. Send control signal when value set
    # fence: [[x_min, x_max],[y_min, y_max],[z_min, z_max]] (in cm)
    def control_clicked(self):
        button = self.sender().text()
        # print("button: ", button)

        if button == "ARM":
            # control(self.drones.currentText(), CMD.ARM)
            start()
        elif button == "DISARM":
            # control(self.drones.currentText(), CMD.DISARM)
            disarm()
            self.flying = False
        elif button == "LIFTOFF":
            takeoff()
            self.flying = True
            # control(self.drones.currentText(), CMD.LIFTOFF, self.LIFTOFFVal.text())
        elif button == "LAND":
            land()
            self.flying = False
            # control(self.drones.currentText(), CMD.LAND)
        elif button == "FWD/BACK":
            print("Value: ", self.FBVal.text())
            val = int(self.FBVal.text())
            cmd = forward(val) if val > 0 else backward(-val)
            # control(self.drones.currentText(), cmd, self.FBVal.text())
        elif button == "LEFT/RIGHT":
            val = int(self.LRVal.text())
            cmd = left(val) if val > 0 else right(-val)
            # control(self.drones.currentText(), cmd, self.LRVal.text())
        elif button == "CLIMB/DSND":
            val = int(self.CDVal.text())
            cmd = up(val) if val > 0 else down(-val)
            # control(self.drones.currentText(), cmd, self.CDVal.text())
        
    
    


    
    



