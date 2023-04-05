from enum import Enum
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from control import *

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


def control(droneID: int, command: CMD, val: int = 0):
    print("Command: ", command)
    handle_control(command, val)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize all widgets
        self.droneID = QLabel("DroneID", self)
        self.drones = QComboBox(self)
        self.ARMButton = QPushButton("ARM", self)
        self.ARMButton.clicked.connect(self.control_clicked)
        self.DISARMButton = QPushButton("DISARM", self)
        self.DISARMButton.clicked.connect(self.control_clicked)
        self.LIFTOFFButton = QPushButton("LIFTOFF")
        self.LIFTOFFButton.clicked.connect(self.control_clicked)
        self.LIFTOFFVal = QLineEdit(self)
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

        # Add widgets to layout
        self.window = QWidget()
        self.layout = QGridLayout()
        self.layout.addWidget(self.droneID, 0, 0, alignment=Qt.AlignCenter)
        self.layout.addWidget(self.drones, 0, 1, 1, 1)
        self.layout.addWidget(self.ARMButton, 1, 0)
        self.layout.addWidget(self.DISARMButton, 2, 0)
        self.layout.addWidget(self.LIFTOFFButton, 3, 0)
        self.layout.addWidget(self.LIFTOFFVal, 3, 1)
        self.layout.addWidget(self.LANDButton, 4, 0)
        self.layout.addWidget(self.FBButton, 5, 0)
        self.layout.addWidget(self.FBVal, 5, 1)
        self.layout.addWidget(self.LRButton, 6, 0)
        self.layout.addWidget(self.LRVal, 6, 1)
        self.layout.addWidget(self.CDButton, 7, 0)
        self.layout.addWidget(self.CDVal, 7, 1)
        self.layout.addWidget(self.droneLoc, 8, 0, 1, 2, alignment=Qt.AlignCenter)
        
        
        # Add plot
        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        self.axes = self.fig.add_subplot(111, projection='3d')
        self.axes.set_xlim([-0.1, 0.1])
        self.axes.set_ylim([-0.1, 0.1])
        self.axes.set_zlim([-0.1, 0.1])
        self.layout.addWidget(self.canvas, 0, 2, 20, 1)
        
        self.window.setLayout(self.layout)



    # Show the interface
    def start_interface(self):
        print("Getting the control information from the user to send to the drones...")
        self.window.setWindowTitle("Drone Control Interface")

        self.timer = QTimer()
        self.location_update()
        self.timer.timeout.connect(self.location_update)
        self.timer.start(500)
        self.window.show()


    # Update drone location
    def location_update(self):
        aoa.lock.acquire()
        coord = aoa.drone_coord
        aoa.lock.release()
        # print("coordinates: ", coord)
        
        self.droneLoc.setText(f"Drone Location: {coord}")
        self.axes.cla()
        self.axes.set_xlim([-0.1, 0.1])
        self.axes.set_ylim([-0.1, 0.1])
        self.axes.set_zlim([-0.1, 0.1])
        self.axes.plot(coord[0],coord[1],coord[2], marker='x')
        self.canvas.draw()

    # Button click controller. Send control signal when value set
    def control_clicked(self):
        button = self.sender().text()
        # print("button: ", button)

        if button == "ARM":
            # control(self.drones.currentText(), CMD.ARM)
            start()
        elif button == "DISARM":
            # control(self.drones.currentText(), CMD.DISARM)
            disarm()
        elif button == "LIFTOFF":
            takeoff()
            # control(self.drones.currentText(), CMD.LIFTOFF, self.LIFTOFFVal.text())
        elif button == "LAND":
            land()
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
        
    
    


    
    



