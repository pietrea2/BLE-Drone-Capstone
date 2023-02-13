from enum import Enum
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
import control


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


def control(droneID: int, command: Command, val: int = 0):
    print("Command: ", command)




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
        self.window.setLayout(self.layout)


    # Show the interface
    def start_interface(self):
        print("Getting the control information from the user to send to the drones...")
        self.window.show()
        
    # Button click controller. Send control signal when value set
    def control_clicked(self):
        button = self.sender().text()
        # print("button: ", button)

        if button == "ARM":
            control(self.drones.currentText(), CMD.ARM)
        elif button == "DISARM":
            control(self.drones.currentText(), CMD.DISARM)
        elif button == "LIFTOFF":
            control(self.drones.currentText(), CMD.LIFTOFF, self.LIFTOFFVal.text())
        elif button == "LAND":
            control(self.drones.currentText(), CMD.LAND)
        elif button == "FWD/BACK":
            cmd = CMD.FORWARD if int(self.LIFTOFFVal.text()) > 0 else CMD.BACKWARD
            control(self.drones.currentText(), cmd, self.FBVal.text())
        elif button == "LEFT/RIGHT":
            cmd = CMD.RIGHT if int(self.LRVal.text()) > 0 else CMD.LEFT
            control(self.drones.currentText(), cmd, self.LRVal.text())
        elif button == "CLIMB/DSND":
            cmd = CMD.CLIMB if int(self.CDVal.text()) > 0 else CMD.DESCEND
            control(self.drones.currentText(), cmd, self.CDVal.text())
        
    
    


    
    



