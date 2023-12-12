# Bluetooth-Localized Drones

Locating and controlling drones using Bluetooth AOA (Angle Of Arrival) Technology.


University of Toronto Computer/Electrical Engineering Capstone Project proposed and developed by:

- Mohammad Ahmed
-	Jun Ho Sung
-	Adam Pietrewicz
-	Natalia Sydorenko

Supervised by Professor Lacra Pavel from the University of Toronto.

&nbsp;

### Motivation:
Indoor drones can’t use GPS for localization due to weak signal, therefore an **Indoor Positioning System** is usually implemented using ultrasonic beacons placed around a room

![GPS](https://github.com/pietrea2/BLE-Drone-Capstone/blob/main/Documents/images/gps.png)

Our team explored using Bluetooth Low Energy (BLE) technology for a drone/swarm drone localization system because it is more energy efficient and modern.

&nbsp;

### BLE AOA Localization Technology
BLE provides two technologies: Angle of Arrival (AOA) and Angle of Departure (AOD).
These technologies locate Bluetooth signal origins by calculating the angle at which signals
are received by an array of antennas down to centimeter accuracy which is
competitively better than the accuracy range for GPS and ultrasonic solutions.

![BLE AOA](https://github.com/pietrea2/BLE-Drone-Capstone/blob/main/Documents/images/BLE_AOA.png)
![Accuracy of Localization Technologies](https://github.com/pietrea2/BLE-Drone-Capstone/blob/main/Documents/images/accuracy.png)

&nbsp;

### Project Goal
To develop a scalable drone infrastructure that utilizes BLE AOA technology to accurately locate indoor drones.

_Complete Project Goals and Requirements (Functions, Constraints, Objectives) are specified in the project documents._

&nbsp;

### Final Design
![System Overview Diagram](https://github.com/pietrea2/BLE-Drone-Capstone/blob/main/Documents/images/final_design.png)
