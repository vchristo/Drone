# Drone
vitor.christo@gmail.com
This project is intended to obtain telemetry from an aircraft and can also send GPS coordinates to a mission (waypoint).
The codes are public domain.
- Cockpit python software, telemetry ground station, this can also send GPS coordinates to the destination (waypoint). This software can be improved. Socket connection must include the IP add of the drone on port 8888.

- Gpsnav, original from "I2CGPS - Inteligent GPS and NAV module for MultiWii by EOSBandi
* V2.2 "was modified to work on raspberry and connect via socket.
The GPS must be set to / dev / ttyAMA0, be sure to disable the linux terminal on this port.

- Drone_main, contains all the necessary code for drone operation, there are some routines to work with NRF24, by default this is running on TCP.

Order of load,
1st load gpsnav
2nd load drone_main
3rd cockpit.py
If you are not seeing the GPS data, do ^ C in gpsnav and reload it, make sure the coordinates appear in the drone_main terminal, some 4 or 5 attempts and it is ok.
If someone will fix the bug, please give me a copy.
Vitor.christo@gmail.com
