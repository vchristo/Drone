all: leadfilter PICtrl PIDCtrl HardwareSerial
	${CCPREFIX}g++-4.6 I2C_GPS_NAV.cpp LeadFilter.o PIDCtrl.o PICtrl.o HardwareSerial.o -std=c++0x -pthread -g3 -o gpsnav

clean:
	rm -f LeadFilter.o PIDCtrl.o PICtrl.o HardwareSerial.o gpsnav

leadfilter:
	${CCPREFIX}g++-4.6 -c LeadFilter.cpp -std=c++0x -pthread

PICtrl:
	${CCPREFIX}g++-4.6 -c PICtrl.cpp -std=c++0x -pthread

PIDCtrl:
	${CCPREFIX}g++-4.6 -c PIDCtrl.cpp -std=c++0x -pthread

HardwareSerial:	
	${CCPREFIX}g++-4.6 -c HardwareSerial.cpp -std=c++0x -pthread

