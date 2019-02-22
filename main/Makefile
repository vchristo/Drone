#############################################################################
#
# Makefile for librf24 examples on Raspberry Pi
#
# License: GPL (General Public License)
# Author:  gnulnulf <arco@appeltaart.mine.nu>
# Date:    2013/02/07 (version 1.0)
#
# Description:
# ------------
# use make all and make install to install the examples
# You can change the install directory by editing the prefix line
#
prefix := /usr/local
CFLAGS	:= -g -DMALLOC_DEBUG
# The recommended compiler flags for the Raspberry Pi
#CCFLAGS=-Ofast -g3 -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s
#CCFLAGS=

# define all programs
all: i2c_io barometer compass giro_acc
	@rm -f drone_main
#	${CCPREFIX}g++-4.6 pi_ppm_io.cpp -lpigpio -lrt -std=c++0x -lpthread -g3 -o pi_ppm_io
     
	${CCPREFIX}g++ drone_main.c *.o   -lpigpio -lrt -std=c++0x -lpthread -lm  -o drone_main -DDMP_FIFO_RATE=9 -Wall -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`
# ${CCPREFIX}g++ drone_main.c *.o -I /home/pi/src/RF24/RPi/RF24/  -lpigpio -lrt -std=c++0x -lpthread -lm -lrf24-bcm  -lrf24network -o drone_main
	


#	g++ ${CCFLAGS} -Wall -lpigpio -lrt -std=c++0x -lpthread  -I../ -lrf24-bcm  -lrf24network $@.cpp -o $@
ifneq ($(MAKECMDGOALS),clean)
include $(SRC:.c=.d) # always include dependency files for all .c files
endif
clean:
	rm -rf drone_main *.o





giro_acc: 
	${CCPREFIX}g++ -c I2Cdev.cpp MPU6050.cpp giro_acc.cpp -std=c++0x

compass: heading calibrate rotate i2c_ops diag bmp085 hmc5883l i2c_io

barometer:
	${CCPREFIX}g++ -c Barometer.cpp -std=c++0x
i2c_io:
	${CCPREFIX}g++ -c i2c_io.c -std=c++0x

hmc5883l:
	${CCPREFIX}g++ -c hmc5883l.c -std=c++0x

bmp085:
	${CCPREFIX}g++ -c bmp085.c -std=c++0x

diag:
	${CCPREFIX}g++ -c diag.c -std=c++0x

i2c_ops:
	${CCPREFIX}g++ -c i2c_ops.c -std=c++0x

rotate:
	${CCPREFIX}g++ -c rotate.c -std=c++0x

calibrate:
	${CCPREFIX}g++ -c calibrate.c -std=c++0x

heading:
	${CCPREFIX}g++ -c heading.c -std=c++0x


