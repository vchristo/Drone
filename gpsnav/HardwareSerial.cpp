/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/
#include "HardwareSerial.h"
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyAMA0"
#define _POSIX_SOURCE 1         //POSIX compliant source
#define FALSE 0
#define TRUE 1
   struct termios oldtio, newtio;       //place for old and new port settings for serial port
   struct termios oldkey, newkey;       //place tor old and new port settings for keyboard teletype
   struct sigaction saio;               //definition of signal action
//class HardwareSerial;
typedef uint8_t byte;
volatile int STOP=FALSE;
int fd;
void signal_handler_IO(int status);    //definition of signal handler
int wait_flag=TRUE;                     //TRUE while no signal received
char devicename[80]={MODEMDEVICE};  /*AMA0*/
long Baud_Rate = 38400;         // default Baud Rate (110 through 38400)
long BAUD;                      // derived baud rate from command line
long DATABITS;
long STOPBITS;
long PARITYON;
long PARITY;
int Data_Bits = 8;              // Number of data bits
int Stop_Bits = 1;              // Number of stop bits
int Parity = 0;                 // Parity as follows:
                  // 00 = NONE, 01 = Odd, 02 = Even, 03 = Mark, 04 = Space
int Format = 4;
FILE *input;
FILE *output;
int status;

// this next line disables the entire HardwareSerial.cpp, 
// this is so I can support Attiny series and any other chip without a uart

// SerialEvent functions are weak, so when the user doesn't define them,
// the linker just sets their address to 0 (which is checked below).
// The Serialx_available is just a wrapper around Serialx.available(),
// but we can refer to it weakly so we don't pull in the entire
// HardwareSerial instance if the user doesn't also refer to it.

 void serialEvent() __attribute__((weak));
 bool Serial0_available() __attribute__((weak));

void serialEventRun(void)
{
  if (Serial0_available && serialEvent && Serial0_available()) serialEvent();
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////


// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerial::begin(unsigned long baud, uint8_t config)
{
	/*
	int fd = open(devicename, O_RDWR);
	struct termios options;
	tcgetattr(fd, &options);
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);
	tcsetattr(fd, TCSANOW, &options);
	close(fd);
	*/

	switch (baud)
	{
		case 115200:
			BAUD  = B115200;
			break;
		case 57600:
			BAUD  = B57600;
			break;
		case 38400:
		default:
			BAUD = B38400;
			break;
		case 19200:
			BAUD  = B19200;
			break;
		case 9600:
			BAUD  = B9600;
			break;
		case 4800:
			BAUD  = B4800;
			break;
		case 2400:
			BAUD  = B2400;
			break;
		case 1800:
			BAUD  = B1800;
			break;
		case 1200:
			BAUD  = B1200;
			break;
		case 600:
			BAUD  = B600;
			break;
		case 300:
			BAUD  = B300;
			break;
		case 200:
			BAUD  = B200;
			break;
		case 150:
			BAUD  = B150;
			break;
		case 134:
			BAUD  = B134;
			break;
		case 110:
			BAUD  = B110;
			break;
		case 75:
			BAUD  = B75;
			break;
		case 50:
			BAUD  = B50;
			break;
	}  //end of switch baud_rate

	switch (config)
	{
		case (0x00):
			Data_Bits=5;
			Stop_Bits=1;
			Parity=0;
			break;
		case (0x02):
			Data_Bits=6;
			Stop_Bits=1;
			Parity=0;
			break;
		case (0x04):
			Data_Bits=7;
			Stop_Bits=1;
			Parity=0;
			break;
		case (0x06):
			Data_Bits=8;
			Stop_Bits=1;
			Parity=0;
			break;
		case (0x08):
			Data_Bits=5;
			Stop_Bits=2;
			Parity=0;
			break;
		case (0x0A):
			Data_Bits=6;
			Stop_Bits=2;
			Parity=0;
			break;
		case (0x0C):
			Data_Bits=7;
		Stop_Bits=2;
		Parity=0;
	break;
	case (0x0E):
		Data_Bits=8;
		Stop_Bits=2;
		Parity=0;
	break;
	case (0x20):
		Data_Bits=5;
		Stop_Bits=1;
		Parity=2;
	break;
	case (0x22):
		Data_Bits=6;
		Stop_Bits=1;
		Parity=2;
	break;
	case (0x24):
		Data_Bits=7;
		Stop_Bits=1;
		Parity=2;
	break;
	case (0x28):
		Data_Bits=5;
		Stop_Bits=2;
		Parity=2;
	break;
	case (0x2A):
		Data_Bits=6;
		Stop_Bits=2;
		Parity=2;
	break;
	case (0x2C):
		Data_Bits=7;
		Stop_Bits=2;
		Parity=2;
	break;	
	case (0x2E):
		Data_Bits=8;
		Stop_Bits=2;
		Parity=2;
	break;	
	case (0x30):
		Data_Bits=5;
		Stop_Bits=1;
		Parity=1;
	break;
	case (0x32):
		Data_Bits=6;
		Stop_Bits=1;
		Parity=1;
	break;
	case (0x34):
		Data_Bits=7;
		Stop_Bits=1;
		Parity=1;
	break;
	case (0x36):
		Data_Bits=8;
		Stop_Bits=1;
		Parity=1;
	break;
	case (0x38):
		Data_Bits=5;
		Stop_Bits=2;
		Parity=1;
	break;
	case (0x3A):
		Data_Bits=6;
		Stop_Bits=2;
		Parity=1;
	break;
	case (0x3C):
		Data_Bits=7;
		Stop_Bits=2;
		Parity=1;
	break;
	case (0x3E):
		Data_Bits=8;
		Stop_Bits=2;
		Parity=1;
	break;
	

}

      switch (Data_Bits)
      {
         case 8:
         default:
            DATABITS = CS8;
            break;
         case 7:
            DATABITS = CS7;
            break;
         case 6:
            DATABITS = CS6;
            break;
         case 5:
            DATABITS = CS5;
            break;
      }  //end of switch data_bits
      switch (Stop_Bits)
      {
         case 1:
         default:
            STOPBITS = 1;
            break;
         case 2:
            STOPBITS = 2;
            break;
      }  //end of switch stop bits
      switch (Parity)
      {
         case 0:
         default:                       //none
            PARITYON = 0;
            PARITY = 0;
            break;
         case 1:                        //odd
            PARITYON = PARENB;
            PARITY = PARODD;
            break;
         case 2:                        //even
            PARITYON = PARENB;
            PARITY = 0;
            break;
      }  //end of switch parity
 //open the device(com port) to be non-blocking (read will return immediately)
      fd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0)
      {
         perror(devicename);
        // exit(-1);
      }

      //install the serial handler before making the device asynchronous
      saio.sa_handler = signal_handler_IO;
      sigemptyset(&saio.sa_mask);   //saio.sa_mask = 0;
      saio.sa_flags = 0;
      saio.sa_restorer = NULL;
      sigaction(SIGIO,&saio,NULL);

      // allow the process to receive SIGIO
      fcntl(fd, F_SETOWN, getpid());
      // Make the file descriptor asynchronous (the manual page says only
      // O_APPEND and O_NONBLOCK, will work with F_SETFL...)
      fcntl(fd, F_SETFL, FASYNC);

      tcgetattr(fd,&oldtio); // save current port settings 
      // set new port settings for canonical input processing 
      newtio.c_cflag = BAUD | CRTSCTS | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
      newtio.c_iflag = IGNPAR;
      newtio.c_oflag = 0;
      newtio.c_lflag = 0;       //ICANON;
      newtio.c_cc[VMIN]=1;
      newtio.c_cc[VTIME]=0;
      tcflush(fd, TCIFLUSH);
      tcsetattr(fd,TCSANOW,&newtio);
}


void signal_handler_IO (int status)
{
//    printf("received SIGIO signal.\n");
   wait_flag = FALSE;
}

void HardwareSerial::end()
{
	close(fd);
}

int HardwareSerial::peek(void)
{
/*
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    return _rx_buffer[_rx_buffer_tail];
  }
*/
return 0;
}

int HardwareSerial::read(void)
{
uint8_t c[2];
  int ret;
  fd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0)
      {
         perror(devicename);
         exit(-1);
      }
	  ret=::read(fd, &c[0], sizeof(char));
  //int n = read (fd, &c[0], sizeof (char));
  close(fd);
  if(ret>0)
  	return c[0];
  else
  	return ret;
}

void HardwareSerial::flush()
{
	fd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0)
      {
         perror(devicename);
       //  exit(-1);
      }
  tcflush(fd,TCIOFLUSH);
}

size_t HardwareSerial::write(uint8_t c)
{
    int fd = open (devicename, O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0)
		{
        	perror(devicename);
        	return 0;
		}

		::write(fd, &c, sizeof (char));   
  	return 1;
}

size_t HardwareSerial::write(const char *c)
{
	if(c == NULL)
		return -1;

    int fd = open (devicename, O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0)
		{
        	perror(devicename);
        	return 0;
		}

		while(*c != 0) {
			::write(fd, c, sizeof (char));   
			c++;
		}
  	return 1;
}

HardwareSerial::HardwareSerial() {}
HardwareSerial::~HardwareSerial() {}
