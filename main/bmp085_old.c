/*
Raspberry Pi Bosch BMP085 communication code.
By:      John Burns (www.john.geek.nz)
Date:    13 February 2013
License: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/

This is a derivative work based on:
	BMP085 Extended Example Code
	by: Jim Lindblom
	SparkFun Electronics
	date: 1/18/11
	license: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/
	Source: http://www.sparkfun.com/tutorial/Barometric/BMP085_Example_Code.pde


 compensated https://mbed.org/users/tkreyche/notebook/bmp085-pressure-sensor/
Compile with: gcc testBMP085.c -lm */



#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <math.h>
#include "bmp085.h"

#define BMP085_I2C_ADDRESS 0x77
#define COEFZ 21
static int k[COEFZ];
const unsigned char BMP085_OVERSAMPLING_SETTING = 3;

// Calibration values - These are stored in the BMP085
short int ac1;
short int ac2; 
short int ac3; 
unsigned short int ac4;
unsigned short int ac5;
unsigned short int ac6;
short int b1; 
short int b2;
short int mb;
short int mc;
short int md;

int b5; 

namespace BMP085
{
  int temperature, pressure,alt;
}

float calcAltitude(float pressure)
{
    float A = pressure/101325;
    float B = 1/5.25588;
    float C = pow(A,B);
    //printf("c = pow(a,b) = %d \n", &C);
    C = 1 - C;
    C = C /0.0000225577;
    return C;
}
// Open a connection to the bmp085
// Returns a file id
int bmp085_i2c_Begin()
{
	int fd;
	char fileName[] = "/dev/i2c-1";
	
	// Open port for reading and writing
	if ((fd = open(fileName, O_RDWR)) < 0){
		printf("i2c erro 1");
	//	exit(1);
	}
	// Set the port options and set the address of the device
	if (ioctl(fd, I2C_SLAVE, BMP085_I2C_ADDRESS) < 0) {	
		printf("i2c erro 1");					
		close(fd);
	//	exit(1);
	}

	return fd;
}

// Read two words from the BMP085 and supply it as a 16 bit integer
__s32 bmp085_i2c_Read_Int(int fd, __u8 address)
{
	__s32 res = i2c_smbus_read_word_data(fd, address);
	if (res < 0) {
		close(fd);
	//	exit(1);
	}

	// Convert result to 16 bits and swap bytes
	res = ((res<<8) & 0xFF00) | ((res>>8) & 0xFF);

	return res;
}

//Write a byte to the BMP085
void bmp085_i2c_Write_Byte(int fd, __u8 address, __u8 value)
{
	if (i2c_smbus_write_byte_data(fd, address, value) < 0) {
		close(fd);
	//	exit(1);
	}
}

// Read a block of data BMP085
void bmp085_i2c_Read_Block(int fd, __u8 address, __u8 length, __u8 *values)
{
	if(i2c_smbus_read_i2c_block_data(fd, address,length,values)<0) {
		close(fd);
		printf("bmp085_i2c_Read_Block ERRO\n");
	//	exit(1);
	}
}


void bmp085_Calibration()
{
	int fd = bmp085_i2c_Begin();
	ac1 = bmp085_i2c_Read_Int(fd,0xAA);
	ac2 = bmp085_i2c_Read_Int(fd,0xAC);
	ac3 = bmp085_i2c_Read_Int(fd,0xAE);
	ac4 = bmp085_i2c_Read_Int(fd,0xB0);
	ac5 = bmp085_i2c_Read_Int(fd,0xB2);
	ac6 = bmp085_i2c_Read_Int(fd,0xB4);
	b1 = bmp085_i2c_Read_Int(fd,0xB6);
	b2 = bmp085_i2c_Read_Int(fd,0xB8);
	mb = bmp085_i2c_Read_Int(fd,0xBA);
	mc = bmp085_i2c_Read_Int(fd,0xBC);
	md = bmp085_i2c_Read_Int(fd,0xBE);
	close(fd);
	usleep(10000);
}

// Read the uncompensated temperature value
unsigned int bmp085_ReadUT()
{
	unsigned int ut = 0;
	int fd = bmp085_i2c_Begin();

	// Write 0x2E into Register 0xF4
	// This requests a temperature reading
	bmp085_i2c_Write_Byte(fd,0xF4,0x2E);
	
	// Wait at least 4.5ms
	usleep(5000);

	// Read the two byte result from address 0xF6
	ut = bmp085_i2c_Read_Int(fd,0xF6);

	// Close the i2c file
	close (fd);
	usleep(10000);
	return ut;
}



unsigned int bmp085_ReadUP()
{
	unsigned int up = 0;
	int fd = bmp085_i2c_Begin();

	// Write 0x34+(BMP085_OVERSAMPLING_SETTING<<6) into register 0xF4
	// Request a pressure reading w/ oversampling setting
	bmp085_i2c_Write_Byte(fd,0xF4,0x34 + (BMP085_OVERSAMPLING_SETTING<<6));

	// Wait for conversion, delay time dependent on oversampling setting
	usleep((2 + (3<<BMP085_OVERSAMPLING_SETTING)) * 1000);

	// Read the three byte result from 0xF6
	// 0xF6 = MSB, 0xF7 = LSB and 0xF8 = XLSB
	__u8 values[3];
	bmp085_i2c_Read_Block(fd, 0xF6, 3, values);

	up = (((unsigned int) values[0] << 16) | ((unsigned int) values[1] << 8) | (unsigned int) values[2]) >> (8-BMP085_OVERSAMPLING_SETTING);

	// Close the i2c file
	close (fd);
	usleep(10000);
	return up;
}

// Calculate pressure given uncalibrated pressure
// Value returned will be in units of Pa
unsigned int bmp085_GetPressure(unsigned int up)
{
	int x1, x2, x3, b3, b6, p;
	unsigned int b4, b7;
  
	b6 = b5 - 4000;
	// Calculate B3
	x1 = (b2 * (b6 * b6)>>12)>>11;
	x2 = (ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((int)ac1)*4 + x3)<<BMP085_OVERSAMPLING_SETTING) + 2)>>2;
  
	// Calculate B4
	x1 = (ac3 * b6)>>13;
	x2 = (b1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (ac4 * (unsigned int)(x3 + 32768))>>15;
  
	b7 = ((unsigned int)(up - b3) * (50000>>BMP085_OVERSAMPLING_SETTING));
	if (b7 < 0x80000000)
		p = (b7<<1)/b4;
	else
		p = (b7/b4)<<1;
	
	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	p += (x1 + x2 + 3791)>>4;
  
	return p;
}

// Calculate temperature given uncalibrated temperature
// Value returned will be in units of 0.1 deg C
unsigned int bmp085_GetTemperature(unsigned int ut)
{
	int x1, x2;
  
	x1 = (((int)ut - (int)ac6)*(int)ac5) >> 15;
	x2 = ((int)mc << 11)/(x1 + md);
	b5 = x1 + x2;

	unsigned int result = ((b5 + 8)>>4);  

	return result;
}
/////////////////////////////////////////////////
// calculate compensated pressure
/////////////////////////////////////////////////
int movAvgIntZ(int input) {
 
    int cum = 0;
 
    for (int i = 0; i < COEFZ; i++) {
        k[i] = k[i+1];
    }
 
    k[COEFZ - 1] = input;
 
    for (int i = 0; i < COEFZ; i++) {
        cum += k[i];
    }
 
    return ( cum / COEFZ ) ;
}
int calcPress(int upp) {
 
    long pressure,x1,x2,x3,b3,b6;
    unsigned long b4, b7;
    int oversampling_setting = 3;
 
    unsigned long up = (unsigned long)upp;
 
    b6 = b5 - 4000;
    // calculate B3
    x1 = (b6*b6) >> 12;
    x1 *= b2;
    x1 >>=11;
 
    x2 = (ac2*b6);
    x2 >>=11;
 
    x3 = x1 +x2;
 
    b3 = (((((long)ac1 )*4 + x3) <<oversampling_setting) + 2) >> 2;
 
    // calculate B4
    x1 = (ac3* b6) >> 13;
    x2 = (b1 * ((b6*b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
 
    b7 = ((unsigned long)(up - b3) * (50000>>oversampling_setting));
    if (b7 < 0x80000000) {
        pressure = (b7 << 1) / b4;
    } else {
        pressure = (b7 / b4) << 1;
    }
 
    x1 = pressure >> 8;
    x1 *= x1;
    x1 = (x1 * 3038) >> 16;
    x2 = (pressure * -7357) >> 16;
    pressure += (x1 + x2 + 3791) >> 4;  // pressure in Pa
 
    return (pressure);
}

/*
int main(int argc, char **argv)
{
	
	bmp085_Calibration();
	temperature = bmp085_GetTemperature(bmp085_ReadUT());
	pressure = bmp085_GetPressure(bmp085_ReadUP());
	alt= calcAltitude(pressure);
	printf("Temperature\t%0.2fC \n", ((double)temperature)/10,0x00B0);
	printf("Pressure\t%0.2fhPa \n", ((double)pressure)/100);
	printf("Altitude\t%0.2fm\n", ((double)alt));
	return 0;
}
*/
