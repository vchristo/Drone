#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <iostream>
#include<pthread.h>
#include "i2c_io.h"
#define MAX_THREAD 20

using namespace std;

pthread_t tid[MAX_THREAD];
int nThread=0;
uint8_t erro_dev[128][255]={{0},{0}};
i2c_io::i2c_io(){
	initBus();
}

i2c_io::~i2c_io(){
}

int retry = 3000;

int i2c_io::i2c_read( void *buf, int len, int16_t dev_add )
{
	// usleep(100000);
	int i = 0;
	int rc = 0;
	int ret=-1;

	while(i < retry){
		close(file);
		initBus();	

//		pthread_mutex_lock(&i2c_lock);
		ret = ioctl(file,I2C_SLAVE,dev_add);
		if(ret < 0) {
			close(file);
			initBus();
			usleep(1000);

			printf("******************************************************************************\n");
			printf("* Failed to acquire bus access and/or talk to slave %d i2c_read  com i = %d   *\n",dev_add,i);
			printf("******************************************************************************\n");
			exit( 1 );
			i++;
		} 
		else { 
			i=retry;
		}
	}
	if(i > 0)
		printf( "I2C read failed: %s no deve %d com i %d \n", strerror( errno ),dev_add,i );
	i=0;
	ret = read( file, buf, len );
//	pthread_mutex_unlock(&i2c_lock);
	if ( ret != len )
	{
		rc = -1;
	}   
	close(file);
	return rc;
}

int i2c_io::i2c_write( void *buf, int len, int16_t dev_add,uint8_t add,bool F )
{
	//   usleep(100000);

	int i=0;
	int j=0;
	int rc = -1;
	int ret=-1;

		while(i < retry){
			close(file);
			initBus();

//			pthread_mutex_lock(&i2c_lock);
			ret = ioctl(file,I2C_SLAVE,dev_add);
			if(ret < 0) {
				close(file);
				initBus();
				usleep(1000);	
				//	printf("******************************************************************************\n");
				//	printf("* Failed to acquire bus access and/or talk to slave %d i2c_Write com i = %d    *\n",dev_add,i);
				//	printf("******************************************************************************\n");
				//exit( 1 );
				i++;j++;
			}
			else 
				i=retry;

		}
	if(j>10){
		printf("******************************************************************************\n");
		printf("* Failed to acquire bus access and/or talk to slave %d i2c_Write com retry = %d    *\n",dev_add,j);
		printf("******************************************************************************\n");

	}
	j=0;
	i=0;

	if(F){
		ret = write( file, buf, len );
		if ( ret != len )
		{

			//	printf("******************************************************************************\n");  
			//	printf( "I2C write failed: %s no deve %d com i %d \n", strerror( errno ),dev_add,i );
			//	printf("******************************************************************************\n");   
			//		erro_dev[dev_add]=erro_dev[dev_add] + 1;
		}
	}else{
		unsigned char bufw[1];
		bufw[0]=add;

		ret = write( file, bufw, 1);
		if(ret != 1){
			printf("******************************************************************************\n");  
			printf( "I2C write failed on read : %s no deve %d com i %d \n", strerror( errno ),dev_add,i );
			printf("******************************************************************************\n");
		}
		if ( read( file, buf, len ) != len )
		{
			rc = -1;
		}

		rc = 0;
		close(file);    

//		pthread_mutex_unlock(&i2c_lock);
		return rc;

	}
//	pthread_mutex_unlock(&i2c_lock);

	rc = 0;
	close(file);    

	return rc;
}


void i2c_io::initBus(void)
{
	if((file = open("/dev/i2c-1",O_RDWR)) < 0) {
		printf("Failed to open the bus.");
	}
}
int16_t i2c_io::i2cReadI(unsigned char address,int16_t dev_add)
{
	unsigned char msb=0, lsb=0;
	unsigned char  reBuf[2] = {0,0};   
	//	unsigned char buf[1] = {address};

	//    i2c_write(buf,1,dev_add);

	if(i2c_write(reBuf, 2,dev_add,address,0)==0)
	{
		msb = reBuf[0];
		lsb = reBuf[1];
	}	
	return (int16_t) msb<<8 | lsb;;
}
char i2c_io::i2cReadC(unsigned char address,int16_t dev_add)
{
	//	unsigned char buf[1] = {address};
	char reBuf[1] = {address};
	//i2c_write(buf,1,dev_add);
	while(address == reBuf[0]) {
		i2c_write(reBuf,1,dev_add,address,0);
	}
	return reBuf[0];
}

void i2c_io::writeReg(unsigned char address, unsigned char val,int16_t dev_add)
{
	unsigned char buf[2] = {address, val};
	i2c_write(buf,2,dev_add,0,1);
}

int16_t i2c_io::readReg(unsigned char address,int16_t dev_add)
{
	unsigned char buf[1] = {address};
	int16_t reBuf[1] = {0};    
	//i2c_write(buf,1,dev_add);
	while(0 == reBuf[0]) {
		i2c_write(reBuf,1,dev_add,address,0);
	}  
	return reBuf[0];
}


void i2c_io::i2cWrite(int16_t dev_add,unsigned char *buffer,uint8_t size){

	//	pthread_create(&(tid[i]), NULL, &doSomeThing, NULL);
	i2c_write(buffer,size,dev_add,0,1);	 

}

void i2c_io::i2cReadBuf(int16_t dev_add,uint8_t *buffer,uint8_t len, unsigned char address){
	unsigned char buf[1];
	//buffer[0]=address;
	//	i2c_write(buf,1,dev_add);
	i2c_write(buffer,len,dev_add,address,0);
}



