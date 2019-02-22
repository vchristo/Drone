

#ifndef __I2C_IO_H__
#define __I2C_IO_H__

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <inttypes.h>

// Global lock, for I2C write/read sequences that must be atomic
static pthread_mutex_t i2c_lock;

class i2c_io
{
    public:
        i2c_io();
        ~i2c_io();
        void initBus(void);
		int16_t i2cReadI(unsigned char address,int16_t dev_add);
		char i2cReadC(unsigned char address,int16_t dev_add);
		void writeReg(unsigned char address, unsigned char val,int16_t dev_add);
		void i2cWrite(int16_t dev_add,unsigned char *buffer,uint8_t size);
		int16_t readReg(unsigned char address,int16_t dev_add);
		void i2cReadBuf(int16_t dev_add,uint8_t *buffer,uint8_t len, unsigned char address);
		
    private:
		int file;
        
        int i2c_write( void *buf, int len,int16_t dev_add,uint8_t add,bool F );
        int i2c_read( void *buf, int len,int16_t dev_add );

};

#endif
