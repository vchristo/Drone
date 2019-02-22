#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "giro_acc.h"
//HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h 
#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;


// orientation/motion vars
static Quaternion q;           // [w, x, y, z]         quaternion container
static VectorInt16 aa;         // [x, y, z]            accel sensor measurements
static VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
static VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
static VectorFloat gravity;    // [x, y, z]            gravity vector
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector




// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup_giro_acc() {
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
      //  mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
void giro_loop(float* ypr ) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          //  printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
        #endif

      //  printf("\n");
    }
}

//int main() {
//    setup_giro_acc();
//    float y_p_r[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//    usleep(100000);
//    for (;;)
//        giro_loop(y_p_r);
//		printf("ypr  %7.2f %7.2f %7.2f    ", y_p_r[0] * 180/M_PI, y_p_r[1] * 180/M_PI, y_p_r[2] * 180/M_PI);
//    return 0;
//}

