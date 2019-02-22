// protocolo oi 21/11/2014 20140129019771
//#include "A_radio_PS2_RX_1.h"
//pthread
#include <thread>
#include <string.h>    //strlen
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <netinet/in.h>
#include <sys/time.h> //FD_SET, FD_ISSET, FD_ZERO macros

#include <sys/resource.h>
#include <sys/stat.h>
#include <math.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <unistd.h>
#include <bcm2835.h>
#include <string>
//#include <chrono>
//#include <time.h>
//#include <RF24/RF24.h>
//#include <RF24Network/RF24Network.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>


#include <errno.h>



#include "diag.h"
#include "debug.h"
#include "hmc5883l.h"
#include "calibrate.h"
#include "heading.h"
#include "Barometer.h"
//#include "bmp085.h"
#include "i2c_io.h"
#include "giro_acc.h"

//#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"
#define GIRO_ACC 1
#define COMPASS 1
#define TCP_SERVER 1
#define RADIO 1
#define OUTPUT 1
#define BAROMETER 1
#define TRUE   1
#define FALSE  0
#define PORT 8888
//#define TCPSTICK
#define TCP_SOCKS
//#define NRF24_TELE
#define ARDUINO_ADDRESS 18
#define CONFIG_FILE ".precal.txt"
#define TELEMETRY 1
#define JOYSTICK 2
#define FLYDATA 3

bool TELEMETRY_LINK=false;
bool JOYSTICK_LINK=false;
bool FLAYDATA_LINK=false;
bool powerOn = true;
float atYaw,atRol,atPit;
char telemetry_buf[2000];
char wp_buff[1025];
char gps_buff[2000];
double gpsLat=0,gpsLon=0;
int gpsAlt=0,gpsNb=0;
short gpsF3d=0,gpsF2d=0,gpsGm=0,gpsNm=0,gpsNs=0;
float volt1=0,volt2=0;
bool TCP_LINK = true;
bool LINK = false;
bool WP_FLAG=false;
int pressZ;
float y_p_r[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float gCalib=0;
/****************************************************************
*              the payload struct to be sent via RF24 
*
*****************************************************************/

#ifdef NRF24_TELE


	RF24 radio(RPI_V2_GPIO_P1_22, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ); 
	RF24Network network(radio);

struct payload_t{

	bool triangle_bl;
	bool triangle_bh;
	bool square_bl;
	bool square_bh;
	bool circle_bl;
	bool circle_bh;
	bool cross_bl;
	bool cross_bh;
	uint8_t servo_1;
	uint8_t servo_2;
	uint8_t servo_3;
	uint8_t servo_4;

}pl;

struct payload_F{  // payload para tipo F (feedback)
	uint16_t sysBat;   // temperatura das baterias
	uint16_t motorBat;    // mode.0 = indoor , 1 go to waypoint
	float tmp;
	float alt;
}fb;
const uint16_t this_node = 0;
const uint16_t other_node = 1;
const unsigned long interval = 200; //ms
const unsigned long linkDown = 200; //ms

#endif
#ifdef TCP_SOCKS

struct payload_tcp{

	uint8_t throtler;
	uint8_t yaw;
	uint8_t pitch;
	uint8_t roll;
	uint8_t aux1;
	uint8_t aux2;
	uint8_t flmode;
	
}Ptcp;

#endif

#ifdef CONSOLE_STICK
typedef struct{
	uint8_t thr;
	uint8_t yaw;
	uint8_t pit;
	uint8_t rol;
	uint8_t sw1;
	uint8_t sw2;
}stick,*pstick;
#endif

    float temperature=-1;
    float pressure=-1;
    float atm=-1;
    float alt=-1;
    float compass = -1;
uint16_t vbat;
uint16_t vbatM;
int fdescriptor;
unsigned long last_sent;
unsigned long last_sent_led;
                             // A pulse starts with a low signal of fixed width (0.3ms), 
                             // followed by a high signal for the remainder of the pulse.
                             // Total pulse width is proportional to servo position (1 to 2ms)
int pulseStart = 300;        // pulse start width in microseconds
int pulseMin = 724;          // pulse minimum width minus start in microseconds
int pulseMax = 2048;         // pulse maximum width in microseconds
int conversionFactor = 5.7;   // (pulseMax - pulseMin - pulseStart)/180

                             // A frame is a succession of pulses, in order of channels,
                             // followed by a synchronisation pulse to fill out the frame.
                             // A frame's total length is fixed (20ms)
unsigned int frameLength = 20;        // The duration in millisecs of a frame

long lastFrame = 0;          // The time in millisecs of the last frame
int channelNumber = 7;       // Number of channels to send (keep below frameLength/pulseMax)
uint8_t channel[7];              // Values to send on channels (duration of pulse minus start, in microseconds)
int i;                       // Counter in for loop
int j = 0;                   // Counter for servo updates

//#define IN  0
//#define OUT 1
//#define POUT 23 
#define PIN RPI_GPIO_P1_12  // gpio18
#define EPSILON 1E-8

void writeByte(uint8_t *buf);
int load_precal(hmc5883l_state_t * const state, calibration_t * const p, FILE *cfg)
{
   size_t len;
   char *str=NULL, buf[2048], *tok, *sp;
   float var;
   int i=0;

   if(getline(&str, &len, cfg)<1) {
	   fprintf(stderr, "ERROR: reading ~/.precal.txt");
	   free(str);
	   return -1;
   }
//   printf(str);
//   printf(",");
   strncpy(buf, str, sizeof(buf));
   str = buf;
   while(1) {
       tok = strtok_r(str, ",", &sp);
       if(tok == NULL)
           break;
       var = atof(tok);

	   switch(i) {
		   case 0:
			   p->xlate.x = var;
			   break;
		   case 1:
			   p->xlate.y = var;
			   break;
		   case 2:
			   p->xlate.z = var;
			   break;
		   case 3:
			   p->rotate.r[0][0] = var;
			   break;
		   case 4:
			   p->rotate.r[1][0] = var;
			   break;
		   case 5:
			   p->rotate.r[2][0] = var;
			   break;
		   case 6:
			   p->rotate.r[0][1] = var;
			   break;
		   case 7:
			   p->rotate.r[1][1] = var;
			   break;
		   case 8:
			   p->rotate.r[2][1] = var;
			   break;
		   case 9:
			   p->rotate.r[0][2] = var;
			   break;
		   case 10:
			   p->rotate.r[1][2] = var;
			   break;
		   case 11:
			   p->rotate.r[2][2] = var;
			   break;
	   }
       str = NULL;
       i++;
   }
   free(str);
   printf("\n");
   return 0;
}

void initialize() {
    close(fdescriptor);
	if ((fdescriptor= open("/dev/i2c-0", O_RDWR)) < 0)   // check for that
       {
          // Open port for reading and writing
          fprintf(stderr, "Failed to open i2c bus\n");
         // exit(1);
       }
	if (ioctl(fdescriptor, I2C_SLAVE, ARDUINO_ADDRESS) < 0)
       {
          fprintf(stderr, "Arduino Not Present\n");
        //  exit(1);
       }
 usleep(1000);
}
/*******************************************************************/
/*          Aduino i2c ppm radio                                   */
/*******************************************************************/
void writeRadio()
{ 

	initialize();

	/********************************************************/

	uint8_t dat[2];

	dat[0]=1;dat[1]=channel[0];
	write(fdescriptor,dat,2);
	usleep(10000);

	dat[0]=2;dat[1]=channel[1];
	write(fdescriptor,dat,2);
	usleep(10000);

	dat[0]=3;dat[1]=channel[2];
	write(fdescriptor,dat,2);
	usleep(10000);

	dat[0]=4;dat[1]=channel[3];
	write(fdescriptor,dat,2);
	usleep(10000);

	dat[0]=5;dat[1]=channel[4];
	write(fdescriptor,dat,2);
	usleep(10000);

	dat[0]=6;dat[1]=channel[5];
	write(fdescriptor,dat,2);
	usleep(10000);
	
	dat[0]=0x08;
	write(fdescriptor,dat,1);
	usleep(10000);
	
    
	// printf("Servo1 %d, Servo2 %d, Servo3 %d, Servo4 %d \r",channel[0],channel[1],channel[2],channel[3]);
}
int read_count=0;
uint16_t readBytes(uint8_t reg)
{

//	char cmd[16];
//	cmd[0] = reg;
	uint16_t temp;
	uint8_t tmp1,tmp2;
	read_count++;
	if(read_count > 30){
	read_count=0;
	initialize();


		// As we are not talking to direct hardware but a microcontroller we
		// need to wait a short while so that it can respond.
		//
		// 1ms seems to be enough but it depends on what workload it has

		usleep(10000);

		uint8_t buf[2];
		if (read(fdescriptor, buf, 2) == 2) {
			tmp1 = (uint8_t) buf[0];
			tmp2 = (uint8_t) buf[1];
			temp=tmp2 <<8;
			temp |=tmp1;
			close(fdescriptor);
			return temp;
		}/*else   {
		   fprintf(stderr, "Arduino Erro de leitura\n");
		//  exit(1);
		}
		  */

	}
	return 65535;
}

int ComPass(void)
{
	
	hmc5883l_state_t state;
	calibration_t calib;
	FILE *cfg;
	struct stat stbuf;
	char *home = getenv("HOME");
	char *fcpath = strcat(home, "/");
	int ret=-1;
	fcpath = strcat(fcpath, CONFIG_FILE);
	diag((char *)"Initializing... please hold still.");
//	uint16_t tmp;
	   /* Set up the sensor: */
   
	state.cfg.avg = hmc5883l_config_t::AVG_8;
	   //state.cfg.gain = GAIN_1370;
	state.cfg.bias = hmc5883l_config_t::BIAS_NONE;
	state.cfg.gain = hmc5883l_config_t::GAIN_1090;
   	if((state.fd = hmc5883l_open()) < 0) return -1;
	if(hmc5883l_config(&state)) return -1;
	
	if(stat(fcpath, &stbuf) == 0) {
		diag((char *)"Precal file exists. Skipping calibration.");
 		cfg = fopen(fcpath, "r");
	   	ret=load_precal(&state, &calib, cfg);
	    fclose(cfg);
	}
	if(ret < 0) {
		cfg = fopen(fcpath, "w");
	    	 /* Get calibration data: */
		if(calibrate(&state, &calib, cfg)) {
	    	fclose(cfg);
	       	return -1;
		}
		fclose(cfg);
	}

	diag((char *)"Ready to read...");
   	diag((char *)"Center: %f %f %f", calib.xlate.x, calib.xlate.y, calib.xlate.z);
   	rot_dump(&(calib.rotate), NULL);
   	while(1) {
		if(hmc5883l_read(&state)) return -1;
		compass = heading(&(state.pos), &calib)*180.0/M_PI;
		usleep(100000);
//		diag("HDG: %f", 180.0*heading(&(state.pos), &calib)/M_PI);
  	}

	if(close(state.fd)) return sysdiag((char *)"close", (char *)"can't close I2C bus");
  	return 0;
}


void BaroMeter(Barometer &b)
{
	uint8_t i = 0;
	uint16_t rdut;
	uint16_t ReadUP; 	
	uint16_t Salt[10];
	uint16_t Malt=0;
	while(1) {
//		printf("B lock wait\n");
		fflush(stdout);
		pthread_mutex_lock(&i2c_lock);
//		printf("B lock\n");
		fflush(stdout);
		rdut = b.bmp085ReadUT();
		ReadUP= b.bmp085ReadUP();
		temperature = b.bmp085GetTemperature(rdut); //Get the temperature, bmp085ReadUT MUST be called first
		pressure = b.bmp085GetPressure(ReadUP);//Get the temperature
		usleep(10000);
		pthread_mutex_unlock(&i2c_lock);
//		printf("B unlock\n");
		fflush(stdout);
		Malt = b.calcAltitude(pressure); //Uncompensated caculation - in Meters
		atm = pressure / 101325;
		Salt[i]=Malt;
		i++;
		if(i==9){
			for(uint8_t j=0; j<10;j++){
					Malt += Salt[j];
				}
				alt=Malt/10;
				i=0;
		}
		usleep(100000);
	}  
} 

/*******************************************************************************************/

void sendPpm(){
	
	uint16_t tmp;
	while(1) {
//		printf("R lock wait\n");
//		fflush(stdout);
//		pthread_mutex_lock(&i2c_lock);
//		printf("R lock\n");
//		fflush(stdout);
		writeRadio();
		usleep(10000);
		tmp=readBytes(0x08);
		usleep(10000);
//		pthread_mutex_unlock(&i2c_lock);
//		printf("R unlock\n");
		fflush(stdout);
		if(tmp !=65535)
			vbatM=tmp;
		vbat=tmp;
		usleep(70000);
	}
}

#ifdef NRF24_TELE
bool handle_P(RF24NetworkHeader& header){

	long time;
	bool ok =false;
	ok=network.read(header,&time,sizeof(time));
	return ok;
}


/****************************************************************************
*         map the payload to use on rx
*****************************************************************************/
bool handle_R(RF24NetworkHeader& header){
	LINK =	network.read(header,&pl,sizeof(payload_t));
	return LINK;
}

void netWork(void)
{

 radio.begin();
 radio.setDataRate( RF24_250KBPS );
 network.begin(/*channel*/ 70, /*node address*/ this_node);
 radio.setRetries(7,7);
 delay(5);

 while(1){
 usleep(1000);
 
 if(!LINK){
 	pl.servo_1=1;
 	pl.servo_2=1;
 	pl.servo_3=1;
 	pl.servo_4=1;
 }
 
 if((!pl.triangle_bl && !pl.triangle_bh) || (pl.triangle_bl && pl.triangle_bh))channel[5]=0;
    if(!pl.triangle_bl && pl.triangle_bh)channel[5]=128;
    if(pl.triangle_bl && !pl.triangle_bh)channel[5]=255;    
    if((!pl.cross_bl && !pl.cross_bh) || (pl.cross_bl && pl.cross_bh))channel[4]=0;
    if(!pl.cross_bl && pl.cross_bh)channel[4]=128;
    if(pl.cross_bl && !pl.cross_bh)channel[4]=255; 
    channel[0]=pl.servo_1;
    channel[1]=pl.servo_2;
    channel[2]=pl.servo_3;
    channel[3]=pl.servo_4;    
 unsigned long now = millis();
  	if ( now - last_sent >= interval  )
  	{
    	last_sent = now;
    	fb.sysBat = vbat;
    	fb.motorBat = vbatM;
    	fb.tmp = temperature;
    	fb.alt = alt;
 	 	RF24NetworkHeader header(other_node,  'F'); // feed back
    	LINK = network.write(header,&fb,sizeof(fb));
 	}
        
  // Pump the network regularly
 	network.update();
    while ( network.available() )
  	{
    // If so, grab it and print it out
    RF24NetworkHeader header;
    network.peek(header);
        // Dispatch the message to the correct handler.
    switch (header.type)
    {
    case 'R': 
      handle_R(header); // get radio data
      break;
      case 'P':
      	handle_P(header);
      break;  
      default:
      	printf(("*** WARNING *** Unknown message type %c\n\r"),header.type);
      	network.read(header,0,0);
      break;
    };
    }
  }  
  

 }
#endif



#ifdef TCP_SOCKS
/********************************************************************
       
       Update data to telemetry
struct     
    uint8_t throtler;
	uint8_t yaw;
	uint8_t pitch;
	uint8_t roll;
	uint8_t aux1;
	uint8_t aux2;
	uint8_t flmode;
	uint16_t vbat;	
}Ptcp;
*******************************************************************/
void telemetry_update(void){ 
    	volt1=vbat * 5.0;
		volt1 /= 1024.0;   
		volt1 *=2.96;   
		volt1=0;     
		volt2=vbatM * 5.0;
		volt2 /= 1024.0;   
		volt2 *=2.96;
        sprintf (telemetry_buf, "%0.1f, %0.1f, %d, %d, %d, %d, %0.2f, %0.0f, %0.1f, %0.1f, %.7f, %.7f, %d, %d,  %d, %d, %d, %d, %0.2f, %0.2f, %0.2f\n", 
        temperature,
        alt, 
        Ptcp.throtler, 
        Ptcp.yaw, 
        Ptcp.pitch, 
        Ptcp.roll, 
        compass, 
        pressure/100, 
        volt1, 
        volt2,
        gpsLat,
  		gpsLon,
  		gpsAlt,
  		gpsF3d,
  		gpsGm,
  		gpsNm,
  		gpsNs,
  		gpsNb,
  		atYaw,
  		atRol,
  		atPit);
         
}
/*
	printf("LAT=%.7f , LON =%.7f, ALT=%d, F3D=%d, F2d=%d, GM=%d, NM=%d, NS=%d, NB=%d, Y=%0.2f, R=%0.2f, P=%0.2f \n",
  	gpsLat,
  	gpsLon,
  	gpsAlt,
  	gpsF3d,
  	gpsF2d,
  	gpsGm,
  	gpsNm,
  	gpsNs,
  	gpsNb,
  	y_p_r[0] * 180/M_PI,
  	y_p_r[1] * 180/M_PI,
  	y_p_r[2] * 180/M_PI);
printf("LAT= %.7f , LON = %.7f, ALT = %d, F3D = %d, F2d = %d, GM = %d, NM = %d, NS = %d, NB = %d \n",
  	gpsLat,
  	gpsLon,
  	gpsAlt,
  	gpsF3d,
  	gpsF2d,
  	gpsGm,
  	gpsNm,
  	gpsNs,
  	gpsNb);



*/
/*********************************************************
*struct payload_tcp{

	uint8_t throtler;
	uint8_t yaw;
	uint8_t pitch;
	uint8_t roll;
	uint8_t aux1;
	uint8_t aux2;
	uint8_t flmode;
	
}Ptcp;
*
*
**********************************************************/
void joystickUpdate(char *buf){
	Ptcp.throtler=buf[1];
	Ptcp.yaw=buf[2];
	Ptcp.pitch=buf[3];
	Ptcp.roll=buf[4];
	Ptcp.aux1=buf[5];
	Ptcp.flmode=buf[6];
	
	channel[0]=Ptcp.throtler;
	channel[1]=Ptcp.yaw;
	channel[2]=Ptcp.pitch;
	channel[3]=Ptcp.roll;
	channel[4]=Ptcp.aux1;
	channel[5]=Ptcp.flmode;
		
}

// processWP(buffer) from telemetry;  gpsUpdate(buffer); from gpsnav
/*
char gps_buff[2000];
double gpsLat,gpsLon;
int gpsAlt,gpsNb;
short gpsF3d,gpsF2d,gpsNm,gpsGm,gpsNs;

*/

void gpsUpdate(char *buffer){
char *ch;
char *buffe;
int i=0;
  buffe = static_cast<char*>(buffer) + 2;
  //printf("\nSplit \"%s\"\n", buffe);
  ch = strtok(buffe, ",");
  while (ch != NULL) {
  	
  	// printf("%s\r", ch);

  	switch (i){
  		case 0:
  			gpsLat=atof(ch);
  		break;
  		case 1:
  			gpsLon=atof(ch);
  		break;
  		case 2:
  			gpsAlt=atoi(ch);
  		break;
  		case 3:
  			gpsF3d=atoi(ch);
  		break;
  		case 4:
  			gpsF2d=atoi(ch);
  		break;
  		case 5:
  			gpsGm=atoi(ch);
  		break;
  		case 6:
  			gpsNm=atoi(ch);
  		break;
  		case 7:
  			gpsNs=atoi(ch);
  		break;
  		case 8:
  			gpsNb=atoi(ch);
  		break;
  		default:
  		break;
  			
  	};
  	ch = strtok(NULL, ",");
  	i++;
  	
  }

}
int tcp_server(void){
/**
    Handle multiple socket connections with select and fd_set on Linux
     
    Silver Moon ( m00n.silv3r@gmail.com)
*/
  

  

 
//	int gps_add;
    int opt = TRUE;
    int master_socket , addrlen , new_socket , client_socket[30] , max_clients = 30 , activity, i , valread , sd;
    int max_sd;
    struct sockaddr_in address;
      
    char buffer[4096];  //data buffer of 1K
      
    //set of socket descriptors
    fd_set readfds;
      
    //a message
    //char *message = "ECHO Daemon v1.0 \r\n";
    //char *message;
    //sprintf(message, "ECHO Daemon v1.0 \r\n");
  
    //initialise all client_socket[] to 0 so not checked
    for (i = 0; i < max_clients; i++)
    {
        client_socket[i] = 0;
    }
      
    //create a master socket
    if( (master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
  
    //set master socket to allow multiple connections , this is just a good habit, it will work without this
    if( setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0 )
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
  
    //type of socket created
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
      
    //bind the socket to localhost port 8888
    if (bind(master_socket, (struct sockaddr *)&address, sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    printf("Listener on port %d \n", PORT);
     
    //try to specify maximum of 3 pending connections for the master socket
    if (listen(master_socket, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
      
    //accept the incoming connection
    addrlen = sizeof(address);
    puts("Waiting for connections ...");
    char jbuff[3];
    while(TRUE)
    {
        //clear the socket set
        FD_ZERO(&readfds);
  
        //add master socket to set
        FD_SET(master_socket, &readfds);
        max_sd = master_socket;
         
        //add child sockets to set
        for ( i = 0 ; i < max_clients ; i++)
        {
            //socket descriptor
            sd = client_socket[i];
             
            //if valid socket descriptor then add to read list
            if(sd > 0)
                FD_SET( sd , &readfds);
             
            //highest file descriptor number, need it for the select function
            if(sd > max_sd)
                max_sd = sd;
        }
  
        //wait for an activity on one of the sockets , timeout is NULL , so wait indefinitely
        activity = select( max_sd + 1 , &readfds , NULL , NULL , NULL);
    
        if ((activity < 0) && (errno!=EINTR))
        {
            printf("select error");
        }
          
        //If something happened on the master socket , then its an incoming connection
        if (FD_ISSET(master_socket, &readfds))
        {
            if ((new_socket = accept(master_socket, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)
            {
                perror("accept");
                exit(EXIT_FAILURE);
            }
          
            //inform user of socket number - used in send and receive commands
            printf("New connection , socket fd is %d , ip is : %s , port : %d \n" , new_socket , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));
            //sprintf(message,"New connection , socket fd is %d , ip is : %s , port : %d \n" , new_socket , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));
        
            //send new connection greeting message
       //     if( send(new_socket, message, strlen(message), 0) != strlen(message) )
       //     {
       //         perror("send");
       //     }
              
       //     puts("Welcome message sent successfully");
              
            //add new socket to array of sockets
            for (i = 0; i < max_clients; i++)
            {
                //if position is empty
                if( client_socket[i] == 0 )
                {
                    client_socket[i] = new_socket;
                    printf("Adding to list of sockets as %d\n" , i);
                     
                    break;
                }
            }
        }
          
        //else its some IO operation on some other socket :)
        for (i = 0; i < max_clients; i++)
        {
            sd = client_socket[i];
              
            if (FD_ISSET( sd , &readfds))
            {
                //Check if it was for closing , and also read the incoming message
                if ((valread = read( sd , buffer, 1024)) == 0)
                {
                    //Somebody disconnected , get his details and print
                    getpeername(sd , (struct sockaddr*)&address , (socklen_t*)&addrlen);
                    printf("Host disconnected , ip %s , port %d \n" , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));
                      
                    //Close the socket and mark as 0 in list for reuse
                    close( sd );
                    client_socket[i] = 0;
                }
                  
                //Echo back the message that came in
                else
                {
                	if(buffer[0]=='T'){
                	//	printf("Enviar telemetria para %d Cliente N = %d\n",ntohs(address.sin_port),i);
                		send(sd , telemetry_buf , strlen(telemetry_buf) , 0 );
                    //set the string terminating NULL byte on the end of the data read
                    	if(buffer[1]=='1'){
                    		WP_FLAG=true;
                    		sprintf(wp_buff,"%s",  buffer);
                    		wp_buff[0]='W';
                    		wp_buff[1]='1';
                    		fflush(stdout);
                    		//processWP(buffer); // processWP(buffer) from telemetry;  gpsUpdate(buffer); from gpsnav
                    		printf("*******************************\n");
                    		printf("*******************************\n");
                    		printf("*******************************\n");
                    		printf("*******************************\n");
                    	}
                    		else
                    		WP_FLAG=false;
                    	
                    }else{
                    		if(buffer[0]=='J'){
                    			
                    			jbuff[0]='J';
                    			jbuff[1]='\0';
                			//	printf("Enviar telemetria para %d Cliente N = %d\n",ntohs(address.sin_port),i);
                			//	if(buffer[7]==1)
                					joystickUpdate(buffer);
                				send(sd , jbuff , 2 , 0 );
                			}else{
                					if(buffer[0]=='G'){
                						if(!WP_FLAG){
                							gpsUpdate(buffer);
                							jbuff[0]='G';
                							jbuff[1]='\0';
                							send(sd , jbuff , 2 , 0 );
                						}else{
                								send(sd,wp_buff,strlen(wp_buff),0);
                								WP_FLAG=false;
                							  }
                					}else
                						{
                    				//	buffer[valread] = '\0';
                    				//	send(sd , buffer , strlen(buffer) , 0 );
                    				// }
                    					}
                    						if(buffer[0]=='W' && buffer[1]=='1'){  // receiv waypoint 
                    							send(sd , buffer , strlen(buffer) , 0 );
                    							wp_buff[0]=0;
                    							sprintf(wp_buff,"%s",buffer);
                    							WP_FLAG=true;
                    				//send(gps_add , buffer , strlen(buffer) , 0 );
                    			
                    						}
                    		}
                }
            }
        }
    }
   }   
    return 0;
} 




#endif

void outPut(){
while(1){
     
	printf ("\n%0.1f, %0.1f, %d, %d, %d, %d, %0.0f, %0.0f, %0.1f, %0.1f \n", temperature,alt, Ptcp.throtler, Ptcp.yaw, Ptcp.pitch, Ptcp.roll, compass, pressure/100, volt1, volt2 );
	printf("LAT=%.7f , LON =%.7f, ALT=%d, F3D=%d, F2d=%d, GM=%d, NM=%d, NS=%d, NB=%d, Y=%0.2f, R=%0.2f, P=%0.2f \n",
  	gpsLat,
  	gpsLon,
  	gpsAlt,
  	gpsF3d,
  	gpsF2d,
  	gpsGm,
  	gpsNm,
  	gpsNs,
  	gpsNb,
  	atYaw, 
  	atRol,
  	atPit);

//    printf("ypr  %7.2f %7.2f %7.2f    \n", y_p_r[0] * 180/M_PI, y_p_r[1] * 180/M_PI, y_p_r[2] * 180/M_PI);
	printf("\nTemperature\t%0.2fC \n", temperature/*,0x00B0*/);
	printf("Pressure\t%0.2fhPa \n", pressure/100);
//	printf("Pressure Compensated \t%0.2fhPa \n", ((double)pressZ)/100); 
	printf("Altitude\t%0.2fm\n", alt); 
	printf("Compass\t\t%0.2fdeg\n", compass); 
	// sleep(10);
	 telemetry_update();
	 usleep(100000);
	 }
}
void GiroAcc(void){
	setup_giro_acc();
    usleep(100000);
int i=0;    
    for (;;){
    	usleep(100000);
    	if(powerOn)
    	i++;	
        giro_loop(y_p_r);
        atYaw=y_p_r[0]* 180/M_PI - gCalib;
        atRol=y_p_r[1]* 180/M_PI-4;
        atPit=y_p_r[2]* 180/M_PI*(-1);
        if(i==200 && powerOn){gCalib=atYaw; powerOn = false;}
        	
        
	//	printf("ypr  %7.2f %7.2f %7.2f    ", y_p_r[0] * 180/M_PI, y_p_r[1] * 180/M_PI, y_p_r[2] * 180/M_PI);
}
}

int main(void){
	i2c_io i2c;
	if (pthread_mutex_init(&i2c_lock, NULL) != 0)
	{
		printf("\n mutex init failed\n");
		exit(1);
	}   
#ifdef COMPASS
	std::thread t1 = std::thread(ComPass);
#endif
#ifdef TCP_SERVER
	std::thread t2 = std::thread(tcp_server);
#endif
#ifdef RADIO
	std::thread t3 = std::thread(sendPpm);
#endif
#ifdef OUTPUT
	std::thread t4 = std::thread(outPut);
#endif
#ifdef BAROMETER
	Barometer barometer = Barometer();
	std::thread t5 = std::thread(BaroMeter, barometer);
#endif
#ifdef GIRO_ACC

	std::thread t6 = std::thread(GiroAcc);
#endif 


#ifdef COMPASS
	t1.join();
#endif
#ifdef TCP_SERVER
	t2.join();
#endif
#ifdef RADIO
	t3.join();
#endif
#ifdef OUTPUT
	t4.join();
#endif
#ifdef BAROMETER
	t5.join();
#endif
#ifdef GIRO_ACC
	t6.join();
#endif
//int a=tcp_server();
}



/*************************************************************
*     END of Main loop
*
*
************************************************************/



