/* This is free software. See COPYING for details. */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "diag.h"
#include "debug.h"
#include "hmc5883l.h"
#include "calibrate.h"
#include "heading.h"

#define CONFIG_FILE ".precal.txt"

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
   printf(str);
   printf(",");
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

int main(void) {
   hmc5883l_state_t state;
   calibration_t calib;
   FILE *cfg;
   struct stat stbuf;
   char *home = getenv("HOME");
   char *fcpath = strcat(home, "/");
   int ret=-1;
   fcpath = strcat(fcpath, CONFIG_FILE);

   diag("Initializing... please hold still.");

   /* Set up the sensor: */
   state.cfg.avg = AVG_8;
   //state.cfg.gain = GAIN_1370;
   state.cfg.bias = BIAS_NONE;
   state.cfg.gain = GAIN_1090;
   if((state.fd = hmc5883l_open()) < 0) return -1;
   if(hmc5883l_config(&state)) return -1;

   if(stat(fcpath, &stbuf) == 0) {
     diag("Precal file exists. Skipping calibration.");
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

   diag("Ready to read...");
   diag("Center: %f %f %f", calib.xlate.x, calib.xlate.y, calib.xlate.z);
   rot_dump(&(calib.rotate), NULL);

   while(1) {
      if(hmc5883l_read(&state)) return -1;
      heading(&(state.pos), &calib);
     // diag("HDG: %f", 180.0*heading(&(state.pos), &calib)/M_PI);
   }

   if(close(state.fd)) return sysdiag("close", "can't close I2C bus");
   return 0;
}
