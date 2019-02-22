/* This is free software. See COPYING for details. */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "diag.h"
#include "debug.h"
#include "hmc5883l.h"
#include "rotate.h"
#include "calibrate.h"
#include "heading.h"

/*** heading() -- transform magnetometer reading into compass heading

Takes a reading from a three-axis magnetometer and transforms it into
a scalar compass heading using calibration data obtained previously.

The first argument is a pointer to a structure representing the raw
magnetometer reading.

The second argument is a pointer to a structure containing the calibration
data previously obtained using calibrate().

Returns a heading (in radians to the right of geographic north). Multiply
by 180.0/M_PI to get degrees.
***/
float heading(const hmc5883l_pos_t * const pos,
 const calibration_t * const calib) {
   hmc5883l_pos_t p;
   float hdg;

   TSTA(!pos); TSTA(!calib);

   memcpy(&p, pos, sizeof(p));
   p.x -= calib->xlate.x; p.y -= calib->xlate.y; p.z -= calib->xlate.z;
   rot_posn(&p, &(calib->rotate));

   hdg = atan(p.x/p.y);
   if(p.y < 0.0) hdg += M_PI;
   if(p.y > 0.0 && p.x < 0.0) hdg += M_PI * 2.0;

//   printf("pos=% 4.3f % 4.3f % 4.3f hdg=%f        \n",
//		   p.x, p.y, p.z, hdg*180.0/M_PI);

   return hdg;
}
