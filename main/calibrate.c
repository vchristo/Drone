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

typedef struct { hmc5883l_pos_t n, w, c; } nwc_t;

/*** sqdist() -- get square of distance between two points

Given two points in three-space, finds the square of the distance between
them. The arguments are pointers to hmc5883l_pos_t structures representing
the points in question.

Returns the distance between the two points.
***/
__attribute__((pure)) static float sqdist(const hmc5883l_pos_t * const a,
 const hmc5883l_pos_t * const b) {
   TSTA(!a); TSTA(!b);

   return (a->x - b->x)*(a->x - b->x) +
          (a->y - b->y)*(a->y - b->y) +
          (a->z - b->z)*(a->z - b->z);
}

/*** jitter() -- estimate noise level when sensor is stationary

This function should be called when the sensor is not moving. It
takes a large number of readings to establish an average position, then
takes another large number of readings and finds the value furthest
away from the average.

On success, returns the square of the distance between the average and
most-extreme values. On error, returns a negative value.
***/
#define NREADINGS_AVG 100
#define NREADINGS_MAX 100

static float jitter(hmc5883l_state_t * const p, hmc5883l_pos_t * const avg) {
   double max, x;
   int i;

   TSTA(!p); TSTA(p->fd < 0); TSTA(!avg);

   diag((char *)"Finding average reading...");
   avg->x = avg->y = avg->z = 0.0;
   for(i = 0; i < NREADINGS_AVG; i++) {
      if(hmc5883l_read(p)) return -1.0;
      avg->x += p->pos.x; avg->y += p->pos.y; avg->z += p->pos.z;
   }
   avg->x *= (1.0/NREADINGS_AVG);
   avg->y *= (1.0/NREADINGS_AVG);
   avg->z *= (1.0/NREADINGS_AVG);
   diag((char *)"OK, avg = %f, %f, %f", avg->x, avg->y, avg->z);

   diag((char *)"Finding jitter magnitude...");
   max = 0.0;
   for(i = 0; i < NREADINGS_MAX; i++) {
      if(hmc5883l_read(p)) return -1.0;
      x = sqdist(avg, &(p->pos));
      if(x > max) max = x;
   }

   diag((char *)"max square of error distance: %g", max);
   return max;
}

/*** get_nwc() -- obtain points N, W and C used as basis for calibration

This function obtains the points N, W and C used as the basis for creating
the calibration matrix.

Point N is the sensor reading when the vehicle is facing (geographic)
North.

Point C is the center of the ellipse created by the sensor readings when
the vehicle undergoes a complete rotation about the yaw axis.

Point W is 1) somewhere on the aforementioned ellipse, 2) not colinear with
N and S and 3) somewhere on the Western half of the compass rose.

The first argument is a pointer to the sensor state, used to take
readings.

The second argument is a pointer to an nwc_t structure which will be
populated with the results.

Returns 0 on success, non-0 on error.
***/
static int get_nwc(hmc5883l_state_t * const state, nwc_t * const p) {
   float err, dist_N, dist_NS, dist_S, diff, best;
   hmc5883l_pos_t pos_S, pos_prev;
   int n;

   /* See how much jitter there is when we're sitting still: */
   if((err = jitter(state, &(p->n))) < -0.1) return -1;

   /* Let N and S start as the average from the jitter calculation: */
   memcpy(&pos_S, &(p->n), sizeof(pos_S));
   memcpy(&pos_prev, &(p->n), sizeof(pos_prev));
   p->c.x = p->c.y = p->c.z = 0.0;
   n = 0;
   dist_NS = 0.0; /* since N and S are the same, the distance is zero */
   best = 0.0;

   diag((char *)"N= %lf %lf %lf", p->n.x, p->n.y, p->n.z);

   diag((char *)"OK, now turn slowly clockwise...");
	do{
      if(hmc5883l_read(state)) return -1; /* take a reading */
      //printf("%lf %lf %lf\n", state->pos.x, state->pos.y, state->pos.z);

      /* Find the (squares of the) distances from the point just read
         to the current N and S: */
      dist_N = sqdist(&(state->pos), &(p->n));
      dist_S = sqdist(&(state->pos), &pos_S);

      /* If the current point is more than a fixed distance away from
         pos_prev, add it to the points averaged to find the center and
         let it be the new pos_prev: */
      if(sqdist(&(state->pos), &pos_prev) > err * 80.0) {
         p->c.x+=state->pos.x; p->c.y+=state->pos.y; p->c.z+=state->pos.z;
         n++;
         memcpy(&pos_prev, &(state->pos), sizeof(pos_prev));
         diag((char *)"Ctr mark: %lf %lf %lf\n", pos_prev.x, pos_prev.y, pos_prev.z);
      }

      /* If the point just read is further away from N that the current S
         by more than a tiny amount (compared to the distance from the
         current point to S), then the current point is our new S.

         The complexity is required because the points are on an elliptic
         curve. If N is near the minor axis of a highly-eccentric ellipse,
         then there are two points at a maximum distance from N, and we
         want to find the first one. Because of jitter, the second one may
         look (slightly) further away... */
      if(dist_N - dist_NS > dist_S * 0.1) {
         memcpy(&pos_S, &(state->pos), sizeof(pos_S));
         best = dist_NS = dist_N; dist_S = 0.0;
         diag((char *)"S (%g): %lf %lf %lf", dist_NS, pos_S.x, pos_S.y, pos_S.z);
      }

      diff = fabsf(dist_N - dist_S);
      if(diff < best) {
         best = diff;
         memcpy(&(p->w), &(state->pos), sizeof(p->w));
         diag((char *)"W (%g): %lf %lf %lf", diff, p->w.x, p->w.y, p->w.z);
      }

   /* End the calibration loop when the most recent reading is close to
      the original position and the distance from N to S is big. */
      //printf(" %f > %f || %f < %f\n", dist_N, err*2.0, dist_NS, err*1000.0);
      usleep(10000);
	} while(dist_N > err * 2.0 || dist_NS < err * 1000.0);
   p->c.x /= n; p->c.y /= n; p->c.z /= n;
 //  p->c.x = 1; p->c.y = 2; p->c.z = n;
   diag((char *)"Center (%d): %lf %lf %lf", n, p->c.x, p->c.y, p->c.z);

   return 0;
}

/*** calibrate() -- find translation and rotation data for magnetometer

Performs a calibration process to find a translation vector and rotation
matrix such that any magnetometer reading can be translated and rotated
so that it will lie on a circle in the XY plane, centered at the origin,
with North on the positive Y axis and West on the negative X axis.

The first argument is a pointer to the magnetometer state used to take
readings.

The second argument is a pointer to a calibration_t vector to be populated
with the calibration data.

Returns 0 on success, non-0 on error.

Note that the calibration process assumes:

1) The user will begin the process facing exactly geographic North.

2) When instructed, the user will turn in a full circle to the right.
   (To be specific: The user will make a full rotation about the yaw
   axis. The path need not be a circle, and neither the speed nor the
   turn rate need be constant.)
***/
int calibrate(hmc5883l_state_t * const state, calibration_t * const p, FILE *cfg) {
   nwc_t nwc;
   rotation_t rz, rx, ry;

   if(get_nwc(state, &nwc))
     return 0;

   /* We know the translation immediately; it's just the center point. */
   memcpy(&(p->xlate), &(nwc.c), sizeof(nwc.c));
   diag_cfg(cfg, (char *)"%lf,%lf,%lf,", p->xlate.x, p->xlate.y, p->xlate.z);

   /* Translate N and W so that C is the new origin: */
   nwc.n.x -= nwc.c.x; nwc.n.y -= nwc.c.y; nwc.n.z -= nwc.c.z;
   nwc.w.x -= nwc.c.x; nwc.w.y -= nwc.c.y; nwc.w.z -= nwc.c.z;

   /* FIXME: If N is initially very close to the Z axis, the following
      is broken. To fix: Do rotation about X axis first, then about Z. */

   /* Find Rz to put N above/below the positive Y axis: */
   if(rot_find_Rz(&(nwc.n), &rz)) return -1;
   rot_dump(&rz, "z");
   rot_posn(&(nwc.n), &rz);
   rot_posn(&(nwc.w), &rz);
   diag((char *)"N1=%4.3f %4.3f %4.3f; W1=%4.3f %4.3f %4.3f",
    nwc.n.x, nwc.n.y, nwc.n.z, nwc.w.x, nwc.w.y, nwc.w.z);

   /* Find Rx to put N on the positive Y axis: */
   if(rot_find_Rx(&(nwc.n), &rx)) return -1;
   rot_dump(&rx, "x");
   rot_posn(&(nwc.n), &rx);
   rot_posn(&(nwc.w), &rx);
   diag((char *)"N2=%4.3f %4.3f %4.3f; W2=%4.3f %4.3f %4.3f",
    nwc.n.x, nwc.n.y, nwc.n.z, nwc.w.x, nwc.w.y, nwc.w.z);

   /* Find Ry to put W on the negative-X half of the XY plane. */
   if(rot_find_Ry(&(nwc.w), &ry)) return -1;
   rot_dump(&ry, "y");
   rot_posn(&(nwc.n), &ry);
   rot_posn(&(nwc.w), &ry);
   diag((char *)"N3=%4.3f %4.3f %4.3f; W3=%4.3f %4.3f %4.3f",
    nwc.n.x, nwc.n.y, nwc.n.z, nwc.w.x, nwc.w.y, nwc.w.z);

   /* Compose the three rotations into a single matrix. Order matters. */
   memcpy(&(p->rotate), &ry, sizeof(rx));
   rot_mult(&(p->rotate), &rx);
   rot_mult(&(p->rotate), &rz);
   rot_dump(&(p->rotate), NULL);
   rot_cfg_dump(&(p->rotate), NULL, cfg);
   return 0;
}
