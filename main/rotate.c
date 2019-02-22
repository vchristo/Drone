/* This is free software. See COPYING for details. */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "diag.h"
#include "debug.h"
#include "rotate.h"

/*** rot_dump() -- dump a rotation matrix in human-readable form

Writes an ASCII-graphic representation of a rotation matrix to the
diagnostic output in (hopefully) human-readable form.

The first argument is a pointer to a rotation_t data structure representing
the matrix in question.

The second argument is a pointer to a nul-terminated string containing
a text label for the matrix, or NULL if no label is desired.

Returns 0 on success, non-0 on error.
***/
int rot_dump(const rotation_t * const rot, const char * const lbl) {
   int len=lbl?strlen(lbl):0;

   diag((char *)" %*s   |% 4.3f % 4.3f % 4.3f|", len, "", rot->r[0][0],
    rot->r[1][0], rot->r[2][0]);
   diag((char *)"R%s = |% 4.3f % 4.3f % 4.3f|", lbl?lbl:"", rot->r[0][1],
    rot->r[1][1], rot->r[2][1]);
   diag((char *)" %*s   |% 4.3f % 4.3f % 4.3f|", len, "", rot->r[0][2],
    rot->r[1][2], rot->r[2][2]);
   return 0;
}

int rot_cfg_dump(const rotation_t * const rot, const char * const lbl, FILE *f) {
	char str[1024];
	snprintf(str,sizeof(str),"%f,%f,%f,%f,%f,%f,%f,%f,%f",rot->r[0][0],rot->r[1][0],rot->r[2][0],rot->r[0][1],rot->r[1][1],rot->r[2][1],rot->r[0][2],rot->r[1][2],rot->r[2][2]);
	fwrite(str, strlen(str), 1, f);
	return 0;
}

/*** rot_find_Rz() -- find rotation about Z that puts point above/below +Y axis

Finds a matrix of rotation about the Z axis such that a specified point
will be placed somewhere directly above or below the positive Y axis. (In
other words, the {X,Y,Z} coordinates of the point after rotation will be
{0, >0, Z}.

The first argument is a pointer to a hmc5883l_pos_t representing the point
in question, which must not be on (nor very near) the Z axis.

The second argument is a pointer to a rotation_t structure into which the
result will be written.
***/
int rot_find_Rz(const hmc5883l_pos_t * const pos, rotation_t * const rot) {
   float theta;
   int i, j;

   theta = -atan(-pos->x / pos->y);
   if(pos->x*sin(theta) + pos->y*cos(theta) < 0) theta += M_PI;

   for(i = 0; i < 3; i++) for(j = 0; j < 3; j++) rot->r[i][j] = 0.0;
   rot->r[2][2] = 1.0;
   rot->r[0][0] = cos(theta); rot->r[1][1]=rot->r[0][0];
   rot->r[0][1] = sin(theta); rot->r[1][0]=-rot->r[0][1];
   return 0;
}

/*** rot_find_Rx() -- find rotation about X that puts a point on +Y axis

Finds a matrix of rotation about the X axis such that a specified point
will be placed directly on the positive Y axis. (In other words, the {X,Y,Z}
coordinates of the point after rotation will be {0, >0, 0}.)

The first argument is a pointer to a hmc5883l_pos_t representing the point
in question, which must be directly above, below or on the +Y axis.

The second argument is a pointer to a rotation_t structure into which the
result will be written.
***/
int rot_find_Rx(const hmc5883l_pos_t * const pos, rotation_t * const rot) {
   float theta;
   int i, j;

   theta = -atan(pos->z / pos->y);
   if(pos->y*cos(theta) - pos->z*sin(theta) < 0) theta += M_PI;

   for(i = 0; i < 3; i++) for(j = 0; j < 3; j++) rot->r[i][j] = 0.0;
   rot->r[0][0] = 1.0;
   rot->r[1][1] = cos(theta); rot->r[2][2]=rot->r[1][1];
   rot->r[1][2] = sin(theta); rot->r[2][1]=-rot->r[1][2];
   return 0;
}

/*** rot_find_Ry() -- find rotation about Y that puts a point on XY plane

Finds a matrix of rotation about the Y axis such that a specified point
will be placed somewhere on the -X half of the XY plane. (In other words,
the {X,Y,Z} coordinates of the point after rotation will be {<0, Y, 0}.)

The first argument is a pointer to a hmc5883l_pos_t representing the point
in question.

The second argument is a pointer to a rotation_t structure into which the
result will be written.
***/
int rot_find_Ry(const hmc5883l_pos_t * const pos, rotation_t * const rot) {
   float theta;
   int i, j;

   theta = -atan(-pos->z / pos->x);
   if(pos->x*cos(theta) + pos->z*sin(theta) > 0) theta += M_PI;

   for(i = 0; i < 3; i++) for(j = 0; j < 3; j++) rot->r[i][j] = 0.0;
   rot->r[1][1] = 1.0;
   rot->r[0][0] = cos(theta); rot->r[2][2]=rot->r[0][0];
   rot->r[2][0] = sin(theta); rot->r[0][2]=-rot->r[2][0];
   return 0;
}

/*** rot_posn() -- rotate a single point using a rotation matrix

Rotates a point (specified by the hmc5883l_pos_t pointed to by the first
argument) using a rotation matrix (pointed to by the second argument).

The point is rotated in-place -- the result is placed back in the pos_t
pointed to by the second argument, overwriting the original data.
***/
void rot_posn(hmc5883l_pos_t * const pos, const rotation_t * const rot) {
   float x, y, z;

   x=rot->r[0][0] * pos->x + rot->r[1][0] * pos->y + rot->r[2][0] * pos->z;
   y=rot->r[0][1] * pos->x + rot->r[1][1] * pos->y + rot->r[2][1] * pos->z;
   z=rot->r[0][2] * pos->x + rot->r[1][2] * pos->y + rot->r[2][2] * pos->z;

   pos->x=x; pos->y=y; pos->z=z;
}

/*** rot_mult() -- multiply rotation matrix A by B, leaving result in A

Performs the matrix multiplication AB (where A and B are 3x3 elemental
rotation matrices), then copies the result into A, overwriting the original
contents.

The arguments are pointers to the rotation matrices in question.
***/
void rot_mult(rotation_t * const a, const rotation_t * const b) {
   rotation_t r;
   int i, j;

   for(i = 0; i < 3; i++) {
      for(j = 0; j < 3; j++) {
         r.r[i][j] = a->r[0][j]*b->r[i][0] +
                     a->r[1][j]*b->r[i][1] +
                     a->r[2][j]*b->r[i][2];
      }
   }

   memcpy(a, &r, sizeof(r));
}
