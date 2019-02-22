#ifndef ROTATE_H
#define ROTATE_H

#include "hmc5883l.h"

typedef struct { float r[3][3]; } rotation_t;

int rot_dump(const rotation_t * const rot, const char * const lbl);
int rot_cfg_dump(const rotation_t * const rot, const char * const lbl, FILE *f);

int rot_find_Rz(const hmc5883l_pos_t * const pos, rotation_t * const rot);
int rot_find_Rx(const hmc5883l_pos_t * const pos, rotation_t * const rot);
int rot_find_Ry(const hmc5883l_pos_t * const pos, rotation_t * const rot);

void rot_posn(hmc5883l_pos_t * const pos, const rotation_t * const rot);

void rot_mult(rotation_t * const a, const rotation_t * const b);

#endif /* ifndef ROTATE_H */
