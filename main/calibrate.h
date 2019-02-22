#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "hmc5883l.h"
#include "rotate.h"

typedef struct {
   hmc5883l_pos_t xlate;
   rotation_t rotate;
} calibration_t;

int calibrate(hmc5883l_state_t * const state, calibration_t * const p, FILE *cfg);

#endif /* ifndef CALIBRATE_H */
