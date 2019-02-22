#ifndef HEADING_H
#define HEADING_H

#include "hmc5883l.h"
#include "calibrate.h"

float heading(const hmc5883l_pos_t * const pos,
 const calibration_t * const calib);

#endif /* ifndef HEADING_H */
