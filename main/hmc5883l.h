#ifndef HMC5883L_H
#define HMC5883L_H

typedef struct { float x, y, z; } hmc5883l_pos_t;

typedef struct {
  /* Number of samples averaged per reading: */
  enum avg {AVG_1=0, AVG_2=1, AVG_4=2, AVG_8=3} avg;

  /* Load bias direction: */
  enum bias {BIAS_NONE=0, BIAS_POS=1, BIAS_NEG=2} bias;

  /* Gain (in LSb per Gauss): */
  enum gain {GAIN_1370=0, GAIN_1090=1, GAIN_820=2, GAIN_660=3, GAIN_440=4,
   GAIN_390=5, GAIN_330=6, GAIN_230=7} gain;
} hmc5883l_config_t;

typedef struct {
   hmc5883l_config_t cfg;
   hmc5883l_pos_t pos;
   int fd;
} hmc5883l_state_t;

int hmc5883l_open(void);
int hmc5883l_read(hmc5883l_state_t * const p);
int hmc5883l_config(hmc5883l_state_t * const p);

#endif /* ifndef HMC5883L_H */
