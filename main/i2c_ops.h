#ifndef I2C_OPS_H
#define I2C_OPS_H

int i2c_find(const uint8_t addr, const char * const devname,
 int(*callback)(const int fd));

#endif /* ifndef I2C_OPS_H */
