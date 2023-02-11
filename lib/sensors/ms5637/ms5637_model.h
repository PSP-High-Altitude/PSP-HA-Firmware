#ifndef MS5637_MODEL_H
#define MS5637_MODEL_H

#include <stdint.h>

#include "i2c/i2c.h"
#include "status.h"

#define MS5637_I2C_ADDR (uint8_t)0b1110110

Status ms5637_model_i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len);

Status ms5637_model_i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len);

Status ms5637_model_set_state(uint32_t d1, uint32_t d2, uint16_t prom[]);

#endif  // MS5637_MODEL_H
