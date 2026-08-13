#ifndef _AS5600_H_
#define _AS5600_H_

#include "gd32f30x.h"
#include "i2c0.h"
#include <math.h>

/* AS5600 I2C address */
#define AS5600_I2C_7BIT_ADDRESS     0x36
#define AS5600_I2C_ADDRESS         AS5600_I2C_7BIT_ADDRESS << 1  /* I2C address */

/* AS5600 register addresses */
#define AS5600_REG_ZMCO             0x00  /* ZMCO */
#define AS5600_REG_ZPOS_H           0x01  /* ZPOS high byte */
#define AS5600_REG_ZPOS_L           0x02  /* ZPOS low byte */
#define AS5600_REG_MPOS_H           0x03  /* MPOS high byte */
#define AS5600_REG_MPOS_L           0x04  /* MPOS low byte */
#define AS5600_REG_MANG_H           0x05  /* MANG high byte */
#define AS5600_REG_MANG_L           0x06  /* MANG low byte */
#define AS5600_REG_CONF_H           0x07  /* CONFIG high byte */
#define AS5600_REG_CONF_L           0x08  /* CONFIG low byte */
#define AS5600_REG_RAW_ANGLE_H      0x0C  /* RAW ANGLE high byte */
#define AS5600_REG_RAW_ANGLE_L      0x0D  /* RAW ANGLE low byte */
#define AS5600_REG_ANGLE_H          0x0E  /* ANGLE high byte */
#define AS5600_REG_ANGLE_L          0x0F  /* ANGLE low byte */
#define AS5600_REG_STATUS           0x0B  /* STATUS */
#define AS5600_REG_AGC              0x1A  /* AGC */
#define AS5600_REG_MAGNITUDE_H      0x1B  /* MAGNITUDE high byte */
#define AS5600_REG_MAGNITUDE_L      0x1C  /* MAGNITUDE low byte */
#define AS5600_REG_BURN             0xFF  /* BURN commands */

/* Status register bits */
#define AS5600_STATUS_MH            0x20  /* Magnet too strong */
#define AS5600_STATUS_ML            0x10  /* Magnet too weak */
#define AS5600_STATUS_MD            0x08  /* Magnet detected */

/* Configuration settings */
#define AS5600_POWER_MODE_NORM      0x00  /* Normal power mode */
#define AS5600_POWER_MODE_LPM1      0x01  /* Low power mode 1 */
#define AS5600_POWER_MODE_LPM2      0x02  /* Low power mode 2 */
#define AS5600_POWER_MODE_LPM3      0x03  /* Low power mode 3 */

#define AS5600_HYSTERESIS_OFF       0x00  /* No hysteresis */
#define AS5600_HYSTERESIS_1LSB      0x01  /* 1 LSB hysteresis */
#define AS5600_HYSTERESIS_2LSB      0x02  /* 2 LSB hysteresis */
#define AS5600_HYSTERESIS_3LSB      0x03  /* 3 LSB hysteresis */

#define AS5600_OUTPUT_STAGE_ANALOG  0x00  /* Analog output */
#define AS5600_OUTPUT_STAGE_PWM     0x01  /* PWM output */

#define AS5600_PWM_FREQ_115HZ       0x00  /* 115Hz PWM frequency */
#define AS5600_PWM_FREQ_230HZ       0x01  /* 230Hz PWM frequency */
#define AS5600_PWM_FREQ_460HZ       0x02  /* 460Hz PWM frequency */
#define AS5600_PWM_FREQ_920HZ       0x03  /* 920Hz PWM frequency */

#define AS5600_SLOW_FILTER_16X      0x00  /* 16x filter */
#define AS5600_SLOW_FILTER_8X       0x01  /* 8x filter */
#define AS5600_SLOW_FILTER_4X       0x02  /* 4x filter */
#define AS5600_SLOW_FILTER_2X       0x03  /* 2x filter */

#define AS5600_FAST_FILTER_SLOW     0x00  /* Slow filter only */
#define AS5600_FAST_FILTER_6LSB     0x01  /* 6 LSB fast filter threshold */
#define AS5600_FAST_FILTER_7LSB     0x02  /* 7 LSB fast filter threshold */
#define AS5600_FAST_FILTER_9LSB     0x03  /* 9 LSB fast filter threshold */
#define AS5600_FAST_FILTER_18LSB    0x04  /* 18 LSB fast filter threshold */
#define AS5600_FAST_FILTER_21LSB    0x05  /* 21 LSB fast filter threshold */
#define AS5600_FAST_FILTER_24LSB    0x06  /* 24 LSB fast filter threshold */
#define AS5600_FAST_FILTER_10LSB    0x07  /* 10 LSB fast filter threshold */

/* Angle calculation constants */
#define AS5600_MAX_ANGLE             4096  /* 12-bit resolution */
#define AS5600_ANGLE_TO_RAD          (2.0f * 3.1415926535f / 4096.0f)

/* AS5600 data structure */
typedef struct {
    uint16_t raw_angle;      /* Raw angle value (0-4095) */
    uint16_t angle;          /* Filtered angle value (0-4095) */
    uint16_t magnitude;      /* Magnet strength (0-4095) */
    uint8_t status;          /* Status register value */
    uint8_t agc;             /* Automatic Gain Control value */
    float angle_rad;         /* Angle in radians (0-2pi) */
} as5600_data_t;

/* Magnet status */
typedef enum {
    AS5600_MAGNET_OK = 0,
    AS5600_MAGNET_TOO_STRONG,
    AS5600_MAGNET_TOO_WEAK,
    AS5600_MAGNET_NOT_DETECTED
} as5600_magnet_status_t;

/* Function prototypes */
void AS5600_Init(void);
as5600_magnet_status_t AS5600_CheckMagnet(void);
i2c_status_t AS5600_ReadRawAngle(uint16_t *angle);
i2c_status_t AS5600_ReadAngle(uint16_t *angle);
uint8_t AS5600_ReadAngleOk(uint16_t *angle);
uint8_t AS5600_ReadAngleRadOk(float *angle_rad);
i2c_status_t AS5600_ReadMagnitude(uint16_t *magnitude);
i2c_status_t AS5600_ReadStatus(uint8_t *status);
i2c_status_t AS5600_ReadAGC(uint8_t *agc);
i2c_status_t AS5600_ReadAll(as5600_data_t *data);
float AS5600_RawAngleToRad(uint16_t angle);

/* Configuration functions */
i2c_status_t AS5600_SetStartPosition(uint16_t start_pos);
i2c_status_t AS5600_SetStopPosition(uint16_t stop_pos);
i2c_status_t AS5600_SetMaxAngle(uint16_t max_angle);
i2c_status_t AS5600_Configure(uint8_t power_mode, uint8_t hysteresis, 
                              uint8_t output_stage, uint8_t pwm_freq,
                              uint8_t slow_filter, uint8_t fast_filter);

#endif /* _AS5600_H_ */
