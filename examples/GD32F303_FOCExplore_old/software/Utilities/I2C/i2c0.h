#ifndef _I2C_H_
#define _I2C_H_

#include "gd32f30x.h"

/* I2C peripheral definitions */
#define I2C0_PERIPH           I2C0
#define I2C0_RCU              RCU_I2C0
#define I2C0_SCL_PIN          GPIO_PIN_6
#define I2C0_SDA_PIN          GPIO_PIN_7
#define I2C0_GPIO_PORT        GPIOB
#define I2C0_GPIO_RCU         RCU_GPIOB

/* I2C speed configuration */
#define I2C_SPEED             300000U  /* 300kHz fast mode */
#define I2C_DUTYCYCLE         I2C_DTCY_2  /* I2C duty cycle 2 */

/* Timeout and unlock delay budgets (loop count, not time unit). */
#ifndef I2C0_TIMEOUT_LOOPS
#define I2C0_TIMEOUT_LOOPS 12000U
#endif

#ifndef I2C0_UNLOCK_HALF_PULSE_DELAY_LOOPS
#define I2C0_UNLOCK_HALF_PULSE_DELAY_LOOPS 600U
#endif

#ifndef I2C0_UNLOCK_STOP_DELAY_LOOPS
#define I2C0_UNLOCK_STOP_DELAY_LOOPS 600U
#endif

/* I2C status codes */
typedef enum {
    I2C_OK = 0,
    I2C_ERROR,
    I2C_TIMEOUT,
    I2C_NACK
} i2c_status_t;

/* Function prototypes */
void I2C0_Init(void);
void I2C0_Unlock(void);
i2c_status_t I2C0_WriteByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
i2c_status_t I2C0_ReadByte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data);
i2c_status_t I2C0_WriteBytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
i2c_status_t I2C0_ReadBytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
i2c_status_t I2C0_CheckDevice(uint8_t device_addr);

#endif /* _I2C_H_ */

