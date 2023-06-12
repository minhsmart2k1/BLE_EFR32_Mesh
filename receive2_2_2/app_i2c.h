/*
 * app_i2c.h
 *
 *  Created on: May 23, 2023
 *      Author: admin
 */

#ifndef APP_I2C_H_
#define APP_I2C_H_

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
//#include "em_gpio.h"
#include "stddef.h"

#define CORE_FREQUENCY              14000000
#define RTC_MIN_TIMEOUT                32000
#define I2C_ADDRESS                     0xAA
#define I2C_ADDRESS_MASK                0xFE // Must match exact I2C_ADDRESS
#define I2C_RXBUFFER_SIZE                 10

#ifndef USER_FILES_USER_PERIPHERAL_INIT_H_
#define USER_FILES_USER_PERIPHERAL_INIT_H_

//Define port and pin for led
#define LED_PORT    (gpioPortF)//platform\emlib\inc
#define LED_0_PIN     (4U)
#define LED_1_PIN     (5U)

//Define port and pin for button
#define BUTTON_PORT   (gpioPortF)
#define BUTTON_0_PIN  (6U)
#define BUTTON_1_PIN  (7U)

#endif /* USER_FILES_USER_PERIPHERAL_INIT_H_ */

// Buffers++

void initCMU(void);
//void initGPIO(void);
void enableI2cSlaveInterrupts(void);
void disableI2cInterrupts(void);
void initI2C(void);
void performI2CTransfer();
void receiveI2CData(void);
uint8_t writeRegister8(uint8_t addr, uint32_t reg);
size_t writeRegister8_value(uint8_t addr, uint8_t reg, uint8_t value);
#endif /* APP_I2C_H_ */

