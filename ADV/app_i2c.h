/*
 * app_i2c.h
 *
 *  Created on: Oct 2, 2022
 *      Author: Phat_Dang
 */

#ifndef APP_I2C_H_
#define APP_I2C_H_

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "app_log.h"
//#include "heartRate.h"

#define CORE_FREQUENCY              14000000
#define RTC_MIN_TIMEOUT                32000
#define I2C_ADDRESS                     0x57
#define MAX30102_DEVICE_ID 0x15

//#define I2C_ADDRESS                       13
#define I2C_RXBUFFER_SIZE                 10

#define BUFFER_LENGTH 32
#define I2C_BUFFER_LENGTH 32

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
void initGPIO(void);
void enableI2cSlaveInterrupts(void);
void disableI2cInterrupts(void);
uint8_t initI2C(void);
void performI2CTransfer(void);
void receiveI2CData(void);
bool max30102_present(uint8_t *device_id);
sl_status_t max30102_Init();
uint8_t readRegister8(uint8_t addr, uint8_t reg);
uint8_t readRegister8_buf(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t writeRegister8(uint8_t addr, uint8_t reg);
size_t writeRegister8_value(uint8_t addr, uint8_t reg, uint8_t value);
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
void enableDIETEMPRDY(void);
float readTemperature();
void softReset(void);
void setFIFOAverage(uint8_t numberOfSamples);
void clearFIFO(void);
void enableFIFORollover(void);
void setLEDMode(uint8_t mode);
void setADCRange(uint8_t adcRange);
void setSampleRate(uint8_t sampleRate);
void setPulseWidth(uint8_t pulseWidth);
void setPulseAmplitudeRed(uint8_t amplitude);
void setPulseAmplitudeIR(uint8_t amplitude);
void setPulseAmplitudeGreen(uint8_t amplitude);
void setPulseAmplitudeProximity(uint8_t amplitude);
void setProximityThreshold(uint8_t threshMSB);
void enableSlot(uint8_t slotNumber, uint8_t device);
uint32_t getIR(void);
bool safeCheck(uint8_t maxTimeToCheck);
uint16_t check(void);
uint8_t getWritePointer(void);
uint8_t getReadPointer(void);
void setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
//sl_status_t max30102_get_firmware_revision(sl_i2cspm_t *i2cspm, uint8_t addr, uint8_t *fwRev);

#endif /* APP_I2C_H_ */
