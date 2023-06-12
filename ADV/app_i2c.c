/*
 * app_i2c.c
 *
 *  Created on: Oct 2, 2022
 *      Author: Phat_Dang
 */
#include "app_i2c.h"
#include <stddef.h>
#include <stdlib.h>
//#include "max30102.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include <string.h>
//#include "stddef.h"
#define MAX30102_READ_ID      0xFF

#define MAX30102_FWREV    0xFE

// Status Registers
static const uint8_t MAX30105_INTSTAT1 =    0x00;
static const uint8_t MAX30105_INTSTAT2 =    0x01;
static const uint8_t MAX30105_INTENABLE1 =    0x02;
static const uint8_t MAX30105_INTENABLE2 =    0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR =  0x04;
static const uint8_t MAX30105_FIFOOVERFLOW =  0x05;
static const uint8_t MAX30105_FIFOREADPTR =   0x06;
static const uint8_t MAX30105_FIFODATA =    0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG =    0x08;
static const uint8_t MAX30105_MODECONFIG =    0x09;
static const uint8_t MAX30105_PARTICLECONFIG =  0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP =   0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP =   0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP =   0x0E;
static const uint8_t MAX30105_LED_PROX_AMP =  0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT =    0x1F;
static const uint8_t MAX30105_DIETEMPFRAC =   0x20;
static const uint8_t MAX30105_DIETEMPCONFIG =   0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH =   0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID =    0xFE;
static const uint8_t MAX30105_PARTID =      0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30105_INT_A_FULL_MASK =   0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE =   0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE =  0x00;

static const uint8_t MAX30105_INT_DATA_RDY_MASK = 0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE = 0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_INT_ALC_OVF_MASK = 0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE =  0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30105_INT_PROX_INT_MASK = 0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = 0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =  0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 =   0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 =   0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 =   0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 =   0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 =  0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 =  0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK =   0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30105_A_FULL_MASK =   0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK =   0x7F;
static const uint8_t MAX30105_SHUTDOWN =    0x80;
static const uint8_t MAX30105_WAKEUP =      0x00;

static const uint8_t MAX30105_RESET_MASK =    0xBF;
static const uint8_t MAX30105_RESET =       0x40;

static const uint8_t MAX30105_MODE_MASK =     0xF8;
static const uint8_t MAX30105_MODE_REDONLY =  0x02;
static const uint8_t MAX30105_MODE_REDIRONLY =  0x03;
static const uint8_t MAX30105_MODE_MULTILED =   0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK =   0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 =   0x00;
static const uint8_t MAX30105_ADCRANGE_4096 =   0x20;
static const uint8_t MAX30105_ADCRANGE_8192 =   0x40;
static const uint8_t MAX30105_ADCRANGE_16384 =  0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 =   0x00;
static const uint8_t MAX30105_SAMPLERATE_100 =  0x04;
static const uint8_t MAX30105_SAMPLERATE_200 =  0x08;
static const uint8_t MAX30105_SAMPLERATE_400 =  0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 =  0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 =   0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 =  0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 =  0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 =  0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK =    0xF8;
static const uint8_t MAX30105_SLOT2_MASK =    0x8F;
static const uint8_t MAX30105_SLOT3_MASK =    0xF8;
static const uint8_t MAX30105_SLOT4_MASK =    0x8F;

static const uint8_t SLOT_NONE =        0x00;
static const uint8_t SLOT_RED_LED =       0x01;
static const uint8_t SLOT_IR_LED =        0x02;
static const uint8_t SLOT_GREEN_LED =       0x03;
static const uint8_t SLOT_NONE_PILOT =      0x04;
static const uint8_t SLOT_RED_PILOT =     0x05;
static const uint8_t SLOT_IR_PILOT =      0x06;
static const uint8_t SLOT_GREEN_PILOT =     0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;

uint8_t activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
  } sense_struct; //This is our circular buffer of readings from the sensor
  sense_struct sense;

uint8_t i2c_txBuffer[] = "7";
uint8_t i2c_txBufferSize = sizeof(i2c_txBuffer);
uint8_t i2c_rxBuffer[13];
uint8_t i2c_rxBufferIndex;

uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

// Transmission flags
volatile bool i2c_rxInProgress;
volatile bool i2c_startTx;


void initCMU(void)
{
  // Enabling clock to the I2C, GPIO, LE
  CMU_ClockEnable(cmuClock_I2C0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_HFLE, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);

  // Starting LFXO and waiting until it is stable
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
}

void initGPIO(void)
{
  // Configure PB0 as input and int
  GPIO_PinModeSet(BUTTON_PORT, BUTTON_0_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(BUTTON_PORT, BUTTON_0_PIN, false, true, true);

  // Configure LED0 and LED1 as output
  GPIO_PinModeSet(LED_PORT, LED_0_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(LED_PORT, LED_1_PIN, gpioModePushPull, 0);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void enableI2cSlaveInterrupts(void)
{
  I2C_IntClear(I2C0, I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_SSTOP); // ADDR Interrupt Enable || RXDATAV Interrupt Enable || SSTOP Interrupt Enable
  I2C_IntEnable(I2C0, I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_SSTOP);
  NVIC_EnableIRQ(I2C0_IRQn); //I2C0_IRQn             = 17, /*!< 16+17 EFR32 I2C0 Interrupt */
}

/**************************************************************************//**
 * @brief  disables I2C interrupts
 *****************************************************************************/
void disableI2cInterrupts(void)
{
  NVIC_DisableIRQ(I2C0_IRQn);
  I2C_IntDisable(I2C0, I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_SSTOP);
  I2C_IntClear(I2C0, I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_SSTOP);
}

/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/

uint8_t initI2C(void)
{
  // Using default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  // Use ~400khz SCK
  i2cInit.freq = I2C_FREQ_FAST_MAX; //392157

  // Using PC10 (SDA) and PC11 (SCL)
  GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(gpioPortC, 11, gpioModeWiredAndPullUpFilter, 1);
  //GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);

  // Enable pins at location 15 as specified in datasheet
  I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
  I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK)) | I2C_ROUTELOC0_SDALOC_LOC16;
  I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK)) | I2C_ROUTELOC0_SCLLOC_LOC14;

  // Initializing the I2C
  I2C_Init(I2C0, &i2cInit);

  // Setting the status flags and index
  i2c_rxInProgress = false;
  i2c_startTx = false;
  i2c_rxBufferIndex = 0;

  // Setting up to enable slave mode
  I2C0->SADDR = I2C_ADDRESS << 1;
  I2C0->CTRL |= I2C_CTRL_SLAVE | I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSN;
  //enableI2cSlaveInterrupts();
  I2C_Enable(I2C0, true);
  return 0;
}

/**************************************************************************//**
 * @brief  Transmitting I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
void performI2CTransfer(void)
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Setting LED to indicate transfer
  GPIO_PinOutSet(LED_PORT, LED_1_PIN);

  // Initializing I2C transfer
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = i2c_txBuffer;
  i2cTransfer.buf[0].len    = i2c_txBufferSize;
  i2cTransfer.buf[1].data   = i2c_rxBuffer;
  i2cTransfer.buf[1].len    = I2C_RXBUFFER_SIZE;
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress)
  {
    result = I2C_Transfer(I2C0);
  }

  // Clearing pin to indicate end of transfer
  GPIO_PinOutClear(LED_PORT, LED_1_PIN);
  enableI2cSlaveInterrupts();
}

/**************************************************************************//**
 * @brief  Receiving I2C data. Along with the I2C interrupt, it will keep the
  EFM32 in EM1 while the data is received.
 *****************************************************************************/
void receiveI2CData(void)
{
  while(i2c_rxInProgress)
  {
    EMU_EnterEM1();

  }
}

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C0_IRQHandler(void)
{
  int status;

  status = I2C0->IF;

  if (status & I2C_IF_ADDR)
  {
    // Address Match
    // Indicating that reception is started
    i2c_rxInProgress = true;
    I2C0->RXDATA;
    i2c_rxBufferIndex = 0;

    I2C_IntClear(I2C0, I2C_IFC_ADDR);

  } else if (status & I2C_IF_RXDATAV)
  {
    // Data received
    i2c_rxBuffer[i2c_rxBufferIndex] = I2C0->RXDATA;
    i2c_rxBufferIndex++;
  }

  if(status & I2C_IEN_SSTOP){
    // Stop received, reception is ended
    I2C_IntClear(I2C0, I2C_IEN_SSTOP);
    i2c_rxInProgress = false;
    i2c_rxBufferIndex = 0;
  }
}


/***************************************************************************//**
 * @brief GPIO Interrupt handler
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  // Clear pending
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  // If RX is not in progress, a new transfer is started
  if (!i2c_rxInProgress)
  {
    disableI2cInterrupts();
      i2c_startTx = true;
  }
}


sl_status_t max30102_Init()
{
  sl_status_t status;

  status = SL_STATUS_OK;

  if (!max30102_present(NULL)) {
    /* Wait for sensor to become ready */
    sl_sleeptimer_delay_millisecond(80);

    if (!max30102_present(NULL)) {
      status = SL_STATUS_INITIALIZATION;
    }
  }

  return status;
}

/**************************************************************************//**
 *   Checks if a Si7006/13/20/21 is present on the I2C bus or not.
 *****************************************************************************/
bool max30102_present(uint8_t *device_id)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[1];

  seq.addr  =  I2C_ADDRESS << 1;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = MAX30102_READ_ID;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;
  ret = I2C_TransferInit(I2C0, &seq);
  //ret = I2CSPM_Transfer(i2cspm, &seq);
  while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }
  if (ret != i2cTransferDone) {
    return 0;
  }
  if (NULL != device_id) {
    *device_id = i2c_read_data[0];
  }
  return 1;
}

uint8_t readRegister8(uint8_t addr, uint8_t reg)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[1];

  seq.addr  =  addr << 1;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = reg;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;
  ret = I2C_TransferInit(I2C0, &seq);
  //ret = I2CSPM_Transfer(i2cspm, &seq);
  while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }
  if (ret != i2cTransferDone) {
    return 0; //fail
  }
  return i2c_read_data[0];
}

uint8_t readRegister8_buf(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  I2C_TransferSeq_TypeDef    seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t                    i2c_read_data[len];
    uint8_t                    i2c_write_data[1];

    seq.addr  = addr << 1;
    seq.flags = I2C_FLAG_WRITE_READ;
    /* Select command to issue */
    i2c_write_data[0] = reg;
    seq.buf[0].data   = i2c_write_data;
    seq.buf[0].len    = 1;
    /* Select location/length of data to be read */
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = len;

    ret = I2C_TransferInit(I2C0, &seq);

    while (ret == i2cTransferInProgress)
        {
          ret = I2C_Transfer(I2C0);
        }
      if (ret != i2cTransferDone) {
        return 0; //fail
      }
//      for(int j = 0; j < len; j++)
//        app_log_debug("\r read fifo %d: %d\n\r", j, i2c_read_data[j]);

  int i = 0;
  while (len != 0)
      {
              *buf = i2c_read_data[i];

          len--;
          buf++;
          i++;
      }
  return 1;

}

uint8_t readRegister8_point(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[1];

  seq.addr  =  addr << 1;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = reg;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;
  ret = I2C_TransferInit(I2C0, &seq);
  //ret = I2CSPM_Transfer(i2cspm, &seq);
  while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }
  if (ret != i2cTransferDone) {
    return 0; //fail
  }
  return i2c_read_data[0];
}

uint8_t writeRegister8(uint8_t addr, uint8_t reg)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[2];

  seq.addr  =  addr << 1;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = reg;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;
  ret = I2C_TransferInit(I2C0, &seq);
  //ret = I2CSPM_Transfer(i2cspm, &seq);
  while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }
  if (ret != i2cTransferDone) {
    return 1; //fail
  }
  return 0;
}

size_t writeRegister8_value(uint8_t addr, uint8_t reg, uint8_t value)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[2];

  seq.addr  =  addr << 1;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = reg;
  i2c_write_data[1] = value;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;
  ret = I2C_TransferInit(I2C0, &seq);
  //ret = I2CSPM_Transfer(i2cspm, &seq);
  while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }
  if (ret != i2cTransferDone) {
    return 1; //fail
  }
  return 0;
}

//Given a register, read it, mask it, and then set the thing
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(I2C_ADDRESS, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8_value(I2C_ADDRESS, reg, originalContents | thing);
}

/**************************************************************************//**
 *   Enable DIE Temp interrupt
 *****************************************************************************/
void enableDIETEMPRDY(void) {
  bitMask(MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_ENABLE);
}

// Die Temperature
// Returns temp in C
float readTemperature() {

  //DIE_TEMP_RDY interrupt must be enabled

  // Step 1: Config die temperature register to take 1 temperature sample
  writeRegister8_value(I2C_ADDRESS, MAX30105_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100m
    //uint8_t response = readRegister8(_i2caddr, MAX30105_DIETEMPCONFIG); //Original way
    //if ((response & 0x01) == 0) break; //We're done!
  while(true){
       sl_sleeptimer_delay_millisecond(100);

         //Check to see if DIE_TEMP_RDY interrupt is set
         uint8_t response = readRegister8(I2C_ADDRESS, MAX30105_INTSTAT2);
         if ((response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
   }
    //if ((response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
  //TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readRegister8(I2C_ADDRESS, MAX30105_DIETEMPINT);
  uint8_t tempFrac = readRegister8(I2C_ADDRESS, MAX30105_DIETEMPFRAC); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)(tempInt + (float)(tempFrac * 0.0625));
}

//End Interrupt configuration

void softReset(void) {
  bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  while (true)
  {
      sl_sleeptimer_delay_millisecond(100);
    uint8_t response = readRegister8(I2C_ADDRESS, MAX30105_MODECONFIG);
    if ((response & MAX30105_RESET) == 0) break; //We're done!

  }
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void clearFIFO(void) {
  writeRegister8_value(I2C_ADDRESS, MAX30105_FIFOWRITEPTR, 0);
  writeRegister8_value(I2C_ADDRESS, MAX30105_FIFOOVERFLOW, 0);
  writeRegister8_value(I2C_ADDRESS, MAX30105_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

void setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8_value(I2C_ADDRESS, MAX30105_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8_value(I2C_ADDRESS, MAX30105_LED2_PULSEAMP, amplitude);
}

void setPulseAmplitudeGreen(uint8_t amplitude) {
  writeRegister8_value(I2C_ADDRESS, MAX30105_LED3_PULSEAMP, amplitude);
}

void setPulseAmplitudeProximity(uint8_t amplitude) {
  writeRegister8_value(I2C_ADDRESS, MAX30105_LED_PROX_AMP, amplitude);
}

void setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  writeRegister8_value(I2C_ADDRESS, MAX30105_PROXINTTHRESH, threshMSB);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void enableSlot(uint8_t slotNumber, uint8_t device) {

  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Setup the sensor
//The MAX30105 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30105 sensor
void setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
  softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) setFIFOAverage(MAX30105_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) setFIFOAverage(MAX30105_SAMPLEAVG_2);
  else if (sampleAverage == 4) setFIFOAverage(MAX30105_SAMPLEAVG_4);
  else if (sampleAverage == 8) setFIFOAverage(MAX30105_SAMPLEAVG_8);
  else if (sampleAverage == 16) setFIFOAverage(MAX30105_SAMPLEAVG_16);
  else if (sampleAverage == 32) setFIFOAverage(MAX30105_SAMPLEAVG_32);
  else setFIFOAverage(MAX30105_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) setLEDMode(MAX30105_MODE_MULTILED); //Watch all three LED channels
  else if (ledMode == 2) setLEDMode(MAX30105_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX30105_MODE_REDONLY); //Red only
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) setADCRange(MAX30105_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) setADCRange(MAX30105_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) setADCRange(MAX30105_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) setADCRange(MAX30105_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30105_ADCRANGE_2048);

  if (sampleRate < 100) setSampleRate(MAX30105_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) setSampleRate(MAX30105_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX30105_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX30105_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX30105_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX30105_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX30105_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX30105_SAMPLERATE_3200);
  else setSampleRate(MAX30105_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) setPulseWidth(MAX30105_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) setPulseWidth(MAX30105_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) setPulseWidth(MAX30105_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) setPulseWidth(MAX30105_PULSEWIDTH_411); //18 bit resolution
  else setPulseWidth(MAX30105_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  setPulseAmplitudeGreen(powerLevel);
  setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
  if (ledMode > 2) enableSlot(3, SLOT_GREEN_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}

//Report the most recent IR value
uint32_t getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
bool safeCheck(uint8_t maxTimeToCheck)
{
  uint8_t cnt = 0;

  while(1)
  {

  if(cnt > maxTimeToCheck) return(false);

  if(check() == true) //We found new data!
    return(true);
  sl_sleeptimer_delay_millisecond(1);
  cnt++;
  }
}

//Read the FIFO Write Pointer
uint8_t getWritePointer(void) {
  return (readRegister8(I2C_ADDRESS, MAX30105_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t getReadPointer(void) {
  return (readRegister8(I2C_ADDRESS, MAX30105_FIFOREADPTR));
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t readPointer = getReadPointer();
  uint8_t writePointer = getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
  //    _i2cPort->beginTransmission(MAX30105_ADDRESS);
  //    _i2cPort->write(MAX30105_FIFODATA);
  //    _i2cPort->endTransmission();


    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }
      app_log_debug("toGet: %d\n\r", toGet);
      bytesLeftToRead -= toGet;
      app_log_debug("bytesLeftToRead: %d\n\r", bytesLeftToRead);

      //Request toGet number of bytes from sensor
      //_i2cPort->requestFrom(MAX30105_ADDRESS, toGet);

      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);
        temp[1] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);
        temp[0] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));

    tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);;
          temp[1] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);;
          temp[0] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);;

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

      sense.IR[sense.head] = tempLong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          temp[3] = 0;
          temp[2] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);;
          temp[1] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);;
          temp[0] = readRegister8(I2C_ADDRESS, MAX30105_FIFODATA);;

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.green[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}


