/*
 * app_i2c.c
 *
 *  Created on: May 23, 2023
 *      Author: admin
 */

#include "app_i2c.h"
#include "app_log.h"

uint8_t i2c_txBuffer[] = "G";
uint8_t i2c_txBufferSize = sizeof(i2c_txBuffer);
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE];
uint8_t i2c_rxBufferIndex;

// Transmission flags
volatile bool i2c_rxInProgress;
volatile bool i2c_startTx;

void initCMU(void)
{
  // Enabling clock to the I2C, GPIO, LE
  CMU_ClockEnable(cmuClock_I2C0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Starting LFXO and waiting until it is stable
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
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

void initI2C(void)
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
  I2C0->SADDR = I2C_ADDRESS;
  I2C0->CTRL |= I2C_CTRL_SLAVE | I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSN;
  enableI2cSlaveInterrupts();
}

/**************************************************************************//**
 * @brief  Transmitting I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
void performI2CTransfer()
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Setting LED to indicate transfer
  //GPIO_PinOutSet(LED_PORT, LED_1_PIN);

  // Initializing I2C transfer
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = i2c_txBuffer;
  //i2cTransfer.buf[0].data   = data;
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
  //GPIO_PinOutClear(LED_PORT, LED_1_PIN);
  //enableI2cSlaveInterrupts();
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


size_t writeRegister8_value(uint8_t addr, uint8_t reg, uint8_t value)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[2];
//  for(int i = 1; i < 8; i++){
//      i2c_write_data[i] = value[i - 1];
//  }

  seq.addr  =  addr << 1;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = reg;
//  for(int j = 0; j < 8; j++){
//        app_log("Data %d: %x\n\r", j,i2c_write_data[j]);
//    }
  app_log("Datadone\n\r");
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
  app_log("transdone\n\r");
  if (ret != i2cTransferDone) {
    return 1; //fail
  }
  return 0;
}
