/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "app_log.h"
#include "custom_adv.h"
#include "sl_simple_timer.h"
#include "driver_max30102.h"
#include "example/driver_max30102_fifo.h"
#include "algorithm_by_RF.h"

CustomAdv_t sData; // Our custom advertising data stored here
max30102_handle_t s;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

uint8_t pos = 0;

#define FreqS 25    //sampling frequency
#define BUFFER_SIZE (FreqS * 4)

float temp, n_spo2, ratio, correl; //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heartrate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid

//static uint32_t id1 = 0x01;
//static uint32_t id2 = 0x01;
uint8_t id1 = 0x01;
uint8_t id2 = 0x02;


//static int32_t temperature = 0;
static sl_simple_timer_t sensor_timer;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void addToBuffer(uint32_t *irBuffer, uint32_t *redBuffer, uint32_t *raw_red, uint32_t *raw_ir, uint8_t *len, uint8_t *pos){
  int i = 0;
  if(*pos == 100){
      //*pos = 0;
      //dumping the first *len sets of samples in the memory and shift the last (100 - *len) sets of samples to the top
      for(i = *len; i < 100; i++){
            irBuffer[i - *len] = irBuffer[i];
            redBuffer[i - *len] = redBuffer[i];
        }
      //take 25 sets of samples before calculating the heart rate.
      for (i = 100 - *len; i < 100; i++){
          irBuffer[i] = *raw_ir;
                redBuffer[i] = *raw_red;
                raw_ir++;
                raw_red++;
      }
  }

  if((*len + *pos) > 100)
    //*pos = 0;
    *len = 100 - *pos;

  for(i = *pos; i < (*len + *pos); i++){
      irBuffer[i] = *raw_ir;
      redBuffer[i] = *raw_red;
      raw_ir++;
      raw_red++;
  }
  *pos = *pos + *len;
}


static void sensor_timer_cb(sl_simple_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;
  uint32_t raw_red[32];
  uint32_t raw_ir[32];
  uint8_t len = 32;
  uint16_t raw = 0;
  max30102_fifo_read(s, (uint32_t *)raw_red, (uint32_t *)raw_ir, (uint8_t *)&len);
  max30102_read_temperature(&s, &raw, &temp);
        addToBuffer((uint32_t *)irBuffer, (uint32_t *)redBuffer,(uint32_t *)raw_red, (uint32_t *)raw_ir, (uint8_t *)&len, (uint8_t *)&pos);
        app_log("Pos: %d\n\r", pos);
        if(pos == 100){
                  //maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
            rf_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &n_spo2, &ch_spo2_valid, &n_heartrate, &ch_hr_valid, &ratio, &correl);
              }
        app_log("HR: %ld, HRvalid: %d, SpO2: %.2f, SpO2valid: %d, Temp: %.2f \n\r",n_heartrate, ch_hr_valid, n_spo2, ch_spo2_valid, temp);
        for(int i = 0; i < len; i++)
          app_log("Raw red: %ld, Raw ir: %ld, Length: %d: \n\r", raw_ir[i], raw_red[i], len);
  // send temperature measurement indication to connected client
//  sl_sensor_rht_get(&id1, &id2);
   //app_log("id1: %d  id2: %d  id3: %d  id4: %d\r\n", id1, id2, id3, id4);
   //update_adv_data(&sData, advertising_set_handle, id1, id2, id3, id4);
        if(ch_hr_valid == 0 && ch_spo2_valid == 0){
            n_heartrate = 0;
            n_spo2 = 0;
        }
        update_adv_data(&sData, advertising_set_handle, id1, id2, n_heartrate, (int32_t)(n_spo2*100.0), (int16_t)(temp*100));
}

SL_WEAK void app_init(void)
{
  float temp = 0.0;
    uint16_t raw = 0;
    uint8_t deviceID, reversionID;

  initCMU();
    initGPIO();
    initI2C();
    //max30102_handle_t s;
    max30102_init(&s);
      max30102_read_temperature(&s, &raw, &temp);
      app_log("Raw: %d, Temp: %f\n\r", raw, temp);
      max30102_get_id(&s, &reversionID, &deviceID);
      app_log("Part ID: 0x%X, \tReversion ID: 0x%X\n\r", deviceID,reversionID);
      max30102_fifo_init(s);
  sl_status_t sc;
  // Init temperature sensor.
  sc = sl_sensor_rht_init();
  app_assert_status(sc);
  sc = sl_simple_timer_start(&sensor_timer,
                             1000,
                             sensor_timer_cb,
                             NULL,
                             true);
  app_assert_status(sc);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);

      sl_bt_advertiser_set_channel_map(advertising_set_handle, 7);

      fill_adv_packet(&sData, 0x06, 0x02FF, id1, id2, n_heartrate, (int32_t)(n_spo2*100), (int32_t)(temp*100), "CustomAdv");
      start_adv(&sData, advertising_set_handle);
      app_assert_status(sc);
      app_log("Started advertising\r\n");

      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
