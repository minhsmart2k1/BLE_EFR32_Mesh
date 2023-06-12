/***************************************************************************//**
 * @file app.c
 * @brief Core application logic for the vendor server node.
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
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
 ******************************************************************************
 * # Experimental Quality
 * This code has not been formally tested and is provided as-is. It is not
 * suitable for production environments. In addition, this code will not be
 * maintained and there may be no bug maintenance planned for these resources.
 * Silicon Labs may update projects from time to time.
 ******************************************************************************/
#include <stdio.h>
#include "em_common.h"
#include "app_assert.h"
#include "app_log.h"

#include "sl_status.h"
#include "app.h"

#include "sl_btmesh_api.h"
#include "sl_bt_api.h"
#include "sl_simple_timer.h"
#include "sl_sleeptimer.h"
#include "sl_sensor_rht.h"

#include "sl_btmesh_factory_reset.h"
#include "sl_btmesh_wstk_lcd.h"

#include "em_cmu.h"
#include "em_gpio.h"
#include "sl_simple_button_instances.h"
#include "gatt_db.h"
#include "btl_interface.h"
#include "btl_interface_storage.h"

/// Advertising Provisioning Bearer
#define PB_ADV                         0x1
/// GATT Provisioning Bearer
#define PB_GATT                        0x2
/// Element index
#define ELEM_INDEX                     0
/// Model ID
#define MODEL_ID                       0x1111
/// Vendor ID
#define VENDOR_ID                      0x1234
/// Length of the display name buffer
#define NAME_BUF_LEN                   20
/// Length of the array
#define ARRAY_LENGTH                   4031
/// Test length
#define TEST_LENGTH                     165

/// Length timer
#define TIME_BUF_LEN                   20


static bool provisioned = false;
static bool full_reset = false;
static uint16_t appkey_index = 0xFFFFu;
//static uint16_t server_address = 0xFFFFu;

static uint8_t opcodes[] = {11};


#ifdef PROV_LOCALLY
// Group Addresses
// Choose any 16-bit address starting at 0xC000
#define CUSTOM_STATUS_GRP_ADDR                      0xC001  // Server PUB address
#define CUSTOM_CTRL_GRP_ADDR                        0xC002  // Server SUB address

// The default settings of the network and the node
#define NET_KEY_IDX                                 0
#define APP_KEY_IDX                                 0
#define IVI                                         0
#define DEFAULT_TTL                                 5


/// Version number for this application (uint32_t)
#define APP_PROPERTIES_VERSION 1
// #define ELEMENT_ID
#endif // #ifdef PROV_LOCALLY

// Buttons
#define EX_B0_PRESS                                 ((1) << 5)
#define EX_B1_PRESS                                 ((1) << 6)

// Button state.
static bool btn0_report_flag = false;
static bool btn1_report_flag = false;
static volatile bool btn0_pressed = false;
static volatile bool btn1_pressed = false;


//the advertising set handle allocated from bluetooth stack
static uint8_t advertising_set_handle = 0xff;





/// Length of the display name buffer
#define NAME_BUF_LEN                   20
#define TEST_LENGTH                     165

static uint8_t tempArr[TEST_LENGTH] = {0};
static size_t bytesLeft = 0;
static size_t loop = 0;
static size_t payload_len = 0;
static uint8_t* payload = NULL;

static uint16_t publish_handle;
static unsigned int time_elapsed;
static bool send_started = false;
static void set_device_name(uuid_128*);
static void timer_cb(sl_simple_timer_t*, void*);


// Timing
// Check section 4.2.2.2 of Mesh Profile Specification 1.0 for format
#define STEP_RES_100_MILLI                          0
#define STEP_RES_1_SEC                              ((1) << 6)
#define STEP_RES_10_SEC                             ((2) << 6)
#define STEP_RES_10_MIN                             ((3) << 6)

#define STEP_RES_BIT_MASK                           0xC0

#define TEMP_DATA_LENGTH               12
#define TEMPP_DATA_LENGTH               4
//#define TEMP_DATA_LENGTH               240
#define HUM_DATA_LENGTH                4
#define UPDATE_INTERVAL_LENGTH          1
#define UNIT_DATA_LENGTH                1

#define NUMBER_OF_OPCODES               11


#define PRIMARY_ELEMENT                 0
#define MY_VENDOR_ID                    0x1234

#define MY_MODEL_SERVER_ID              0x1111

#define ACK_REQ                         (0x1)
#define STATUS_UPDATE_REQ               (0x2)

#define INDEX_OF(x)                     ((x) - 1)

/// Advertising Provisioning Bearer
#define PB_ADV                                      0x1
/// GATT Provisioning Bearer
#define PB_GATT                                     0x2

int32_t temmm;
int32_t xx;


typedef enum {
  temperature_get = 0x1,
  temperature_status,
  unit_get,
  unit_set,
  unit_set_unack,
  unit_status,
  update_interval_get,
  update_interval_set,
  update_interval_set_unack,
  update_interval_status,
  test
} my_msg_t;

typedef enum {
  celsius = 0x1,
  fahrenheit
} unit_t;

typedef struct {
  uint16_t elem_index;
  uint16_t vendor_id;
  uint16_t model_id;
  uint8_t publish; // publish - 1, not - 0
  uint8_t opcodes_len;
  uint8_t opcodes_data[NUMBER_OF_OPCODES];
} my_model_t;

static uint8_t temperature[TEMP_DATA_LENGTH];
static uint8_t datatx[TEMPP_DATA_LENGTH] = {0, 0, 0, 0};
static uint8_t myArr[4] = {0, 0, 0, 0};
//static uint8_t temperature[TEMP_DATA_LENGTH] = {0};

static unit_t unit[UNIT_DATA_LENGTH] = {
    celsius
};
// Check section 4.2.2.2 of Mesh Profile Specification 1.0 for format
static uint8_t update_interval[UPDATE_INTERVAL_LENGTH] = {
    0
};

static uint32_t periodic_timer_ms = 0;

static my_model_t my_model = {
    .elem_index = PRIMARY_ELEMENT,
    .vendor_id = MY_VENDOR_ID,
    .model_id = MY_MODEL_SERVER_ID,
    .publish = 1,
    .opcodes_len = NUMBER_OF_OPCODES,
    .opcodes_data[0] = temperature_get,
    .opcodes_data[1] = temperature_status,
    .opcodes_data[2] = unit_get,
    .opcodes_data[3] = unit_set,
    .opcodes_data[4] = unit_set_unack,
    .opcodes_data[5] = unit_status,
    .opcodes_data[6] = update_interval_get,
    .opcodes_data[7] = update_interval_set,
    .opcodes_data[8] = update_interval_set_unack,
    .opcodes_data[9] = update_interval_status
//    .opcodes_data[10] = test
};


/// Local provisioning
#ifdef PROV_LOCALLY
static uint16_t uni_addr = 0;

static aes_key_128 enc_key = {
    .data = "\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03"
};
#endif /* #ifdef PROV_LOCALLY */

static sl_simple_timer_t timer;
static void read_temperature(void);
static void send_rssi(void);
static void setup_periodcal_update(uint8_t interval);
static void delay_reset_ms(uint32_t ms);

static sl_simple_timer_t scan_timer;
#define REFRESH_PERIOD                      (3 * 32768)
#define REFRESH_TIMER_ID                    (5)
//sl_sleeptimer_timer_handle_t sleeptimer_handle;
static void sleeptimer_callback(sl_sleeptimer_timer_handle_t *timer, void *data);
static void start_timer(uint32_t period_ms);
#define MAX_SCANNED_DEVICES           10
static sl_bt_evt_scanner_scan_report_t scanned_devices[MAX_SCANNED_DEVICES];
static uint8_t num_scanned_devices = 0, i, *address;
static uint8_t closest_device;
//static void scanner_timer_cb(/*sl_bt_evt_scanner_scan_report_t *pResp,*/ sl_simple_timer_t *timer, void *data);
//static void scanner_timer_cb();
static bool jump = false;
static int8_t max_rssi;

//static int32_t get_slot_info();
/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  app_log("======================\r\n");
  app_log("Server Device\r\n");
  app_log("Server da san sang \r\n");

  ////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  if (sl_simple_button_get_state(&sl_button_btn0) == SL_SIMPLE_BUTTON_PRESSED){
      sl_btmesh_initiate_full_reset();
      app_log("Full reset\r\n");
      full_reset = true;
      app_log("Node reset \r\n");
      sl_btmesh_LCD_write("Full reset server", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);

  }
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
  ///
  if (btn0_report_flag ) {
      app_log("Button even\r\n");
      btn0_report_flag = 0 ;
  }
}
uuid_128 uuid;
/**************************************************************************/ /**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  // Button 1 state changed
  if (handle == SL_SIMPLE_BUTTON_INSTANCE(1)) {
      // Button 1 pressed.
      if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
          btn1_pressed = true;
          sl_bt_external_signal(EX_B1_PRESS);
      }
      // Button 1 released.
      else if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED) {
          btn1_pressed = false;
      }
      btn1_report_flag = true;
  }
  // Button 0 state changed
  else if (handle == SL_SIMPLE_BUTTON_INSTANCE(0)) {
      // Button 0 pressed
      if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
          btn0_pressed = true;
          sl_bt_external_signal(EX_B0_PRESS);
      }
      // Button 0  released
      else if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED) {
          btn0_pressed = false;
      }
      btn0_report_flag = true;
  }
}

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the UUID of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] uuid  Pointer to device UUID.
 ******************************************************************************/
static void set_device_name(uuid_128 *uuid)
{
  char name[NAME_BUF_LEN];
  sl_status_t result;

  // Create unique device name using the last two bytes of the device UUID
  snprintf(name,
           NAME_BUF_LEN,
           "vendor server %02x%02x",
           uuid->data[14],
           uuid->data[15]);

  app_log("Device name: '%s'\r\n", name);

  result = sl_bt_gatt_server_write_attribute_value(gattdb_device_name,
                                                   0,
                                                   strlen(name),
                                                   (uint8_t *)name);
  if (result != SL_STATUS_OK) {
      app_log("sl_bt_gatt_server_write_attribute_value() failed, code %x\r\n",
              result);
  }

  // Show device name on the LCD
  sl_btmesh_LCD_write(name, SL_BTMESH_WSTK_LCD_ROW_NAME_CFG_VAL);
}
/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
static int on_phase = 0;
static void scanner_timer_cb()
{
  sl_status_t sc;
    max_rssi = -80;
    closest_device = num_scanned_devices;
    if (on_phase == 0)
      {
            num_scanned_devices = 0;

        sl_bt_scanner_start (gap_1m_phy, scanner_discover_observation);

        on_phase = 1;
        app_log_info("sl_bt_scanner_start scanning\n");
      }
    else
      {
        sl_bt_scanner_stop ();
  //      for (i = 0; i < num_scanned_devices; i++)
  //        {
  //          if (scanned_devices[i].rssi > max_rssi)
  //            {
  //              max_rssi = scanned_devices[i].rssi;
  //              closest_device = i;
  //            }
  //        }
  //      app_log_info("closest_device: %d\r\n", closest_device);
  //      app_log_info("num_scanned_devices: %d\r\n", num_scanned_devices);

   //     app_log_info("stop scanning\n");
  //      if (closest_device < num_scanned_devices)
  //        sc = sl_bt_connection_open (
  //            scanned_devices[closest_device].address,
  //            scanned_devices[closest_device].address_type, sl_bt_gap_phy_1m,
  //            NULL);
  //      app_assert_status(sc);
  //      app_log_info("stop scanning\n");
        on_phase = 0;
      }

//  (void)data;
//  (void)timer;
//  sl_status_t sc;
//  bd_addr addr;
//  bd_addr *remote_address;
//  int ad_len;
//  int ad_data;
//
//  if(scanned_devices->data.len) {
//    if((scanned_devices->data.data[7] ==0x01) &&(scanned_devices->data.data[8] ==0x02)) {
//    app_log_info("\r\n");
//    app_log_info("      DEVICE\r\n");
//    remote_address = &(scanned_devices->address);
////      if (evt->data.evt_scanner_scan_report.data = 0x0101) {
//    app_log_info("ADV address %02X %02X %02X %02X %02X %02X\r\n",
//                 remote_address->addr[5],
//                 remote_address->addr[4],
//                 remote_address->addr[3],
//                 remote_address->addr[2],
//                 remote_address->addr[1],
//                 remote_address->addr[0]);
//    app_log_info("RSSI %d\r\n", scanned_devices->rssi);
//    app_log_info("\r\----------------------------------\r\n");
//    app_log_info("id = 0x0%x\r\n", scanned_devices->data.data[9]);
//    app_log_info("id = 0x0%x\r\n", scanned_devices->data.data[10]);
//    app_log_info("\r\----------------------------------\r\n");
//    temperature[0]= scanned_devices->data.data[9];
//    temperature[1]= scanned_devices->data.data[10];
//    temperature[2]= scanned_devices->rssi;
//                           uint8_t opcode = 0, length = 0, *data = NULL;
//                           read_temperature();
//                           opcode = temperature_status;
//                           length = TEMP_DATA_LENGTH;
//                           data = temperature;
//                           sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
//                                                                             my_model.vendor_id,
//                                                                             my_model.model_id,
//                                                                             opcode,
//                                                                             1, length, data);
//                                 if(sc != SL_STATUS_OK) {
//                                     app_log("Set publication error: 0x%04X\r\n", sc);
//                                 } else {
//                                     app_log("Set publication done. Publishing...\r\n");
//                                     // publish the vendor model publication message
//                                     sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
//                                                                         my_model.vendor_id,
//                                                                         my_model.model_id);
//                                     if(sc != SL_STATUS_OK) {
//                                         app_log("Publish error: 0x%04X\r\n", sc);
//                                     } else {
//                                         app_log("Publish done.\r\n");
//                                     }
//                                 }
//    }
//  }
}

void sl_bt_on_event(struct sl_bt_msg *evt)
{
  sl_status_t sc;
    bd_addr addr;
    uint8_t address_type;
    uint8_t system_id[8];
    bd_addr *remote_address;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      app_log("The program has been updated \r\n");
      sl_bt_system_get_identity_address(&addr, NULL);
      app_log("Addr : %02X %02X %02X %02X %02X %02X\r\n", addr.addr[5],addr.addr[4],addr.addr[3],addr.addr[2],addr.addr[1],addr.addr[0]);
      // Initialize Mesh stack in Node operation mode,
      // wait for initialized event
      app_log("Node init\r\n");
      sc = sl_btmesh_node_init();
      app_assert_status_f(sc, "Failed to init node\n");

      // periodic scanner setting
            sl_bt_scanner_set_timing(gap_1m_phy, 20000, 2000);
            app_assert_status(sc);
            sl_bt_scanner_set_mode(gap_1m_phy, 0);
            app_assert_status(sc);
            sl_bt_scanner_start(gap_1m_phy, scanner_discover_observation);
//            sc = sl_sleeptimer_start_periodic_timer(&sleeptimer_handle, REFRESH_PERIOD, sleeptimer_callback, (void*)NULL, 0, 0);
//            app_assert_status(sc);
            sl_simple_timer_start (&scan_timer,
                                   500,
                                   scanner_timer_cb,
                                   NULL,
                                   true);

      break;

//    case sl_bt_evt_connection_opened_id:
//         sl_bt_scanner_stop();
//         break;
//         // This event indicates that a connection was closed.
//       case sl_bt_evt_connection_closed_id:
//         sl_bt_scanner_start(gap_1m_phy, scanner_discover_observation);
//         break;
         // scan response
             case sl_bt_evt_scanner_scan_report_id:
                //print_scan_resp(&evt->data.evt_scanner_scan_report);

               if(evt->data.evt_scanner_scan_report.data.len)
                 {
//                   if(evt->data.evt_scanner_scan_report.data.data[3] == 1 && evt->data.evt_scanner_scan_report.data.data[4] == 2 && evt->data.evt_scanner_scan_report.data.data[5] == 0 && evt->data.evt_scanner_scan_report.data.data[6] == 1){
//                                 app_log_info("\r\n");
//                                 app_log_info("      DEVICE\r\n");
//                                 remote_address = &(evt->data.evt_scanner_scan_report.address);
//                                 //      if (evt->data.evt_scanner_scan_report.data = 0x0101) {
//                                 app_log_info("ADV address %02X %02X %02X %02X %02X %02X\r\n",
//                                              remote_address->addr[5],
//                                              remote_address->addr[4],
//                                              remote_address->addr[3],
//                                              remote_address->addr[2],
//                                              remote_address->addr[1],
//                                              remote_address->addr[0]);
//                                 char buf[NAME_BUF_LEN];
//                                 snprintf(buf, NAME_BUF_LEN, "adv %02X:%02X:%02X:%02X:%02X:%02X",
//                                          remote_address->addr[5],
//                                          remote_address->addr[4],
//                                          remote_address->addr[3],
//                                          remote_address->addr[2],
//                                          remote_address->addr[1],
//                                          remote_address->addr[0]);
//                                 sl_btmesh_LCD_write(buf, 4);
//                                 app_log_info("RSSI %d\r\n", evt->data.evt_scanner_scan_report.rssi);
//                                 app_log_info("data %d\r\n", evt->data.evt_scanner_scan_report.data);
//                   for(int i = 0; i < evt->data.evt_scanner_scan_report.data.len; i++){
//                   app_log_info("id%d = 0x0%x\r\n", i, evt->data.evt_scanner_scan_report.data.data[i]);
                   if((evt->data.evt_scanner_scan_report.data.data[7] ==0x01) &&(evt->data.evt_scanner_scan_report.data.data[8] ==0x02))
                     {
                       sl_sleeptimer_delay_millisecond(500);
                       app_log_info("\r\n");
                       app_log_info("      DEVICE\r\n");
                       remote_address = &(evt->data.evt_scanner_scan_report.address);
                       //      if (evt->data.evt_scanner_scan_report.data = 0x0101) {
                       app_log_info("ADV address %02X %02X %02X %02X %02X %02X\r\n",
                                    remote_address->addr[5],
                                    remote_address->addr[4],
                                    remote_address->addr[3],
                                    remote_address->addr[2],
                                    remote_address->addr[1],
                                    remote_address->addr[0]);
                       char buf[NAME_BUF_LEN];
                       snprintf(buf, NAME_BUF_LEN, "adv %02X:%02X:%02X:%02X:%02X:%02X",
                                remote_address->addr[5],
                                remote_address->addr[4],
                                remote_address->addr[3],
                                remote_address->addr[2],
                                remote_address->addr[1],
                                remote_address->addr[0]);
                       sl_btmesh_LCD_write(buf, 4);
                       app_log_info("RSSI %d\r\n", evt->data.evt_scanner_scan_report.rssi);
//                       app_log_info("data %d\r\n", evt->data.evt_scanner_scan_report.data);
                       app_log_info("\r\----------------------------------\r\n");
                       app_log_info("heart rate = %ld\r\n", evt->data.evt_scanner_scan_report.data.data[9]);
                       app_log_info("SpO2 = %.2f\r\n",
                                    (float)((evt->data.evt_scanner_scan_report.data.data[16] << 24 |
                                    evt->data.evt_scanner_scan_report.data.data[15] << 16 |
                                    evt->data.evt_scanner_scan_report.data.data[14] << 8 |
                                    evt->data.evt_scanner_scan_report.data.data[13]) / 100.0));
                       app_log_info("Temp = %.2f\r\n",
                                    (float)((evt->data.evt_scanner_scan_report.data.data[18] << 8 |
                                     evt->data.evt_scanner_scan_report.data.data[17]) / 100.0));
                       app_log_info("\r\----------------------------------\r\n");

                       /////////////
                       temperature[0]=evt->data.evt_scanner_scan_report.data.data[9];
                       temperature[1]=evt->data.evt_scanner_scan_report.data.data[10];
                       temperature[2]=evt->data.evt_scanner_scan_report.data.data[11];
                       temperature[3]=evt->data.evt_scanner_scan_report.data.data[12];
                       temperature[4]=evt->data.evt_scanner_scan_report.data.data[13];
                       temperature[5]=evt->data.evt_scanner_scan_report.data.data[14];
                       temperature[6]=evt->data.evt_scanner_scan_report.data.data[15];
                       temperature[7]=evt->data.evt_scanner_scan_report.data.data[16];
                       temperature[8]=evt->data.evt_scanner_scan_report.data.data[7];
                       temperature[9]=evt->data.evt_scanner_scan_report.data.data[8];
                       temperature[10]=evt->data.evt_scanner_scan_report.data.data[17];
                       temperature[11]=evt->data.evt_scanner_scan_report.data.data[18];
                       ////////////
                       uint8_t opcode = 0, length = 0, *data = NULL;

                       opcode = temperature_status;
                       length = TEMP_DATA_LENGTH;
                       data = temperature;
                       sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
                                                                         my_model.vendor_id,
                                                                         my_model.model_id,
                                                                         opcode,
                                                                         1, length, data);
                             if(sc != SL_STATUS_OK) {
                                 app_log("Set publication error: 0x%04X\r\n", sc);
                             } else {
                                 app_log("Set publication done. Publishing...\r\n");
                                 // publish the vendor model publication message
                                 sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
                                                                     my_model.vendor_id,
                                                                     my_model.model_id);
                                 if(sc != SL_STATUS_OK) {
                                     app_log("Publish error: 0x%04X\r\n", sc);
                                 } else {
                                     app_log("Publish done.\r\n");
                                 }
                             }
                     }
                 }

               //sl_sleeptimer_delay_millisecond(2000);
               //app_log("id1: 0x0%x  id2: 0x0%x  id3: 0x0%x  id4: 0x0%x\r\n", evt->data.evt_scanner_scan_report.data.data[]
               //evt->data.evt_scanner_scan_report.data[]
               //      }
//               start_timer(2000);
               break;

      // -------------------------------
      // Handle Button Presses
//    case sl_bt_evt_system_external_signal_id: {
//      if(evt->data.evt_system_external_signal.extsignals == REFRESH_TIMER_ID){
//
//             }
//    }
//    break;

//      sb = sl_bt_evt_scanner_scan_report_id;
//      uint8_t opcode = 0, length = 0, *data = NULL;
//      // check if external signal triggered by button 0 press
//      if(evt->data.evt_system_external_signal.extsignals & EX_B0_PRESS) {
//          read_temperature();
//
//          opcode = temperature_status;
//          length = TEMP_DATA_LENGTH;
//
//          data = temperature;
//          app_log("B0 Pressed.\r\n");
//      }
//      // check if external signal triggered by button 1 press
////      if(evt->data.evt_system_external_signal.extsignals & EX_B1_PRESS) {
////          send_rssi();
////          opcode = unit_status;
////          length = TEMPP_DATA_LENGTH;
////          data = datatx;
////          app_log("B1 Pressed.\r\n");
////      }
//
//      // set the vendor model publication message
//      sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
//                                                  my_model.vendor_id,
//                                                  my_model.model_id,
//                                                  opcode,
//                                                  1, length, data);
//      if(sc != SL_STATUS_OK) {
//          app_log("Set publication error: 0x%04X\r\n", sc);
//      } else {
//          app_log("Set publication done. Publishing...\r\n");
//          // publish the vendor model publication message
//          sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
//                                              my_model.vendor_id,
//                                              my_model.model_id);
//          if(sc != SL_STATUS_OK) {
//              app_log("Publish error: 0x%04X\r\n", sc);
//          } else {
//              app_log("Publish done.\r\n");
//          }
//      }
//      break;
//    }

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
//static void scanner_timer_cb(sl_simple_timer_t *timer, void *data){
//  (void)data;
//  (void)timer;
//  jump = true;
//}
//static void scanner_timer_cb(/*sl_bt_evt_scanner_scan_report_t *pResp, sl_simple_timer_t *timer, void *data*/)


//static void start_timer(uint32_t period_ms)
//{
//  sl_status_t sc;
//  bool running;
//  sc = sl_sleeptimer_is_timer_running(&scan_timer, &running);
//  app_assert_status(sc);
//  if(running) {
//    sc = sl_sleeptimer_restart_periodic_timer_ms( &scan_timer,
//                                                period_ms,
//                                                sleeptimer_callback,
//                                                NULL,
//                                                0,
//                                                0);
//  }
//  else {
//    sc = sl_sleeptimer_start_periodic_timer_ms( &scan_timer,
//                                                period_ms,
//                                                sleeptimer_callback,
//                                                NULL,
//                                                0,
//                                                0);
//  }
//  app_assert_status_f(sc, "Failed to start periodic timer\r\n");
//}
//static void sleeptimer_callback(sl_sleeptimer_timer_handle_t *timer, void *data){
//  (void)timer;
//  (void)data;
//
////  sl_bt_scanner_scan_report();
//}
/**************************************************************************//**
 * Bluetooth Mesh stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth Mesh stack.
 *****************************************************************************/
void sl_btmesh_on_event(sl_btmesh_msg_t *evt)
{

  sl_status_t sc;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_btmesh_evt_node_initialized_id:
      sc = sl_btmesh_node_get_uuid(&uuid);
      set_device_name(&uuid);

      sl_btmesh_vendor_model_init(my_model.elem_index,
                                  my_model.vendor_id,
                                  my_model.model_id,
                                  my_model.publish,
                                  my_model.opcodes_len,
                                  my_model.opcodes_data);
      sl_btmesh_vendor_model_init(ELEM_INDEX,
                                        VENDOR_ID,
                                        MODEL_ID,
                                        1,
                                        1,
                                        opcodes);
      app_assert_status_f(sc, "Failed to initialize vendor model\r\n");

      if (!evt->data.evt_node_initialized.provisioned) {
          app_log("Node already provisioned\r\n");
          sl_btmesh_LCD_write("ready provisioned", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);


          // The Node is now initialized,
          // start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
          sc = sl_btmesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
          app_assert_status_f(sc, "Failed to start unprovisioned beaconing\n");
      }




#ifdef PROV_LOCALLY
      // Set the publication and subscription
      uint16_t appkey_index;
      uint16_t pub_address;
      uint8_t ttl;
      uint8_t period;
      uint8_t retrans;
      uint8_t credentials;
      sc = sl_btmesh_test_get_local_model_pub(my_model.elem_index,
                                              my_model.vendor_id,
                                              my_model.model_id,
                                              &appkey_index,
                                              &pub_address,
                                              &ttl,
                                              &period,
                                              &retrans,
                                              &credentials);
      if (!sc && pub_address == CUSTOM_STATUS_GRP_ADDR) {
          app_log("Configuration done already.\r\n");
      } else {
          app_log("Pub setting result = 0x%04X, pub setting address = 0x%04X\r\n", sc, pub_address);
          app_log("Add local app key ...\r\n");
          sc = sl_btmesh_test_add_local_key(1,
                                            enc_key,
                                            APP_KEY_IDX,
                                            NET_KEY_IDX);
          app_assert_status_f(sc, "Failed to add local app key\r\n");

          app_log("Bind local app key ...\r\n");
          sc = sl_btmesh_test_bind_local_model_app(my_model.elem_index,
                                                   APP_KEY_IDX,
                                                   my_model.vendor_id,
                                                   my_model.model_id);
          app_assert_status_f(sc, "Failed to bind local app key\r\n");

          app_log("Set local model pub ...\r\n");
          sc = sl_btmesh_test_set_local_model_pub(my_model.elem_index,
                                                  APP_KEY_IDX,
                                                  my_model.vendor_id,
                                                  my_model.model_id,
                                                  CUSTOM_STATUS_GRP_ADDR,
                                                  DEFAULT_TTL,
                                                  0, 0, 0);
          app_assert_status_f(sc, "Failed to set local model pub\r\n");

          app_log("Add local model sub ...\r\n");
          sc = sl_btmesh_test_add_local_model_sub(my_model.elem_index,
                                                  my_model.vendor_id,
                                                  my_model.model_id,
                                                  CUSTOM_CTRL_GRP_ADDR);
          app_assert_status_f(sc, "Failed to add local model sub\r\n");

          app_log("Set relay ...\r\n");
          sc = sl_btmesh_test_set_relay(1, 0, 0);
          app_assert_status_f(sc, "Failed to set relay\r\n");

          app_log("Set Network tx state.\r\n");
          sc = sl_btmesh_test_set_nettx(2, 4);
          app_assert_status_f(sc, "Failed to set network tx state\r\n");
      }
#endif // #ifdef PROV_LOCALLY
      break;

      break;


      // -------------------------------
      // Provisioning Events
    case sl_btmesh_evt_node_provisioned_id:
      app_log("Provisioning done.\r\n");
      sl_btmesh_LCD_write("Provisioning done.", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
      break;

    case sl_btmesh_evt_node_provisioning_failed_id:
      app_log("Provisioning failed. Result = 0x%04x\r\n",
              evt->data.evt_node_provisioning_failed.result);
      break;

    case sl_btmesh_evt_node_provisioning_started_id:
      app_log("Provisioning started.\r\n");
      sl_btmesh_LCD_write("Provisioning failed.", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
      break;

    case sl_btmesh_evt_node_key_added_id:
      app_log("got new %s key with index %x\r\n",
              evt->data.evt_node_key_added.type == 0 ? "network" : "application",
                  evt->data.evt_node_key_added.index);
      break;

    case sl_btmesh_evt_node_config_set_id:
      app_log("evt_node_config_set_id\r\n\t");
      break;

    case sl_btmesh_evt_node_model_config_changed_id:
      app_log("model config changed, type: %d, elem_addr: %x, model_id: %x, vendor_id: %x\r\n",
              evt->data.evt_node_model_config_changed.node_config_state,
              evt->data.evt_node_model_config_changed.element_address,
              evt->data.evt_node_model_config_changed.model_id,
              evt->data.evt_node_model_config_changed.vendor_id);
      break;

      // -------------------------------
      // Handle vendor model messages
    case sl_btmesh_evt_vendor_model_receive_id: {
      sl_btmesh_evt_vendor_model_receive_t *rx_evt = (sl_btmesh_evt_vendor_model_receive_t *)&evt->data.evt_vendor_model_receive;
      uint8_t action_req = 0, opcode = 0, payload_len = 0, *payload_data = NULL;

      app_log("Vendor model data received.\r\n\t"
          "Element index = %d\r\n\t"
          "Vendor id = 0x%04X\r\n\t"
          "Model id = 0x%04X\r\n\t"
          "Source address = 0x%04X\r\n\t"
          "Destination address = 0x%04X\r\n\t"
          "Destination label UUID index = 0x%02X\r\n\t"
          "App key index = 0x%04X\r\n\t"
          "Non-relayed = 0x%02X\r\n\t"
          "Opcode = 0x%02X\r\n\t"
          "Final = 0x%04X\r\n\t"
          "Data length = %u\r\n\t"
          "Payload: ",
          rx_evt->elem_index,
          rx_evt->vendor_id,
          rx_evt->model_id,
          rx_evt->source_address,
          rx_evt->destination_address,
          rx_evt->va_index,
          rx_evt->appkey_index,
          rx_evt->nonrelayed,
          rx_evt->opcode,
          rx_evt->final,
          rx_evt->payload.len);
      for(int i = 0; i < evt->data.evt_vendor_model_receive.payload.len; i++) {
          app_log("%X ", evt->data.evt_vendor_model_receive.payload.data[i]);
      }
      app_log("\r\n");


      switch (rx_evt->opcode) {
        // Server
        case temperature_get:
          app_log("Sending/publishing temperature status as response to "
              "temperature get from client...\r\n");
          read_temperature();

          action_req = ACK_REQ;
          opcode = temperature_status;
          payload_len = TEMP_DATA_LENGTH;
          payload_data = temperature;
          break;
          // Server
//        case unit_get:
//          app_log("Sending/publishing unit status as response to unit get from "
//              "client...\r\n");
//          send_rssi();
//          action_req = ACK_REQ;
//          opcode = unit_status;
//          payload_len = TEMPP_DATA_LENGTH;
//          payload_data = datatx;
//          break;
          // Server
//        case unit_set:
//          app_log("Sending/publishing unit status as response to unit set from "
//              "client...\r\n");
//          memcpy(unit, rx_evt->payload.data, rx_evt->payload.len);
//          action_req = ACK_REQ | STATUS_UPDATE_REQ;
//          opcode = unit_status;
//          payload_len = UNIT_DATA_LENGTH;
//          payload_data = (uint8_t *) unit;
//          break;
//          // Server
//        case unit_set_unack:
//          app_log("Publishing unit status as response to unit set unacknowledged "
//              "from client...\r\n");
//          memcpy(unit, rx_evt->payload.data, rx_evt->payload.len);
//          action_req = STATUS_UPDATE_REQ;
//          opcode = unit_status;
//          payload_len = UNIT_DATA_LENGTH;
//          payload_data = (uint8_t *) unit;
//          break;
//
//        case update_interval_get:
//          app_log("Publishing Update Interval status as response to Update "
//              "interval get from client...\r\n");
//          action_req = ACK_REQ;
//          opcode = update_interval_status;
//          payload_len = UPDATE_INTERVAL_LENGTH;
//          payload_data = update_interval;
//          break;
//        case update_interval_set:
//          app_log("Publishing Update Interval status as response to "
//              "update_interval_set from client...\r\n");
//          memcpy(update_interval,
//                 rx_evt->payload.data,
//                 rx_evt->payload.len);
//          action_req = ACK_REQ | STATUS_UPDATE_REQ;
//          opcode = update_interval_status;
//          payload_len = UPDATE_INTERVAL_LENGTH;
//          payload_data = update_interval;
//          setup_periodcal_update(update_interval[0]);
//          break;
//        case update_interval_set_unack:
//          app_log("Publishing Update Interval status as response to "
//              "update_interval_set_unack from client...\r\n");
//          memcpy(update_interval,
//                 rx_evt->payload.data,
//                 rx_evt->payload.len);
//          action_req = STATUS_UPDATE_REQ;
//          opcode = update_interval_status;
//          payload_len = UPDATE_INTERVAL_LENGTH;
//          payload_data = update_interval;
//          setup_periodcal_update(update_interval[0]);
//          break;
//        case test:
//          app_log("Sending/publishing data as response to "
//              "test opcode from client...\r\n");
//          //          read_temperature();
//          for(int i = 0 ; i < TEMP_DATA_LENGTH; i++){
//              temperature[i] = 1;
//          }
//          action_req = ACK_REQ;
//          opcode = test;
//          payload_len = TEMP_DATA_LENGTH;
//          payload_data = temperature;
//          break;

          // Add more cases here if more opcodes are defined
        default:
          break;
      }
//        case sl_btmesh_evt_vendor_model_send_complete_id:
//              if (bytesLeft > 0) {
//                  send_message();
//
//                  loop++;
//
//              }
//              break;

      if(action_req & ACK_REQ) {
          sc = sl_btmesh_vendor_model_send(rx_evt->source_address,
                                           rx_evt->va_index,
                                           rx_evt->appkey_index,
                                           my_model.elem_index,
                                           my_model.vendor_id,
                                           my_model.model_id,
                                           rx_evt->nonrelayed,
                                           opcode,
                                           1,
                                           payload_len,
                                           payload_data);
          // Errors that are returned from this function are usually due to low
          // memory. Low memory is non-critical and we can try sending again later.
          if(sc != SL_STATUS_OK) {
              app_log("Vendor model send error: 0x%04X\r\n", sc);
          } else {
              app_log("Acknowledge sent.\r\n");
              sl_btmesh_LCD_write("Acknowledge sent", 6);
          }
      }
      if(action_req & STATUS_UPDATE_REQ) {
          app_log("Publishing status update.\r\n");
          sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
                                                      my_model.vendor_id,
                                                      my_model.model_id,
                                                      opcode,
                                                      1,
                                                      payload_len,
                                                      payload_data);
          if(sc != SL_STATUS_OK) {
              app_log("Set publication error: 0x%04X\r\n", sc);
          } else {
              app_log("Set publication done. Publishing ...\r\n");
              sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
                                                  my_model.vendor_id,
                                                  my_model.model_id);
              if(sc != SL_STATUS_OK) {
                  app_log("Publish error: 0x%04X\r\n", sc);
              } else {
                  app_log("Publish done.\r\n");
              }
          }
      }
      break;
    }

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

//data tx
//static void read_datatx(void)
//{
//  *(int32_t *) datatx = 0x18;
//}

/// Temperature
//static void send_rssi(void)
//{
//  *(int32_t *) datatx = xx;
//
//    app_log("RSSI: %d\r\n", (int)datatx);
//}
static void read_temperature(void)
{
//  uint32_t rel_hum;
//  float temp;
//  //  int32_t temperature_int = 0;
//
//  sl_status_t sc;
//  sl_sensor_rht_init();
//
//  if(sl_sensor_rht_get(&rel_hum, (int32_t *)temperature) != SL_STATUS_OK) {
//      app_log("Error while reading temperature sensor. Clear the buffer.\r\n");
//      memset(temperature, 0, sizeof(temperature));
//  }

  ///

  //  app_log("%d\r\n", (int)*temperature);

  //  // Measure temperature; units are % and milli-Celsius.
  //  data_send[0] = (((int32_t)*temperature/10) & 0x00FF);
  //    data_send[1] = ( ((int32_t)*temperature/10) & 0xFF00) >> 8;
  //    adv_data.temp[0] = data_send[0];
  //    adv_data.temp[1] = data_send[1];
  //    // Set data for advertise packet.
  //    sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle, sl_bt_advertiser_advertising_data_packet , sizeof(adv_data), (uint8_t*)(&adv_data));
  //    app_assert_status(sc);
  //    // Start advertising and enable connections.
  //    sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
  //                                       sl_bt_legacy_advertiser_non_connectable);
  //    app_assert_status(sc);
//  int t = 0x10;
//  if (unit[0] == fahrenheit) {
////      temp = (float) (*(int32_t *) temperature / 1000);
////      temp = temp * 1.8 + 32;
//      *(int32_t *) temperature = (int32_t) t;
//  }
 //*(int32_t *) temperature = temmm;

  app_log("Data sent\r\n", (int)temperature);
}
/// Reset
static void app_reset_timer_cb(sl_simple_timer_t *handle, void *data)
{
  (void)handle;
  (void)data;
  sl_bt_system_reset(0);
}

static sl_simple_timer_t app_reset_timer;
static void delay_reset_ms(uint32_t ms)
{
  if(ms < 10) {
      ms = 10;
  }
  sl_simple_timer_start(&app_reset_timer,
                        ms,
                        app_reset_timer_cb,
                        NULL,
                        false);

}


/// Update Interval
static void periodic_update_timer_cb(sl_simple_timer_t *handle, void *data)
{
  (void)handle;
  (void)data;
  sl_status_t sc;

  app_log("New temperature update\r\n");
  read_temperature();
  sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
                                              my_model.vendor_id,
                                              my_model.model_id,
                                              temperature_status,
                                              1,
                                              TEMP_DATA_LENGTH,
                                              temperature);


  if(sc != SL_STATUS_OK) {
      app_log("Set publication error: 0x%04X\r\n", sc);
  } else {
      app_log("Set publication done. Publishing...\r\n");
      sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
                                          my_model.vendor_id,
                                          my_model.model_id);
      if (sc != SL_STATUS_OK) {
          app_log("Publish error: 0x%04X\r\n", sc);
      } else {
          app_log("Publish done.\r\n");
      }
  }
}

static void parse_period(uint8_t interval)
{
  switch (interval & STEP_RES_BIT_MASK) {
    case STEP_RES_100_MILLI:
      periodic_timer_ms = 100 * (interval & (~STEP_RES_BIT_MASK));
      break;
    case STEP_RES_1_SEC:
      periodic_timer_ms = 1000 * (interval & (~STEP_RES_BIT_MASK));
      break;
    case STEP_RES_10_SEC:
      periodic_timer_ms = 10000 * (interval & (~STEP_RES_BIT_MASK));
      break;
    case STEP_RES_10_MIN:
      // 10 min = 600000ms
      periodic_timer_ms = 600000 * (interval & (~STEP_RES_BIT_MASK));
      break;
    default:
      break;
  }
}

static sl_simple_timer_t periodic_update_timer;
static void setup_periodcal_update(uint8_t interval)
{
  parse_period(interval);
  sl_simple_timer_start(&periodic_update_timer,
                        periodic_timer_ms,
                        periodic_update_timer_cb,
                        NULL,
                        true);
}

//static void timer_cb(sl_simple_timer_t *timer, void *data){
//  (void) timer;
//  (void) data;
//  time_elapsed++;
//}

//static void send_message(void){
//  sl_status_t sc = SL_STATUS_OK;
//  app_log("---------------------------\r\n");
//  app_log("loop = %u\r\n", loop);
//
//
//  if (bytesLeft >= TEST_LENGTH) {
//      payload_len = TEST_LENGTH;
//      bytesLeft -= TEST_LENGTH;
//  }
//  else {
//      payload_len = bytesLeft;
//      bytesLeft = 0;
//  }
//  app_log("bytesLeft = %u\r\n", bytesLeft);
//  memcpy(tempArr, myArr + (loop*TEST_LENGTH), payload_len);
//  payload = tempArr;
//  uint8_t load = 0;
//  uint8_t load_1 = 1;
//  if(bytesLeft == 0){
//      app_log("THE END\r\n");
//        sc = sl_btmesh_vendor_model_set_publication_tracked(ELEM_INDEX,
//                                                              VENDOR_ID,
//                                                              MODEL_ID,
//                                                              0,
//                                                              opcodes[0],
//                                                              0,
//                                                              1,
//                                                              (const uint8_t*) &load_1,
//                                                              &publish_handle);
//          app_assert_status(sc);
//
//    }else {
//        sc = sl_btmesh_vendor_model_set_publication_tracked(ELEM_INDEX,
//                                                                      VENDOR_ID,
//                                                                      MODEL_ID,
//                                                                      0,
//                                                                      opcodes[0],
//                                                                      0,
//                                                                      1,
//                                                                      (const uint8_t*) &load,
//                                                                      &publish_handle);
//    }
//  sc = sl_btmesh_vendor_model_set_publication_tracked(ELEM_INDEX,
//                                                      VENDOR_ID,
//                                                      MODEL_ID,
//                                                      0,
//                                                      opcodes[0],
//                                                      0,
//                                                      2,
//                                                      (const uint8_t*) &loop,
//                                                      &publish_handle);
//  app_assert_status(sc);
//  sc = sl_btmesh_vendor_model_set_publication_tracked(ELEM_INDEX,
//                                                      VENDOR_ID,
//                                                      MODEL_ID,
//                                                      0,
//                                                      opcodes[0],
//                                                      1,
//                                                      payload_len,
//                                                      payload,
//                                                      &publish_handle);
//  app_assert_status(sc);
//
//
//  sc = sl_btmesh_vendor_model_publish(ELEM_INDEX,
//                                      VENDOR_ID,
//                                      MODEL_ID);
////  app_log("Content sent: \r\t",payload);
//
//  app_assert_status(sc);
//  if(sc != SL_STATUS_OK) {
//      app_log("Vendor model send error: 0x%04X\r\n", sc);
//  }else if (bytesLeft == 0) {
//      loop = 0;
//      app_log("Time = %u ms\r\n", time_elapsed*10);
//      sl_simple_timer_stop(&timer);
//      send_started = false;
//      app_log("Acknowledge sent.\r\n");
//      sl_btmesh_LCD_write("Acknowledge sent", 6);
//  }
//}

