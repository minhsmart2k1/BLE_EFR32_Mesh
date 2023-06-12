/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
 ******************************************************************************/
#include <stdio.h>
#include "em_common.h"
#include "app_assert.h"
#include "sl_status.h"
#include "sl_bluetooth.h"
#include "app.h"

#include "sl_simple_button_instances.h"

#include "sl_btmesh_api.h"
#include "sl_bt_api.h"
#include "app_log.h"
#include "sl_btmesh_factory_reset.h"
#include "sl_btmesh_wstk_lcd.h"
#include "gatt_db.h"
#include "sl_simple_timer.h"
#include "app_i2c.h"
#include "app_log.h"
//#include "my_model_def.h"



// Group Addresses
// Choose any 16-bit address starting at 0xC000
#define CUSTOM_STATUS_GRP_ADDR                      0xC001  // Server PUB address
#define CUSTOM_CTRL_GRP_ADDR                        0xC002  // Server SUB address

//#define MY_MODEL_SERVER_ID              0x1111
#define MY_MODEL_CLIENT_ID              0x2222

#define EX_B0_PRESS                                 ((1) << 5)
#define EX_B1_PRESS                                 ((1) << 6)
#define EX_B1_LONG_PRESS                            ((1) << 8)

/// Length of the display name buffer
#define NAME_BUF_LEN                   20

/// Length temperature
#define TEM_BUF_LEN                   20
#define TEN_BUF_LEN                   20
#define ACK_REQ                         (0x1)
#define STATUS_UPDATE_REQ               (0x2)
#define TEMP_DATA_LENGTH                4
#define TEMPP_DATA_LENGTH                4

static uint16_t network_index = 0x0000;
static uint16_t appkey_index = 0x0000;
static bool full_reset = false;
static uint8_t update_interval = 0;

// Button state.
static bool btn0_report_flag = false;
static bool btn1_report_flag = false;
static volatile bool btn0_pressed = false;
static volatile bool btn1_pressed = false;

uint8_t value[7];

//static uint32_t temperature = 0;


/// Used button indexes
#define BUTTON_PRESS_BUTTON_0          0
#define BUTTON_PRESS_BUTTON_1          1

/// Advertising Provisioning Bearer
#define PB_ADV                         0x1
/// GATT Provisioning Bearer
#define PB_GATT                        0x2

#define NUMBER_OF_OPCODES               11
#define PRIMARY_ELEMENT                 0
#define MY_VENDOR_ID                    0x1234
//
//#define MY_MODEL_SERVER_ID              0x1111
//#define MY_MODEL_CLIENT_ID              0x2222

#define UNIT_DATA_LENGTH                1
#define UPDATE_INTERVAL_LENGTH          1
#define UNIT_DATA_LENGTH                1
uint8_t conn_handle = 0xFF;


// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static uint32_t periodic_timer_ms = 0;


//static uint8_t b0_state = 0;
//static uint8_t b1_state = 0;
//static uint32_t b1_counter = 0;
//static uint32_t b0_counter = 0;

//static uint8_t period_idx = 0;
//static uint8_t periods[] = {
//    SET_100_MILLI(3),        /* 300ms */
//    0,
//    SET_100_MILLI(20),       /* 2s    */
//    0,
//    SET_1_SEC(10),           /* 10s   */
//    0,
//    SET_10_SEC(12),          /* 2min  */
//    0,
//    SET_10_MIN(1),           /* 10min */
//    0
//};

// Timing
// Check section 4.2.2.2 of Mesh Profile Specification 1.0 for format
#define STEP_RES_100_MILLI                          0
#define STEP_RES_1_SEC                              ((1) << 6)
#define STEP_RES_10_SEC                             ((2) << 6)
#define STEP_RES_10_MIN                             ((3) << 6)

#define STEP_RES_BIT_MASK                           0xC0

// Max x is 63
#define SET_100_MILLI(x)                            (uint8_t)(STEP_RES_100_MILLI | ((x) & (0x3F)))
#define SET_1_SEC(x)                                (uint8_t)(STEP_RES_1_SEC | ((x) & (0x3F)))
#define SET_10_SEC(x)                               (uint8_t)(STEP_RES_10_SEC | ((x) & (0x3F)))
#define SET_10_MIN(x)                               (uint8_t)(STEP_RES_10_MIN | ((x) & (0x3F)))


static const char str[6] = "Sensor";
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
  update_interval_status
  //test

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

typedef struct {
  uint8_t mandata_ten;   // Length of the Manufacturer Data field.
  uint8_t mandata_type;  // Type of the Manufacturer Data field.
  uint8_t temp_uuid[2];  // Temperature UUID.
  uint8_t temp[2];       // Temperature in 2 byte, 0.01 precision.
  uint8_t compname_len;  // Complete name length.
  uint8_t compname_type; // Complete name type.
  uint8_t compname[6];   // Complete name = Sensor
} adv_data_t;


my_model_t my_model = {
    .elem_index = PRIMARY_ELEMENT,
    .vendor_id = MY_VENDOR_ID,
    .model_id = MY_MODEL_CLIENT_ID,
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




static adv_data_t adv_data = {
    // Manufacturer specific data.
    5,   // Length of field.
    0xFF, // Type of field.

    {0x6e, 0x2a}, // Temperature UUID.
    {0, 0},       // Temperature default value = 0.

    7,
    0x09,
    {str[0], str[1], str[2], str[3], str[4], str[5]}
};



static void delay_reset_ms(uint32_t ms);
static void parse_period(uint8_t interval);


#ifdef PROV_LOCALLY
static uint16_t uni_addr = 0;

static aes_key_128 enc_key = {
    .data = "\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03"
};
#endif


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  app_log("======================\r\n");
  app_log("Client Device\r\n");
  app_log("Client da san sang \r\n");

//  initCMU();
//  app_log("CMU Done\r\n");
//  initI2C();
//  app_log("i2c Done\r\n");

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  if (sl_simple_button_get_state(&sl_button_btn0) == SL_SIMPLE_BUTTON_PRESSED){
      sl_btmesh_initiate_full_reset();
      app_log("Full reset\r\n");
      full_reset = true;
      app_log("Node reset \r\n");
      sl_btmesh_LCD_write("Full reset client", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);

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
      }
      // Button 1 released.
      else if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED) {
          btn1_pressed = false;
      }
      btn1_report_flag = true;
  }
  // Button 0 state changed
  else if (handle == SL_SIMPLE_BUTTON_INSTANCE(0)) {
      // Button 0 pressed.
      if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
          btn0_pressed = true;
          sl_bt_external_signal(EX_B0_PRESS);
      }
      // Button 0  released.
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
           "vendor client %02x%02x",
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
static unit_t unit = celsius;
void sl_bt_on_event(struct sl_bt_msg *evt)
{
  sl_status_t sc;
  bd_addr addr;
  bd_addr *remote_address;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      app_log("The program has been updated\r\n");
      sl_bt_system_get_identity_address(&addr, NULL);
      app_log("Addr : %02X %02X %02X %02X %02X %02X\r\n", addr.addr[5],addr.addr[4],addr.addr[3],addr.addr[2],addr.addr[1],addr.addr[0]);
      // Initialize Mesh stack in Node operation mode,
      // wait for initialized event
      app_log("Node init\r\n");
      sc = sl_btmesh_node_init();
      app_assert_status_f(sc, "Failed to init node\n");
      // periodic scanner setting
                  sl_bt_scanner_set_timing(gap_1m_phy, 2000, 2000);
                  sl_bt_scanner_set_mode(gap_1m_phy, 0);
                  sl_bt_scanner_start(gap_1m_phy, scanner_discover_observation);
      break;
      ///////////////////////////////////////////////////////////////////////////
      // Add additional event handlers here as your application requires!      //
      ///////////////////////////////////////////////////////////////////////////


      // Handle Button Presses
    case sl_bt_evt_connection_opened_id:
             sl_bt_scanner_stop();
             break;
             // This event indicates that a connection was closed.
           case sl_bt_evt_connection_closed_id:
             sl_bt_scanner_start(gap_1m_phy, scanner_discover_observation);
             break;
             // scan response
             /*
                          case sl_bt_evt_scanner_scan_report_id:
                            //print_scan_resp(&evt->data.evt_scanner_scan_report);

                            if(evt->data.evt_scanner_scan_report.data.len)
                              {
                                //if(evt->data.evt_scanner_scan_report.data.data[3] == 1 && evt->data.evt_scanner_scan_report.data.data[4] == 2 && evt->data.evt_scanner_scan_report.data.data[5] == 0 && evt->data.evt_scanner_scan_report.data.data[6] == 1){
                                //              sl_simple_timer_start(&scan_timer,
                                //                                    10*1000,
                                //                                    NULL,
                                //                                    NULL,
                                //                                    false);
                                //              app_log_info("\r\n");
                                //              app_log_info("      DEVICE\r\n");
                                //              remote_address = &(evt->data.evt_scanner_scan_report.address);
                                //              //      if (evt->data.evt_scanner_scan_report.data = 0x0101) {
                                //              app_log_info("ADV address %02X %02X %02X %02X %02X %02X\r\n",
                                //                           remote_address->addr[5],
                                //                           remote_address->addr[4],
                                //                           remote_address->addr[3],
                                //                           remote_address->addr[2],
                                //                           remote_address->addr[1],
                                //                           remote_address->addr[0]);
                                //              char buf[NAME_BUF_LEN];
                                //              snprintf(buf, NAME_BUF_LEN, "adv %02X:%02X:%02X:%02X:%02X:%02X",
                                //                       remote_address->addr[5],
                                //                       remote_address->addr[4],
                                //                       remote_address->addr[3],
                                //                       remote_address->addr[2],
                                //                       remote_address->addr[1],
                                //                       remote_address->addr[0]);
                                //              sl_btmesh_LCD_write(buf, 4);
                                //              app_log_info("RSSI %d\r\n", evt->data.evt_scanner_scan_report.rssi);
                                //              app_log_info("data %d\r\n", evt->data.evt_scanner_scan_report.data);
                                //for(int i = 0; i < evt->data.evt_scanner_scan_report.data.len; i++){
                                //app_log_info("id%d = 0x0%x\r\n", i, evt->data.evt_scanner_scan_report.data.data[i]);
                                if((evt->data.evt_scanner_scan_report.data.data[7] ==0x01) &&(evt->data.evt_scanner_scan_report.data.data[8] ==0x02))
                                  {
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
                                    app_log_info("data %d\r\n", evt->data.evt_scanner_scan_report.data);
                                    app_log_info("\r\----------------------------------\r\n");
                                    app_log_info("id = 0x0%x\r\n", evt->data.evt_scanner_scan_report.data.data[9]);
                                    app_log_info("id = 0x0%x\r\n", evt->data.evt_scanner_scan_report.data.data[10]);
                                    app_log_info("\r\----------------------------------\r\n");
                                  }
                              }

                            break;
*/
//    case sl_bt_evt_system_external_signal_id: {
//      uint8_t opcode = 0, length = 0, data = 0;
//      if(evt->data.evt_system_external_signal.extsignals & EX_B0_PRESS) {
//          opcode = temperature_get;
//          app_log("PB0 Pressed.\r\n");
//      }
//      // check if external signal triggered by button 1 press
////      if(evt->data.evt_system_external_signal.extsignals & EX_B1_PRESS) {
////          opcode = unit_get;
////          app_log("PB1 Pressed.\r\n");
////      }
////      if(evt->data.evt_system_external_signal.extsignals & EX_B1_LONG_PRESS) {
////          if (unit == celsius) {
////              opcode = unit_set_unack;
////              length = 1;
////              data = fahrenheit;
////          } else {
////              opcode = unit_set;
////              length = 1;
////              data = celsius;
////          }
////          app_log("B1 Long Pressed.\r\n");
////      }
//      if(evt->data.evt_system_external_signal.extsignals & 1 << 2) {
//          opcode = temperature_get;
//          app_log("Response\r\n");
//      }
////      if(evt->data.evt_system_external_signal.extsignals & 1 << 2) {
////                opcode = unit_get;
////                app_log("Response\r\n");
////            }
//      // set the vendor model publication message
//      sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
//                                                  my_model.vendor_id,
//                                                  my_model.model_id,
//                                                  opcode,
//                                                  1,
//                                                  length,
//                                                  &data);
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
//    }
//
//
//    break;


    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

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
      if (!sc && pub_address == CUSTOM_CTRL_GRP_ADDR) {
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
                                                  CUSTOM_CTRL_GRP_ADDR,
                                                  DEFAULT_TTL,
                                                  0, 0, 0);
          app_assert_status_f(sc, "Failed to set local model pub\r\n");

          app_log("Add local model sub ...\r\n");
          sc = sl_btmesh_test_add_local_model_sub(my_model.elem_index,
                                                  my_model.vendor_id,
                                                  my_model.model_id,
                                                  CUSTOM_STATUS_GRP_ADDR);
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



      ///////////////////////////////////////////////////////////////////////////
      // Add additional event handlers here as your application requires!      //
      ///////////////////////////////////////////////////////////////////////////

      // Provisioning Events
    case sl_btmesh_evt_node_provisioned_id:
      app_log("Provisioning done.\r\n");
      sl_btmesh_LCD_write("Provisioning done.", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
      break;

    case sl_btmesh_evt_node_provisioning_failed_id:
      app_log("Provisioning failed. Result = 0x%04x\r\n",
              evt->data.evt_node_provisioning_failed.result);
      sl_btmesh_LCD_write("Provisioning failed.", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);

      break;

    case sl_btmesh_evt_node_provisioning_started_id:
      app_log("Provisioning started.\r\n");
      sl_btmesh_LCD_write("Provisioning started", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
      break;

    case sl_btmesh_evt_node_key_added_id:
      app_log("got new %s key with index %x\r\n",
              evt->data.evt_node_key_added.type == 0 ? "network " : "application ",
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


      //---------------------------------
      // Handle vendor model messages
    case sl_btmesh_evt_vendor_model_receive_id:

      {
        char temp[TEM_BUF_LEN];
        char tempp[TEN_BUF_LEN];
        int32_t temperature = 0;
        int32_t datatx = 0;
        float time = 0;
        sl_btmesh_evt_vendor_model_receive_t *rx_evt = (sl_btmesh_evt_vendor_model_receive_t *)&evt->data.evt_vendor_model_receive;
        uint8_t action_req = 0, opcode = 0, payload_len = 0, *payload_data = NULL;
        app_log("Vendor model data received.**********************\r\n\t"
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
            "Id = 0x%02X "
            "0x%02X \r\n\t"
            "Heart rate = %d \r\n\t"
            "SpO2 = %.2f \r\n\t"
            "Temp = %.2f \r\n\t",

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
            rx_evt->payload.len,
           evt->data.evt_vendor_model_receive.payload.data[8],
           evt->data.evt_vendor_model_receive.payload.data[9],
           evt->data.evt_vendor_model_receive.payload.data[0],
           (float)((evt->data.evt_vendor_model_receive.payload.data[7] << 24 |
               evt->data.evt_vendor_model_receive.payload.data[6] << 16 |
               evt->data.evt_vendor_model_receive.payload.data[5] << 8 |
               evt->data.evt_vendor_model_receive.payload.data[4]) / 100.0),
           (float)((evt->data.evt_vendor_model_receive.payload.data[11] << 8 |
               evt->data.evt_vendor_model_receive.payload.data[10]) / 100.0));

        value[0] = evt->data.evt_vendor_model_receive.payload.data[0];
        value[1] = evt->data.evt_vendor_model_receive.payload.data[7];
        value[2] = evt->data.evt_vendor_model_receive.payload.data[6];
        value[3] = evt->data.evt_vendor_model_receive.payload.data[5];
        value[4] = evt->data.evt_vendor_model_receive.payload.data[4];
        value[5] = evt->data.evt_vendor_model_receive.payload.data[11];
        value[6] = evt->data.evt_vendor_model_receive.payload.data[10];
        for(int i = 0; i < 7; i++){
            app_log("Value %d: %x\n\r", i,value[i]);
        }
        for(int j = 0; j < 7; j++){
              if(writeRegister8_value(0x55, 2, value[j]) == 0)
                {
                  app_log("TX: 0x%x\n\r", value[j]);
                }
              else app_log("TX: 0x%x false\n\r", value[j]);
              sl_sleeptimer_delay_millisecond(1000);
        }
                app_log("Done\n\r");

//        for(int i = 0; i < evt->data.evt_vendor_model_receive.payload.len; i++) {
//            app_log("data: %d\n\r", evt->data.evt_vendor_model_receive.payload.data[i]);
//        }


        app_log("\r\n");


        switch (rx_evt->opcode ) {
//          case temperature_status:
//            temperature = *(uint32_t *) rx_evt->payload.data;
//            app_log("Data = %x\r\n",
//                    temperature);
//            snprintf(temp,
//                     20,
//                     "data = %x",
//                     temperature);
//            sl_btmesh_LCD_write(temp, 6);
//            break;

            // Measure temperature; units are % and milli-Celsius.
//          case temperature_get:
//            app_log("Sending/publishing temperature status as response to "
//                "temperature get from client...\r\n");
////                        read_temperature();
//
//            action_req = ACK_REQ;
//            opcode = temperature_status;
//            payload_len = TEMP_DATA_LENGTH;
//            payload_data = temperature;
//
//            break;
//          case unit_get:
//            app_log("Sending/publishing unit status as response to unit get from "
//                "client...\r\n");
//            action_req = ACK_REQ;
//            opcode = unit_status;
//            payload_len = TEMPP_DATA_LENGTH;
//            payload_data = datatx;
//            break;
            // Client
//          case unit_set:
//            app_log("Sending/publishing unit status as response to unit set from "
//                "client...\r\n");
////                        memcpy(unit, rx_evt->payload.data, rx_evt->payload.len);
//            action_req = ACK_REQ | STATUS_UPDATE_REQ;
//            opcode = unit_status;
//            payload_len = UNIT_DATA_LENGTH;
//            payload_data = (uint8_t *) unit;
//            break;
//            // Client
//          case unit_set_unack:
//            app_log("Publishing unit status as response to unit set unacknowledged "
//                "from client...\r\n");
////                        memcpy(unit, rx_evt->payload.data, rx_evt->payload.len);
//            action_req = STATUS_UPDATE_REQ;
//            opcode = unit_status;
//            payload_len = UNIT_DATA_LENGTH;
//            payload_data = (uint8_t *) unit;
//            break;
//
          case update_interval_get:
            app_log("Publishing Update Interval status as response to Update "
                "interval get from client...\r\n");
            action_req = ACK_REQ;
            opcode = update_interval_status;
            payload_len = UPDATE_INTERVAL_LENGTH;
            payload_data = update_interval;
            break;
          case update_interval_set:
            app_log("Publishing Update Interval status as response to "
                "update_interval_set from client...\r\n");
            memcpy(update_interval,
                   rx_evt->payload.data,
                   rx_evt->payload.len);
            action_req = ACK_REQ | STATUS_UPDATE_REQ;
            opcode = update_interval_status;
            payload_len = UPDATE_INTERVAL_LENGTH;
            payload_data = update_interval;
//                        setup_periodcal_update(update_interval[0]);
            break;
//          case update_interval_set_unack:
//            app_log("Publishing Update Interval status as response to "
//                "update_interval_set_unack from client...\r\n");
//            memcpy(update_interval,
//                   rx_evt->payload.data,
//                   rx_evt->payload.len);
//            action_req = STATUS_UPDATE_REQ;
//            opcode = update_interval_status;
//            payload_len = UPDATE_INTERVAL_LENGTH;
//            payload_data = update_interval;
////            setup_periodcal_update(update_interval[0]);
//            break;



//          case unit_status:
////            unit = (unit_t) rx_evt->payload.data[0];
////            app_log("Unit = %s\r\n",
////                    unit == celsius ? (char * )"Celsius" : (char * )"Fahrenheit");
//
//            datatx = *(uint32_t *) rx_evt->payload.data;
//                        app_log("rssi = %d\r\n",
//                                datatx);
//                        snprintf(tempp,
//                                 20,
//                                 "rssi = %d",
//                                 datatx);
//                        sl_btmesh_LCD_write(tempp, 6);
//            break;
//          case update_interval_status:
//            update_interval = rx_evt->payload.data[0];
//            app_log("Period received = %d\r\n", update_interval);
//            parse_period(update_interval);
//            break;

          default:
            break;

        }
//        sl_bt_external_signal(1 << 2);

        //        uint8_t data = 0x01;
        //        // set the vendor model publication message
        //        sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
        //                                                    my_model.vendor_id,
        //                                                    my_model.model_id,
        //                                                    opcode,
        //                                                    1,
        //                                                    1,
        //                                                    &data);
        //        if(sc != SL_STATUS_OK) {
        //            app_log("Set publication error: 0x%04X\r\n", sc);
        //        } else {
        //            app_log("Set publication done. Publishing...\r\n");
        //            // publish the vendor model publication message
        //            sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
        //                                                my_model.vendor_id,
        //                                                my_model.model_id);
        //            if(sc != SL_STATUS_OK) {
        //                app_log("Publish error: 0x%04X\r\n", sc);
        //            } else {
        //                app_log("Publish done.\r\n");
        //
        //
        //
        //
        //            }
        //        }
        //        if(action_req & ACK_REQ) {
        //            sc = sl_btmesh_vendor_model_send(rx_evt->source_address,
        //                                             rx_evt->va_index,
        //                                             rx_evt->appkey_index,
        //                                             my_model.elem_index,
        //                                             my_model.vendor_id,
        //                                             my_model.model_id,
        //                                             rx_evt->nonrelayed,
        //                                             opcode,
        //                                             1,
        //                                             payload_len,
        //                                             payload_data);
        //            // Errors that are returned from this function are usually due to low
        //            // memory. Low memory is non-critical and we can try sending again later.
        //            if(sc != SL_STATUS_OK) {
        //                app_log("Vendor model send error: 0x%04X\r\n", sc);
        //            } else {
        //                app_log("Acknowledge sent.\r\n");
        //
        //
        //            }
        //        }

        break;

      }
      // -------------------------------
      // Default event handler.
    default:
      break;
  }


}

///reset
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
  if (periodic_timer_ms) {
      app_log("Update period [hh:mm:ss:ms]= %02d:%02d:%02d:%04d\r\n",
              (periodic_timer_ms / (1000 * 60 * 60)),
              (periodic_timer_ms % (1000 * 60 * 60)) / (1000 * 60),
              (periodic_timer_ms % (1000 * 60)) / 1000,
              ((periodic_timer_ms % (1000)) / 1000) * 100);
  } else {
      app_log("Periodic update off.\r\n");
  }
}



