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
#include "app.h"

#include "sl_btmesh_api.h"
#include "sl_bt_api.h"
#include "sl_btmesh_factory_reset.h"
#include "sl_simple_button_instances.h"
#include "sl_btmesh_wstk_lcd.h"


//#define LIGHT_CTRL_GRP_ADDR   0xC001
//#define LIGHT_STATUS_GRP_ADDR 0xC002
//#define SERVER_MODEL_ID  0x1000 // Generic On/Off Server
//#define CLIENT_MODEL_ID  0x1001 // Generic On/Off Client

#define VENDOR_MODEL_ID 0x1113
#define VENDOR_ID 0x1234
#define VENDOR_GRP_APR  0xC003

#define MY_MODEL_SERVER_ID              0x1111
#define MY_MODEL_CLIENT_ID              0x2222

#define CUSTOM_STATUS_GRP_ADDR                      0xC001  // Server PUB address
#define CUSTOM_CTRL_GRP_ADDR                        0xC002  // Server SUB address



#define BLE_MESH_UUID_LEN_BYTE (16)
#define BLE_ADDR_LEN_BYTE (6)
#define MAX_NUM_BTMESH_DEV (4)
/// Advertising Provisioning Bearer
#define PB_ADV                         0x1
/// GATT Provisioning Bearer
#define PB_GATT                        0x2
/// Length of boot error message buffer
#define MSG_BUF_LEN           30
// Max number of SIG models in the DCD
#define MAX_SIG_MODELS    16
// Max number of vendor models in the DCD
#define MAX_VENDOR_MODELS 4

typedef struct{
  uint8_t address[BLE_ADDR_LEN_BYTE];
  uint8_t uuid[BLE_MESH_UUID_LEN_BYTE];
  uint8_t is_provisioned;
} device_table_entry_t;

typedef struct
{
  uint16_t model_id;
  uint16_t vendor_id;
} Model_t;

// This struct is used to help decoding the raw DCD data
typedef struct
{
  uint16_t companyID;
  uint16_t productID;
  uint16_t version;
  uint16_t replayCap;
  uint16_t featureBitmask;
  uint8_t payload[1];
} DCD_Header_t;

// This struct is used to help decoding the raw DCD data
typedef struct
{
  uint16_t location;
  uint8_t numSIGModels;
  uint8_t numVendorModels;
  uint8_t payload[1];
} DCD_Elem_t;

// Struct for storing the content of one element in the DCD
typedef struct
{
  uint16_t SIG_models[MAX_SIG_MODELS];
  uint8_t numSIGModels;

  Model_t vendor_models[MAX_VENDOR_MODELS];
  uint8_t numVendorModels;
} DCD_ElemContent_t;

typedef struct
{
  // model bindings to be done. for simplicity, all models are bound to same appkey in this example
  // (assuming there is exactly one appkey used and the same appkey is used for all model bindings)
  Model_t bind_model[4];
  uint8_t num_bind;
  uint8_t num_bind_done;

  // publish addresses for up to 4 models
  Model_t pub_model[4];
  uint16_t pub_address[4];
  uint8_t num_pub;
  uint8_t num_pub_done;

  // subscription addresses for up to 4 models
  Model_t sub_model[4];
  uint16_t sub_address[4];
  uint8_t num_sub;
  uint8_t num_sub_done;

} Config_t;

static const uint8_t fixed_netkey[16] = {0x23, 0x98, 0xdf, 0xa5, 0x09, 0x3e, 0x74, 0xbb, 0xc2, 0x45, 0x1f, 0xae, 0xea, 0xd7, 0x67, 0xcd};
static const uint8_t fixed_appkey[16] = {0x16, 0x39, 0x38, 0x03, 0x9b, 0x8d, 0x8a, 0x20, 0x81, 0x60, 0xa7, 0x93, 0x33, 0x3d, 0x03, 0x61};
/* DCD receive */
static uint8_t _dcd_raw[256] = {0}; // raw content of the DCD received from remote node
static uint8_t _dcd_raw_len = 0;
static DCD_ElemContent_t _DCD_Primary;
static DCD_ElemContent_t _DCD_Secondary; // Not used, just for informative
static Config_t _Config;// configuration data to be sent to last provisioned node
static bd_addr dev_address = {{0}};
static uuid_128 uuid = {{0}};
static uint16_t network_index = 0x0000;
static uint16_t appkey_index = 0x0000;
static uint16_t provisionee_addr = 0x0000;
static uint16_t sub_address;
static bool full_reset = false;
static device_table_entry_t bluetooth_device_table[MAX_NUM_BTMESH_DEV];
static uint32_t handle = 0xFFFFFFFFu;
static bool device_found = false;
static bool scanning = false;
// Button state.
static bool btn0_report_flag = false;
static bool btn1_report_flag = false;
static volatile bool btn0_pressed = false;
static volatile bool btn1_pressed = false;
static uint8_t dev_num = 0;


static void config_bind_add(uint16_t, uint16_t);
static void config_sub_add(uint16_t, uint16_t, uint16_t);
static void config_pub_add(uint16_t, uint16_t, uint16_t);
static int8_t IsDevPresent(const uint8_t * addr);
static void ConfigCheck(void);
static void DCD_Decode(void);
static void DCD_Decode_element(DCD_Elem_t *pElem, DCD_ElemContent_t *pDest);


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{


  app_log("Application starting - Provisioner\r\n");
  if (sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(0)) == SL_SIMPLE_BUTTON_PRESSED) {
      // Button 0 pressed on startup, delete bondings.
      sl_btmesh_initiate_full_reset();
      app_log("Full reset\r\n");
      sl_btmesh_LCD_write("full reset", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
      full_reset = true;
      /* Reset working structure */
      memset(&bluetooth_device_table,0x00, (sizeof(device_table_entry_t) * MAX_NUM_BTMESH_DEV));
  }
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  sl_status_t sc = SL_STATUS_OK;
  if (btn0_report_flag) {
      if (btn0_pressed) {
          if (device_found) {
              // Accept
              sc = sl_btmesh_prov_create_provisioning_session(network_index,
                                                              uuid,
                                                              0);
              app_assert_status(sc);
              sc = sl_btmesh_prov_provision_adv_device(uuid);
              app_assert_status(sc);
              device_found = false;
              memcpy(&bluetooth_device_table[dev_num].address[0], &dev_address, BLE_ADDR_LEN_BYTE);
              memcpy(&bluetooth_device_table[dev_num].uuid[0], &uuid, BLE_MESH_UUID_LEN_BYTE);
              dev_num++;
              memset(&dev_address, 0, sizeof(dev_address));
              memset(&uuid, 0, sizeof(uuid));
          }
          else if (!scanning) {
              app_log("Scanning for unprov beacons\r\n");
              sl_btmesh_LCD_write("scanning", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
              sc = sl_btmesh_prov_scan_unprov_beacons();
              app_assert_status(sc);
              scanning = true;
          }
          sl_btmesh_LCD_write("", 8);
      }

      btn0_report_flag = false;
  }
  if (btn1_report_flag) {
      if (btn1_pressed) {
          if (device_found) {
              // Deny
              app_log("Scanning for unprov beacons\r\n");
              sc = sl_btmesh_prov_scan_unprov_beacons();
              app_assert_status(sc);
              scanning = true;
              sl_btmesh_LCD_write("scanning", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
              sl_btmesh_LCD_write("", 4);
              device_found = false;
              memset(&dev_address, 0, sizeof(dev_address));
              memset(&uuid, 0, sizeof(uuid));
          }
          else if (scanning) {
              app_log("Stop scanning\r\n");
              sl_btmesh_LCD_write("Stop scanning", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
              sc = sl_btmesh_prov_stop_scan_unprov_beacons();
              app_assert_status(sc);
              scanning = false;
          }
          sl_btmesh_LCD_write("", 8);
      }
      btn1_report_flag = false;
  }
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
void sl_bt_on_event(struct sl_bt_msg *evt)
{
  sl_status_t sc;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      if (full_reset)
        break;
      // Initialize Mesh stack in provision mode
      app_log("Provisioner init\r\n");
      sc = sl_btmesh_prov_init();
      app_assert_status(sc);
      break;
      ///////////////////////////////////////////////////////////////////////////
      // Add additional event handlers here as your application requires!      //
      ///////////////////////////////////////////////////////////////////////////

    case sl_bt_evt_connection_opened_id:
      sl_btmesh_LCD_write("Connected", SL_BTMESH_WSTK_LCD_ROW_CONNECTION_CFG_VAL);
      app_log("Connected\r\n");
      break;

    case sl_bt_evt_connection_closed_id:
      sl_btmesh_LCD_write("", SL_BTMESH_WSTK_LCD_ROW_CONNECTION_CFG_VAL);
      app_log("Disconnected\r\n");
      break;

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
  int8_t dev_idx = 0; /* device */
  uint16_t vendor_id = 0;
  uint16_t model_id = 0;
  uint16_t pub_address = 1;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_btmesh_evt_prov_initialized_id:
      {
        app_log("Number of network keys: %d\r\n", evt->data.evt_prov_initialized.networks);
        app_log("Provisioner's unicast address of the primary element: %d\r\n", evt->data.evt_prov_initialized.address);
        app_log("IV Index: %d\r\n", evt->data.evt_prov_initialized.iv_index);
        if (evt->data.evt_prov_initialized.networks == 0) {
            app_log("Creating network key\r\n");
            sc = sl_btmesh_prov_create_network(network_index,
                                               sizeof(fixed_netkey), // NetKey length
                                               fixed_netkey); // NetKey
            app_assert_status(sc);
            app_log("Creating application key\r\n");
            sc = sl_btmesh_prov_create_appkey(network_index, // NetKey index
                                              appkey_index, // AppKey index
                                              sizeof(fixed_appkey), // AppKey length
                                              fixed_appkey, // AppKey
                                              0, // Max AppKey buffer out size, does not used if the key is not generated
                                              NULL, // AppKey length out
                                              NULL); // AppKey out
            app_assert_status(sc);
        }
        else {
            app_log("%d NetKey already existed\r\n", evt->data.evt_prov_initialized.networks);
        }
        sl_btmesh_LCD_write("PROVISIONER", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
        break;
      }


      ///////////////////////////////////////////////////////////////////////////
      // Add additional event handlers here as your application requires!      //
      ///////////////////////////////////////////////////////////////////////////

    case sl_btmesh_evt_prov_initialization_failed_id:
      app_log("Provision init failed: %4X\r\n", evt->data.evt_prov_initialization_failed.result);
      sl_btmesh_LCD_write("init failed", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
      break;

    case sl_btmesh_evt_prov_provisioning_failed_id:
      app_log("Provision failed %d\r\n", evt->data.evt_prov_provisioning_failed.reason);
      sl_btmesh_LCD_write("prov failed...", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
      break;

    case  sl_btmesh_evt_prov_unprov_beacon_id:
      {
        // PB-ADV
        if (evt->data.evt_prov_unprov_beacon.bearer == 0x0) {
            app_log("Beacon found, device address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    evt->data.evt_prov_unprov_beacon.address.addr[5],
                    evt->data.evt_prov_unprov_beacon.address.addr[4],
                    evt->data.evt_prov_unprov_beacon.address.addr[3],
                    evt->data.evt_prov_unprov_beacon.address.addr[2],
                    evt->data.evt_prov_unprov_beacon.address.addr[1],
                    evt->data.evt_prov_unprov_beacon.address.addr[0]);
            app_log("UUID last 2 bytes: %02X:%02X\r\n",
                    evt->data.evt_prov_unprov_beacon.uuid.data[14],
                    evt->data.evt_prov_unprov_beacon.uuid.data[15]);
            dev_idx = IsDevPresent(evt->data.evt_prov_unprov_beacon.address.addr);
            if (dev_idx < 0) {
                // Device is not in device table
                if((0x00 == bluetooth_device_table[dev_num].address[0]) &&
                    (0x00 == bluetooth_device_table[dev_num].address[1]) &&
                    (0x00 == bluetooth_device_table[dev_num].address[2]) &&
                    (0x00 == bluetooth_device_table[dev_num].address[3]) &&
                    (0x00 == bluetooth_device_table[dev_num].address[4]) &&
                    (0x00 == bluetooth_device_table[dev_num].address[5])) {
                    app_log("New mesh device found\r\n");
                    dev_address = evt->data.evt_prov_unprov_beacon.address;
                    uuid = evt->data.evt_prov_unprov_beacon.uuid;
                    sc = sl_btmesh_prov_stop_scan_unprov_beacons();
                    app_assert_status(sc);
                    scanning = false;
                    device_found = true;
                    sl_btmesh_LCD_write("prov pending...", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
                    char buf[MSG_BUF_LEN];
                    snprintf(buf, MSG_BUF_LEN, "Dev UUID: %02X%02X",
                             evt->data.evt_prov_unprov_beacon.uuid.data[14],
                             evt->data.evt_prov_unprov_beacon.uuid.data[15]);
                    sl_btmesh_LCD_write(buf, 4);
                    sl_btmesh_LCD_write("NO               YES", 8);
                }
            }
        }
        break;
      }

    case sl_btmesh_evt_prov_device_provisioned_id:
      {
        app_log("Primary element address: %4X\r\n", evt->data.evt_prov_device_provisioned.address);
        app_log("UUID: %02X:%02X\r\n",
                evt->data.evt_prov_device_provisioned.uuid.data[14],
                evt->data.evt_prov_device_provisioned.uuid.data[15]);
        sl_btmesh_LCD_write("prov successful", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
        provisionee_addr = evt->data.evt_prov_device_provisioned.address;
        sc = sl_btmesh_config_client_get_dcd(network_index,
                                             provisionee_addr,
                                             0, // Page0
                                             &handle);
        app_assert_status(sc);
        break;
      }


    case sl_btmesh_evt_config_client_dcd_data_id:
      {
        const uint8_t *pData = evt->data.evt_config_client_dcd_data.data.data;
        uint8_t data_len = evt->data.evt_config_client_dcd_data.data.len;
        app_log("DCD data event, received %d bytes\r\n", data_len);
        sl_btmesh_LCD_write("DCD data event", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
        // Copy the data into one large array. the data may come in multiple smaller pieces.
        // the data is not decoded until all DCD events have been received (see below)
        if((_dcd_raw_len + data_len) <= 256)
          {
            memcpy(&(_dcd_raw[_dcd_raw_len]), pData, data_len);
            _dcd_raw_len += data_len;
          }
        break;
      }


    case sl_btmesh_evt_config_client_dcd_data_end_id:
      {
        app_log("DCD Data end event\r\n");
        sl_btmesh_LCD_write("DCD data event end", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
        DCD_Decode();
        ConfigCheck();
        memset(&_dcd_raw, 0, sizeof(_dcd_raw));
        _dcd_raw_len = 0;
        sc = sl_btmesh_config_client_add_appkey(network_index,
                                                provisionee_addr,
                                                appkey_index,
                                                network_index,
                                                &handle);
        app_assert_status(sc);
        break;
      }

    case sl_btmesh_evt_config_client_appkey_status_id:
      {
        app_log("AppKey operation status: 0x%04X\r\n", evt->data.evt_config_client_appkey_status.result);
        sl_btmesh_LCD_write("model binding", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
        // take the next model from the list of models to be bound with application key.
        // for simplicity, the same appkey is used for all models but it is possible to also use several appkeys
        model_id = _Config.bind_model[_Config.num_bind_done].model_id;
        vendor_id = _Config.bind_model[_Config.num_bind_done].vendor_id;
        app_log("Binding model: 0x%04X\r\n", model_id);
        sc = sl_btmesh_config_client_bind_model(network_index,
                                                provisionee_addr,
                                                0, // Main element
                                                vendor_id,
                                                model_id,
                                                appkey_index,
                                                &handle);
        app_assert_status(sc);
        break;
      }

    case sl_btmesh_evt_config_client_binding_status_id:
      {
        app_log("Model binding operation status: 0x%04X\r\n", evt->data.evt_config_client_binding_status.result);
        if (evt->data.evt_config_client_binding_status.result == 0) {
            _Config.num_bind_done++;
            if(_Config.num_bind_done < _Config.num_bind)
              {
                // take the next model from the list of models to be bound with application key.
                // for simplicity, the same appkey is used for all models but it is possible to also use several appkeys
                model_id = _Config.bind_model[_Config.num_bind_done].model_id;
                vendor_id = _Config.bind_model[_Config.num_bind_done].vendor_id;
                app_log("Binding model: 0x%04X (%d/%d)\r\n", model_id, _Config.num_bind_done+1, _Config.num_bind);
                sc = sl_btmesh_config_client_bind_model(network_index,
                                                        provisionee_addr,
                                                        0, // Main element
                                                        vendor_id,
                                                        model_id,
                                                        appkey_index,
                                                        &handle);
                app_assert_status(sc);
              }
            else {
                app_log("All models are binded\r\n");
                sl_btmesh_LCD_write("setting publication", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
                // get the next model/address pair from the configuration list:
                model_id = _Config.pub_model[_Config.num_pub_done].model_id;
                vendor_id = _Config.pub_model[_Config.num_pub_done].vendor_id;
                pub_address = _Config.pub_address[_Config.num_pub_done];
                app_log("Publish setting, model ID: 0x%04X -> 0x%04X (%d/%d)\r\n", model_id, pub_address, _Config.num_pub_done+1, _Config.num_pub);
                sc = sl_btmesh_config_client_set_model_pub(network_index,
                                                           provisionee_addr,
                                                           0, // Main element
                                                           vendor_id,
                                                           model_id,
                                                           pub_address,
                                                           appkey_index,
                                                           0, // Normal credential, if not 0 friendship is required
                                                           0, // Time-to-live
                                                           0, // Period
                                                           0, // Transmission count
                                                           50, // Retransmit interval, range from 50 to 1600, resolution 50 ms
                                                           &handle);
                app_assert_status(sc);
            }
        }
        break;
      }

    case sl_btmesh_evt_config_client_model_pub_status_id:
      {
        app_log("Publish setting operation status: 0x%04X\r\n", evt->data.evt_config_client_model_pub_status.result);
        if (evt->data.evt_config_client_model_pub_status.result == 0) {
            _Config.num_pub_done++;
            if (_Config.num_pub_done < _Config.num_pub) {
                /* more publication settings to be done
                 ** get the next model/address pair from the configuration list: */
                model_id = _Config.pub_model[_Config.num_pub_done].model_id;
                vendor_id = _Config.pub_model[_Config.num_pub_done].vendor_id;
                pub_address = _Config.pub_address[_Config.num_pub_done];

                app_log("Publish setting, model ID: 0x%04X -> 0x%04X (%d/%d)\r\n", model_id, pub_address, _Config.num_pub_done+1, _Config.num_pub);

                sc = sl_btmesh_config_client_set_model_pub(network_index,
                                                           provisionee_addr,
                                                           0, // Main element
                                                           vendor_id,
                                                           model_id,
                                                           pub_address,
                                                           appkey_index,
                                                           0, // Normal credential, if not 0 friendship is required
                                                           0, // Time-to-live
                                                           0, // Period
                                                           0, // Transmission count
                                                           50, // Retransmit interval, range from 50 to 1600, resolution 50 ms
                                                           &handle);
                app_assert_status(sc);
            }
            else {
                app_log("All publication setting are set\r\n");
                sl_btmesh_LCD_write("setting subscription", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
                // move to next step which is configuring subscription settings
                // get the next model/address pair from the configuration list:
                model_id = _Config.sub_model[_Config.num_sub_done].model_id;
                vendor_id = _Config.sub_model[_Config.num_sub_done].vendor_id;
                sub_address = _Config.sub_address[_Config.num_sub_done];

                app_log("Subscribe setting, model ID: 0x%04X -> 0x%04X (%d/%d)\r\n", model_id, sub_address, _Config.num_sub_done+1, _Config.num_sub);
                sc = sl_btmesh_config_client_add_model_sub(network_index,
                                                           provisionee_addr,
                                                           0, // Main element
                                                           vendor_id,
                                                           model_id,
                                                           sub_address,
                                                           &handle);
                app_assert_status(sc);
            }
        }
        break;
      }

    case sl_btmesh_evt_config_client_model_sub_status_id:
      {
        app_log("Subscribe setting operation status: 0x%04X\r\n", evt->data.evt_config_client_model_sub_status.result);
        if (evt->data.evt_config_client_model_sub_status.result == 0) {
            _Config.num_sub_done++;
            if (_Config.num_sub_done < _Config.num_sub) {
                // move to next step which is configuring subscription settings
                // get the next model/address pair from the configuration list:
                model_id = _Config.sub_model[_Config.num_sub_done].model_id;
                vendor_id = _Config.sub_model[_Config.num_sub_done].vendor_id;
                sub_address = _Config.sub_address[_Config.num_sub_done];
                app_log("Subscribe setting, model ID: 0x%04X -> 0x%04X (%d/%d)\r\n", model_id, sub_address, _Config.num_sub_done+1, _Config.num_sub);
                sc = sl_btmesh_config_client_add_model_sub(network_index,
                                                           provisionee_addr,
                                                           0, // Main element
                                                           vendor_id,
                                                           model_id,
                                                           sub_address,
                                                           &handle);
                app_assert_status(sc);
            }
            else {
                app_log("********\r\nConfiguration completed\r\n********\r\n");
                sl_btmesh_LCD_write("config done", SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
                sl_btmesh_LCD_write("", 4);
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
      }
      // Button 0  released.
      else if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED) {
          btn0_pressed = false;
      }
      btn0_report_flag = true;
  }
}



/*
 * Add one publication setting to the list of configurations to be done
 * */
static void config_pub_add(uint16_t model_id, uint16_t vendor_id, uint16_t address)
{
  _Config.pub_model[_Config.num_pub].model_id = model_id;
  _Config.pub_model[_Config.num_pub].vendor_id = vendor_id;
  _Config.pub_address[_Config.num_pub] = address;
  _Config.num_pub++;
}

/*
 * Add one subscription setting to the list of configurations to be done
 * */
static void config_sub_add(uint16_t model_id, uint16_t vendor_id, uint16_t address)
{
  _Config.sub_model[_Config.num_sub].model_id = model_id;
  _Config.sub_model[_Config.num_sub].vendor_id = vendor_id;
  _Config.sub_address[_Config.num_sub] = address;
  _Config.num_sub++;
}

/*
 * Add one appkey/model bind setting to the list of configurations to be done
 * */
static void config_bind_add(uint16_t model_id, uint16_t vendor_id)
{
  _Config.bind_model[_Config.num_bind].model_id = model_id;
  _Config.bind_model[_Config.num_bind].vendor_id = vendor_id;
  _Config.num_bind++;
}

static int8_t IsDevPresent(const uint8_t * restrict const addr)
{
  uint8_t idx;
  uint8_t *pdata;
  int8_t res_val;

  /* Initialize locals */
  res_val= -1; /* Pessimistic assumption, dev isn't present */
  pdata = NULL;

  for(idx=0; idx < MAX_NUM_BTMESH_DEV; idx++)
    {
      pdata=(uint8_t*)(&bluetooth_device_table[idx].address[0]);
      if((addr[0] == pdata[0]) &&
          (addr[1] == pdata[1]) &&
          (addr[2] == pdata[2]) &&
          (addr[3] == pdata[3]) &&
          (addr[4] == pdata[4]) &&
          (addr[5] == pdata[5])) {
          res_val = idx;
          break;
      }
    }

  return res_val;
}

void DCD_Decode(void)
{
  DCD_Header_t *pHeader;
  DCD_Elem_t *pElem;
  uint8_t byte_offset;

  pHeader = (DCD_Header_t *)&_dcd_raw;

  app_log("DCD: company ID 0x%4.4X, Product ID 0x%4.4X\r\n", pHeader->companyID, pHeader->productID);

  pElem = (DCD_Elem_t *)pHeader->payload;

  // decode primary element:
  DCD_Decode_element(pElem, &_DCD_Primary);

  // check if DCD has more than one element by calculating where we are currently at the raw
  // DCD array and compare against the total size of the raw DCD:
  byte_offset = 10 + 4 + pElem->numSIGModels * 2 + pElem->numVendorModels * 4; // +10 for DCD header, +4 for header in the DCD element

  if(byte_offset < _dcd_raw_len)
    {
      // set elem pointer to the beginning of 2nd element:
      pElem = (DCD_Elem_t *)&(_dcd_raw[byte_offset]);

      app_log("Decoding 2nd element (just informative, not used for anything)\r\n");
      DCD_Decode_element(pElem, &_DCD_Secondary);
    }
}

static void ConfigCheck()
{
  int i;

  memset(&_Config, 0, sizeof(_Config));

  // scan the SIG models in the DCD data
  //  for(i = 0; i < _DCD_Primary.numSIGModels; i++)
  //    {
  //      if(_DCD_Primary.SIG_models[i] == CLIENT_MODEL_ID)
  //        {
  //          app_log("Client model found\r\n");
  //          config_pub_add(CLIENT_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
  //          config_sub_add(CLIENT_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
  //          config_bind_add(CLIENT_MODEL_ID, 0xFFFF);
  //        }
  //      else if(_DCD_Primary.SIG_models[i] == SERVER_MODEL_ID)
  //        {
  //          app_log("Server model found\r\n");
  //          config_pub_add(SERVER_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
  //          config_sub_add(SERVER_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
  //          config_bind_add(SERVER_MODEL_ID, 0xFFFF);
  //        }
  //    else if(_DCD_Primary.SIG_models[i] == DIM_SWITCH_MODEL_ID)
  //    {
  //      config_pub_add(DIM_SWITCH_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
  //      config_sub_add(DIM_SWITCH_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
  //      config_bind_add(DIM_SWITCH_MODEL_ID, 0xFFFF, 0, 0);
  //    }
  //    else if(_DCD_Primary.SIG_models[i] == DIM_LIGHT_MODEL_ID)
  //    {
  //      config_pub_add(DIM_LIGHT_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
  //      config_sub_add(DIM_LIGHT_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
  //      config_bind_add(DIM_LIGHT_MODEL_ID, 0xFFFF, 0, 0);
  //    }
  //    }
  //}

  for (i = 0; i < _DCD_Primary.numVendorModels; i++ )
    {
//      if(_DCD_Primary.vendor_models[i].model_id == VENDOR_MODEL_ID){
//          app_log("vendor model found\r\n");
//          config_pub_add(VENDOR_MODEL_ID,VENDOR_ID,VENDOR_GRP_APR);
//          config_sub_add(VENDOR_MODEL_ID,VENDOR_ID,VENDOR_GRP_APR);
//          config_bind_add(VENDOR_MODEL_ID, VENDOR_ID);
//      }
      if(_DCD_Primary.vendor_models[i].model_id == MY_MODEL_CLIENT_ID)
        {
          app_log("Client model found\r\n");
          config_pub_add(MY_MODEL_CLIENT_ID, VENDOR_ID, CUSTOM_CTRL_GRP_ADDR );
          config_sub_add(MY_MODEL_CLIENT_ID, VENDOR_ID, CUSTOM_STATUS_GRP_ADDR  );
          config_bind_add(MY_MODEL_CLIENT_ID, VENDOR_ID);
        }
      if(_DCD_Primary.vendor_models[i].model_id == MY_MODEL_SERVER_ID)
        {
          app_log("Server model found\r\n");
          config_pub_add(MY_MODEL_SERVER_ID, VENDOR_ID, CUSTOM_STATUS_GRP_ADDR );
          config_sub_add(MY_MODEL_SERVER_ID, VENDOR_ID, CUSTOM_CTRL_GRP_ADDR  );
          config_bind_add(MY_MODEL_SERVER_ID, VENDOR_ID);
        }

    }
}

/* function for decoding one element inside the DCD. Parameters:
 *  pElem: pointer to the beginning of element in the raw DCD data
 *  pDest: pointer to a struct where the decoded values are written
 * */
static void DCD_Decode_element(DCD_Elem_t *pElem, DCD_ElemContent_t *pDest)
{
  uint16_t *pu16;
  int i;

  memset(pDest, 0, sizeof(*pDest));

  pDest->numSIGModels = pElem->numSIGModels;
  pDest->numVendorModels = pElem->numVendorModels;

  app_log("Num sig models: %d\r\n", pDest->numSIGModels);
  app_log("Num vendor models: %d\r\n", pDest->numVendorModels);

  if(pDest->numSIGModels > MAX_SIG_MODELS)
    {
      app_log("ERROR: number of SIG models in DCD exceeds MAX_SIG_MODELS (%u) limit!\r\n", MAX_SIG_MODELS);
      return;
    }
  if(pDest->numVendorModels > MAX_VENDOR_MODELS)
    {
      app_log("ERROR: number of VENDOR models in DCD exceeds MAX_VENDOR_MODELS (%u) limit!\r\n", MAX_VENDOR_MODELS);
      return;
    }

  // set pointer to the first model:
  pu16 = (uint16_t *)pElem->payload;

  // grab the SIG models from the DCD data
  for(i = 0; i<pDest->numSIGModels; i++)
    {
      pDest->SIG_models[i] = *pu16;
      pu16++;
      app_log("model ID: %4.4x\r\n", pDest->SIG_models[i]);
    }

  // grab the vendor models from the DCD data
  for (i = 0; i < pDest->numVendorModels; i++) {
      pDest->vendor_models[i].vendor_id = *pu16;
      pu16++;
      pDest->vendor_models[i].model_id = *pu16;
      pu16++;

      app_log("vendor ID: %4.4x, model ID: %4.4x\r\n", pDest->vendor_models[i].vendor_id, pDest->vendor_models[i].model_id);
  }
}
