/***************************************************************************//**
 *@brief Bluetooth dynamic GATT database configuration
 *******************************************************************************
 * # License
 * <b> Copyright 2021 Silicon Laboratories Inc.www.silabs.com </b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement(MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef SL_BT_DYNAMIC_GATTDB_CONFIG_H
#define SL_BT_DYNAMIC_GATTDB_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

// <h> Bluetooth Dynamic GATT Configuration

// <o SL_BT_GATTDB_INCLUDE_STATIC_DATABASE> Static GATT database configuration
// <i> Default: 1 (Enabled)
// <i>
// <i> Configures whether a static GATT database, which is generated from a
// <i> GATT XML file, should be included. When included, services and
// <i> characteristics created dynamically are appended into the attribute
// <i> table after the static attributes.
// <i>
// <i> Values:
// <i> * 1: Include the static GATT database.
// <i> * 0: Do not include the static GATT database.
// <i>
#define SL_BT_GATTDB_INCLUDE_STATIC_DATABASE     (1)

// <o SL_BT_GATTDB_ENABLE_GATT_CACHING> GATT Caching support configuration
// <i> Default: 1 (Enabled)
// <i>
// <i> Configures whether the database should support GATT caching. When enabled,
// <i> a Generic Attribute Profile Service will be created in the beginning of
// <i> the database, and the GATT server will handle service-changed indications
// <i> and robust caching with remote GATT clients automatically.
// <i>
// <i> Note that this configuration does not apply when a static database
// <i> is included. GATT caching support configuration must come from the
// <i> static database in this case.
// <i>
// <i> There could be cases that the database does not enable the GATT caching
// <i> feature while one or more GATT clients are connected, or the feature is
// <i> enabled but a connected remote GATT client does not enable GATT caching
// <i> (by enabling service-changed indications or indicating that it supports
// <i> robust caching). In these cases, the user application should not update
// <i> the database. Otherwise a GATT client and local GATT server become out
// <i> of sync, and there is no means for the GATT client to realize this
// <i> situation.
// <i>
// <i> Values:
// <i> * 1: Enable GATT Caching.
// <i> * 0: Disable GATT Caching.
// <i>
#define SL_BT_GATTDB_ENABLE_GATT_CACHING        (1)

// </h> End Bluetooth Dynamic GATT Configuration

// <<< end of configuration section >>>

/**
 * @brief Configure the Bluetooth Dynamic GATT feature.
 *
 * This function is automatically called by code generated by the Universal
 * Configurator if the feature is included in the project. The application is
 * not expected to call this function directly.
 */
sl_status_t sl_bt_dynamic_gattdb_configure();

#endif // SL_BT_DYNAMIC_GATTDB_CONFIG_H