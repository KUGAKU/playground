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

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
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
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

typedef struct {
  uint16_t connection_handle;
  uint16_t characteristic_handle;
} ble_stack_session_t;

typedef enum {
  ACP_SERVICE_EVT__CONNECT = 0,
  ACP_SERVICE_EVT__MTU_EXCHANGE,
  ACP_SERVICE_EVT__DISCONNECT,
  ACP_SERVICE_EVT__WRITE,
  ACP_SERVICE_EVT__TX_COMPLETE
} acp_service__evt_id_t;

typedef struct {
  uint8_t reserve;
} acp_service__gap_evt_t;

typedef struct {
  uint16_t client_rx_mtu;
} acp_service__gatts_evt_mtu_exchange_t;

typedef struct {
  uint16_t len;
  uint8_t* p_data;
} acp_service__gatts_evt_write_t;

typedef struct {
  union {
    acp_service__gatts_evt_mtu_exchange_t mtu_exchange;
    acp_service__gatts_evt_write_t write;
  } gatts_params;
} acp_service__gatts_evt_t;

typedef struct {
  uint16_t conn_handle;
  acp_service__evt_id_t evt_id;
  union {
    acp_service__gap_evt_t gap_evt;
    acp_service__gatts_evt_t gatts_evt;
  } params;
} acp_service__evt_t;

typedef void (*ble_stack__event_callback_t)(acp_service__evt_t* p_ble_evt,
                                            void* p_context);

static ble_stack_session_t s_ble_stack_session[SL_BT_CONFIG_MAX_CONNECTIONS];
static ble_stack__event_callback_t s_ble_stack__event_callback;

static void session_update_characteristic(uint16_t conn_handle,
                                          uint16_t new_characteristic_handle) {
  for (int i = 0; i < SL_BT_CONFIG_MAX_CONNECTIONS; i++) {
    if (s_ble_stack_session[i].connection_handle != conn_handle) {
      continue;
    }
    s_ble_stack_session[i].characteristic_handle = new_characteristic_handle;
    return;
  }
}

static void on_ble_gatt_status(sl_bt_evt_gatt_server_characteristic_status_t* p_evt) {
  session_update_characteristic(p_evt->connection, p_evt->characteristic);
  return;
}


static void on_ble_write(sl_bt_evt_gatt_server_attribute_value_t* p_evt) {
  acp_service__evt_t evt = {
      .conn_handle = p_evt->connection,
      .evt_id = ACP_SERVICE_EVT__WRITE,
      .params.gatts_evt.gatts_params.write.p_data = p_evt->value.data,
      .params.gatts_evt.gatts_params.write.len = p_evt->value.len,
  };
  s_ble_stack__event_callback(&evt, NULL);
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
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    case sl_bt_evt_connection_parameters_id:
    break;

    case sl_bt_evt_gatt_server_attribute_value_id:
      on_ble_write(&evt->data.evt_gatt_server_attribute_value);
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      on_ble_gatt_status(&evt->data.evt_gatt_server_characteristic_status);
    break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
