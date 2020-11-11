/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/event-manager.h>
#include <zmk/events/position-state-changed.h>
#include <zmk/matrix_transform.h>
#include <init.h>

static int start_scan(void);

// TODO TODO TODO figure out how to have array of devices. ignore name matching
// if not using zmk peripheral.
char devices[][16] = {"ErgoBlue Left", "ErgoBlue Right"};
#define DEVICE_COUNT sizeof(devices)/sizeof(devices[0])

// Define array for holding peripheral connections.
static struct bt_conn *peripheral_conns[DEVICE_COUNT];

// TODO TODO TODO I believe we need one for each connection
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

// TODO TODO TODO service uuid, characteric uuid, easy to change using config.
static struct bt_uuid *service_uuid = BT_UUID_HIDS;
static struct bt_uuid *characteristic_uuid = BT_UUID_HIDS_REPORT;
static struct bt_uuid *ccc_uuid = BT_UUID_GATT_CCC;

// Track whether central is currently scanning for peripherals.
static bool is_scanning = false;

// Define array for state of peripheral keys. Each array value is initialized to
// 255 inside zmk_split_bt_central_init() as no key is pressed initially. The
// array value corresponds to the peripheral_conns index for pressed keys.
#define POSITION_STATE_DATA_LEN 16
static uint8_t position_state[8 * POSITION_STATE_DATA_LEN];

static void split_set_position(int device_id, u32_t position, bool isPressed) {
	bool wasPressed = position_state[position] != 255;

	// Determine new state for position. Ignore released key if it was pressed
	// on a different peripheral.
	if (isPressed) {
		position_state[position] = device_id;
	} else if (position_state[position] == device_id) {
		position_state[position] = 255;
	} else {
		return;
	}

	// Handle event if state changed.
	bool changed = wasPressed != isPressed;
	if (changed) {

		// TODO TODO TODO explain transform. we'll likely add transform
		// to the nordic sdk firmware so only difference will be
		// scanning detection code.
		position = zmk_matrix_transform_row_column_to_position(position / 14, position % 14);

		struct position_state_changed *pos_ev = new_position_state_changed();
		pos_ev->position = position;
		pos_ev->state = isPressed;
		pos_ev->timestamp = k_uptime_get();

		LOG_DBG("Trigger key position state change for %d", position);
		ZMK_EVENT_RAISE(pos_ev);
	}
}

static u8_t split_central_notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                                      const void *data, u16_t length) {
    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

	
	int device_id = -1;
	for (int i = 0; i < DEVICE_COUNT; i++) {
		if (peripheral_conns[i] == conn) {
			device_id = i;
		}
	}

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            u32_t position = (i * 8) + j;
			bool isPressed = ((u8_t *)data)[i] & BIT(j);
			split_set_position(device_id, position, isPressed);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static int split_central_subscribe(struct bt_conn *conn) {
    int err = bt_gatt_subscribe(conn, &subscribe_params);
    switch (err) {
    case -EALREADY:
        LOG_DBG("[ALREADY SUBSCRIBED]");
        break;
        // break;
        // bt_gatt_unsubscribe(conn, &subscribe_params);
        // return split_central_subscribe(conn);
    case 0:
        LOG_DBG("[SUBSCRIBED]");
        break;
    default:
        LOG_ERR("Subscribe failed (err %d)", err);
        break;
    }

    return 0;
}

static u8_t split_central_discovery_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                         struct bt_gatt_discover_params *params) {

	int err;

	if (!attr) {
		LOG_DBG("Discover complete");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

	// After discovering service, discover using characteristic.
	if (!bt_uuid_cmp(discover_params.uuid, service_uuid)) {
		discover_params.uuid = characteristic_uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			LOG_DBG("Discover failed (err %d)", err);
		}
		return BT_GATT_ITER_STOP;
	}

	// After discovering characteristic, discover using descriptor.
	if (!bt_uuid_cmp(discover_params.uuid, characteristic_uuid)) {
		discover_params.uuid = ccc_uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			LOG_DBG("Discover failed (err %d)", err);
		}
		return BT_GATT_ITER_STOP;
	}

	// After discovering descriptor, enable notification on descriptor.
	subscribe_params.notify = split_central_notify_func;
	subscribe_params.value = BT_GATT_CCC_NOTIFY;
	subscribe_params.ccc_handle = attr->handle;
	split_central_subscribe(conn);
	return BT_GATT_ITER_STOP;
}

static void split_central_process_connection(struct bt_conn *conn) {
    int err;

    LOG_DBG("Current security for connection: %d", bt_conn_get_security(conn));

    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        LOG_ERR("Failed to set security (reason %d)", err);
        return;
    }

	// TODO TODO TODO is this loop necessary?
	for (int i = 0; i < DEVICE_COUNT; i++) {
		// TODO TODO TODO need to check subscribe_params?
		if (conn == peripheral_conns[i] ) { // && !subscribe_params.value) {
			LOG_DBG("starting discover for %d", i);

			// Clear discovery parameters from previous run.
			(void)memset(&discover_params, 0, sizeof(discover_params));

			// Discover based on Bluetooth service.
			discover_params.uuid = service_uuid;
			discover_params.func = split_central_discovery_func;
			discover_params.start_handle = 0x0001;
			discover_params.end_handle = 0xffff;
			discover_params.type = BT_GATT_DISCOVER_PRIMARY;

			err = bt_gatt_discover(peripheral_conns[i], &discover_params);
			if (err) {
				LOG_ERR("Discover failed(err %d)", err);
				return;
			}
		}
	}

    struct bt_conn_info info;

    bt_conn_get_info(conn, &info);

    LOG_DBG("New connection params: Interval: %d, Latency: %d, PHY: %d", info.le.interval,
            info.le.latency, info.le.phy->rx_phy);
}

static bool split_central_eir_found(struct bt_data *data, void *user_data) {
	// Continue parsing advertisement data if data is not device name.
    if (data->type != BT_DATA_NAME_COMPLETE) {
		return true;
	}

	// TODO TODO TODO ideally use whitelist. also dosn't seem like this setting
	// is actually used? we should actually store something to prevent a
	// malicious peripheral from imitating the real peripheral.
	// zmk_ble_set_peripheral_addr(addr);

	// Check if device name matches the specified device names TODO TODO TODO do
	// this for ErgoBlue, use different logic for zmk peripheral
	int device_id = -1;
	for (int i = 0; i < DEVICE_COUNT; i++) {
		if (data->data_len == strlen(devices[i])) {
			if (!memcmp(devices[i], data->data, strlen(devices[i]))) {
				device_id = i;
   				LOG_DBG("[NAME MATCH for device %i]", i);
			}
		}
	}

	// Continue parsing if device name does not match.
	if (device_id == -1) {
		return true;
	}

	// Stop scanning so we can connect to the peripheral device.
    LOG_DBG("Stopping peripheral scanning");
	is_scanning = false;
	int err = bt_le_scan_stop();
	if (err) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
		return true;
	}

	// Create connection to peripheral with the given connection parameters.
	bt_addr_le_t *addr = user_data;
	struct bt_le_conn_param *param = BT_LE_CONN_PARAM(0x0006, 0x0006, 30, 400);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &peripheral_conns[device_id]);
	if (err) {
		LOG_ERR("Create conn failed (err %d) (create conn? 0x%04x)", err, BT_HCI_OP_LE_CREATE_CONN);
		return true;
	}

	err = bt_conn_le_phy_update(peripheral_conns[device_id], BT_CONN_LE_PHY_PARAM_2M);
	if (err) {
		LOG_ERR("Update phy conn failed (err %d)", err);
		return true;
	}

	// Stop processing advertisement data.
	return false;
}

static void split_central_device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
                                       struct net_buf_simple *ad) {
    char dev[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, dev, sizeof(dev));

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		LOG_DBG("[DEVICE]: %s", log_strdup(dev));
        bt_data_parse(ad, split_central_eir_found, (void *)addr);
	}
}

static int start_scan(void) {
	// No action is necessary if central is already scanning.
	if (is_scanning) {
		LOG_DBG("scanning is already on");
		return 0;
	}

	// If all the devices are connected, there is no need to scan.
	bool has_unconnected = false;
	for (int i = 0; i < DEVICE_COUNT; i++) {
		if (peripheral_conns[i] == NULL) {
			has_unconnected = true;
			break;
		}
	}
	if (!has_unconnected) {
		LOG_DBG("all devices are connected");
		return 0;
	}

	// Start scanning otherwise.
	is_scanning = true;
    LOG_DBG("Starting peripheral scanning");
    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, split_central_device_found);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }
    LOG_DBG("Scanning successfully started");
    return 0;
}

static void split_central_connected(struct bt_conn *conn, u8_t conn_err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_ERR("Failed to connect to %s (%u)", log_strdup(addr), conn_err);

		// Unset peripheral connection on failure.
		for (int i = 0; i < DEVICE_COUNT; i++) {
			if (peripheral_conns[i] == conn) {
				bt_conn_unref(peripheral_conns[i]);
				peripheral_conns[i] = NULL;
			}
		}
    } else {
		// TODO TODO TODO I think this is called for hosts (phones, etc). skip
		// for those? do loop and ID lookup here?
		LOG_DBG("Connected: %s", log_strdup(addr));
		split_central_process_connection(conn);
	}

	// Start scanning again if necessary.
	start_scan();
}

static void split_central_disconnected(struct bt_conn *conn, u8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_DBG("Disconnected: %s (reason %d)", log_strdup(addr), reason);

	// Handle if connection is a peripheral connection.
	for (int device_id = 0; device_id < DEVICE_COUNT; device_id++) {
		if (peripheral_conns[device_id] == conn) {
			// Unset peripheral connection.
			bt_conn_unref(peripheral_conns[device_id]);
    		peripheral_conns[device_id] = NULL;

			// Release all keys that were held by the peripheral.
			for (int position = 0; position < 8*POSITION_STATE_DATA_LEN; position++) {
				split_set_position(device_id, position, false);
			}
		}
	}

	// Start scanning again if necessary.
	start_scan();
}

static struct bt_conn_cb conn_callbacks = {
    .connected = split_central_connected,
    .disconnected = split_central_disconnected,
};

int zmk_split_bt_central_init(struct device *_arg) {
	// Initialize array for state of peripheral keys. Set each to 255, which
	// signifies that the key is released.
	memset(position_state, 255, 8*POSITION_STATE_DATA_LEN);

    bt_conn_cb_register(&conn_callbacks);
	return start_scan();
}

SYS_INIT(zmk_split_bt_central_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
