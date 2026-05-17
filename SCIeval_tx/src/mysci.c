/*
 * coder         : kawadak
 * date          : 2026-05-18
 * description   : SCI module
 */

#include "mysci.h"

#include <zephyr/kernel.h>

#include <zephyr/console/console.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/services/latency.h>
#include <bluetooth/services/latency_client.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

LOG_MODULE_REGISTER(mysci, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define INTERVAL_INITIAL 0x8
#define INTERVAL_INITIAL_US 10000
#define INTERVAL_UPDATE_PERIOD_MS 3000

static uint32_t test_intervals[] = {
	0,
	1000,
	1250,
	2000,
	4000
};

#define BT_UUID_VS_SCI_MIN_INTERVAL_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x1c840001, 0x49ac, 0x4905, 0x9702, 0x6e836da4cadd)

#define BT_UUID_VS_SCI_MIN_INTERVAL_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_VS_SCI_MIN_INTERVAL_SERVICE_VAL)

#define BT_UUID_VS_SCI_MIN_INTERVAL_CHAR_VAL \
	BT_UUID_128_ENCODE(0x1c840002, 0x49ac, 0x4905, 0x9702, 0x6e836da4cadd)

#define BT_UUID_VS_SCI_MIN_INTERVAL_CHAR \
	BT_UUID_DECLARE_128(BT_UUID_VS_SCI_MIN_INTERVAL_CHAR_VAL)

static K_SEM_DEFINE(phy_updated, 0, 1);
static K_SEM_DEFINE(min_interval_read_sem, 0, 1);
static K_SEM_DEFINE(frame_space_updated_sem, 0, 1);
static K_SEM_DEFINE(discovery_complete_sem, 0, 1);

static bool test_ready;
static bool initiate_conn_rate_update;
static bool is_central;

static uint32_t latency_response;

static uint16_t local_min_interval_us;
static uint16_t remote_min_interval_us;
static uint16_t common_min_interval_us;
static uint16_t remote_min_interval_handle;

static struct bt_conn *default_conn;

static struct bt_latency latency;
static struct bt_latency_client latency_client;

static struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(INTERVAL_INITIAL, INTERVAL_INITIAL, 0, 400);

static struct bt_conn_info conn_info = {0};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      BT_UUID_VS_SCI_MIN_INTERVAL_SERVICE_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE,
		DEVICE_NAME,
		DEVICE_NAME_LEN),
};

static ssize_t read_min_interval(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 void *buf,
				 uint16_t len,
				 uint16_t offset) {
	uint16_t value = sys_cpu_to_le16(local_min_interval_us);

	return bt_gatt_attr_read(conn,
				 attr,
				 buf,
				 len,
				 offset,
				 &value,
				 sizeof(value));
}

BT_GATT_SERVICE_DEFINE(sci_min_interval_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_VS_SCI_MIN_INTERVAL_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_VS_SCI_MIN_INTERVAL_CHAR,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_min_interval,
			       NULL,
			       NULL));

static void adv_start(void) {
	int err;

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
			      ad,
			      ARRAY_SIZE(ad),
			      sd,
			      ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed (%d)", err);
		return;
	}

	LOG_INF("Advertising started");
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr,
			  addr,
			  sizeof(addr));

	LOG_INF("Matched: %s", addr);
}

static void scan_filter_no_match(struct bt_scan_device_info *device_info,
				 bool connectable) {
}

static void scan_connecting_error(struct bt_scan_device_info *device_info) {
	LOG_WRN("Connecting failed");
}

BT_SCAN_CB_INIT(scan_cb,
		scan_filter_match,
		scan_filter_no_match,
		scan_connecting_error,
		NULL);

static void scan_init(void) {
	int err;

	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval = 0x0010,
		.window = 0x0010,
	};

	struct bt_scan_init_param scan_init = {
		.connect_if_match = true,
		.scan_param = &scan_param,
		.conn_param = conn_param,
	};

	bt_scan_init(&scan_init);

	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID,
				 BT_UUID_VS_SCI_MIN_INTERVAL_SERVICE);

	if (!err) {
		bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	}
}

static void scan_start(void) {
	int err;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);

	if (err) {
		LOG_ERR("Scan start failed (%d)", err);
		return;
	}

	LOG_INF("Scanning started");
}

static void connected(struct bt_conn *conn, uint8_t err) {
	if (err) {
		LOG_ERR("Connection failed");
		return;
	}

	default_conn = bt_conn_ref(conn);

	bt_conn_get_info(default_conn, &conn_info);

	if (conn_info.role == BT_CONN_ROLE_CENTRAL) {
		bt_scan_stop();
	} else {
		bt_le_adv_stop();
	}

	test_ready = true;

	k_sem_give(&discovery_complete_sem);

	LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	LOG_INF("Disconnected");

	test_ready = false;

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	if (is_central) {
		scan_start();
	} else {
		adv_start();
	}
}

static bool le_param_req(struct bt_conn *conn,
			 struct bt_le_conn_param *param) {
	return false;
}

static void conn_rate_changed(
	struct bt_conn *conn,
	uint8_t status,
	const struct bt_conn_le_conn_rate_changed *params) {
	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("Conn interval = %u us", params->interval_us);
	}
}

static void frame_space_updated(
	struct bt_conn *conn,
	const struct bt_conn_le_frame_space_updated *params) {
	k_sem_give(&frame_space_updated_sem);
}

static void le_phy_updated(
	struct bt_conn *conn,
	struct bt_conn_le_phy_info *param) {
	k_sem_give(&phy_updated);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.conn_rate_changed = conn_rate_changed,
	.frame_space_updated = frame_space_updated,
	.le_phy_updated = le_phy_updated,
};

static void latency_response_handler(const void *buf, uint16_t len) {
	uint32_t latency_time;

	if (len == sizeof(latency_time)) {

		latency_time = *((uint32_t *)buf);

		uint32_t cycles_spent =
			k_cycle_get_32() - latency_time;

		latency_response =
			(uint32_t)k_cyc_to_ns_floor64(cycles_spent) / 2000;
	}
}

static const struct bt_latency_client_cb latency_client_cb = {
	.latency_response = latency_response_handler,
};

int mysci_init(void) {
	int err;

	err = bt_enable(NULL);

	if (err) {
		LOG_ERR("Bluetooth init failed");
		return err;
	}

	LOG_INF("Bluetooth initialized");

	err = bt_conn_le_read_min_conn_interval(
		&local_min_interval_us);

	if (err) {
		LOG_ERR("Read min interval failed");
		return err;
	}

	err = bt_latency_init(&latency, NULL);

	if (err) {
		LOG_ERR("Latency init failed");
		return err;
	}

	err = bt_latency_client_init(
		&latency_client,
		&latency_client_cb);

	if (err) {
		LOG_ERR("Latency client init failed");
		return err;
	}

	return 0;
}

void mysci_select_mode(void) {
	while (1) {

		LOG_INF("Initiate update? y/n");

		char c = console_getchar();

		if (c == 'y') {
			initiate_conn_rate_update = true;
			break;
		}

		if (c == 'n') {
			initiate_conn_rate_update = false;
			break;
		}
	}

	while (1) {

		LOG_INF("Role c/p");

		char c = console_getchar();

		if (c == 'c') {

			is_central = true;

			scan_init();

			break;
		}

		if (c == 'p') {

			is_central = false;

			break;
		}
	}
}

void mysci_start(void) {
	if (is_central) {
		scan_start();
	} else {
		adv_start();
	}
}

void mysci_run(void) {
	k_sem_take(&discovery_complete_sem, K_FOREVER);

	while (default_conn) {

		uint32_t time = k_cycle_get_32();

		int err = bt_latency_request(
			&latency_client,
			&time,
			sizeof(time));

		if (err && err != -EALREADY) {
			LOG_WRN("Latency request failed");
		}

		k_sleep(K_MSEC(200));

		if (latency_response) {

			LOG_INF("Latency: %u us",
				latency_response);

			latency_response = 0;
		}
	}
}