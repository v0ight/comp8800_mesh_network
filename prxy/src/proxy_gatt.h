#ifndef PROXY_GATT_H_
#define PROXY_GATT_H_

#include <zephyr/types.h>
#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>


// service UUID
#define BT_UUID_PROXY_GATT_VAL \
	BT_UUID_128_ENCODE(0xb0000001, 0x67bd, 0x4eab, 0xaea8, 0xe29e731dd4cd)

#define BT_UUID_PROXY_GATT BT_UUID_DECLARE_128(BT_UUID_PROXY_GATT_VAL)

// status characteristic UUID
#define BT_UUID_PROXY_GATT_STATUS_VAL \
    BT_UUID_128_ENCODE(0xb0000201, 0x67bd, 0x4eab, 0xaea8, 0xe29e731dd4cd)

#define BT_UUID_PROXY_GATT_STATUS BT_UUID_DECLARE_128(BT_UUID_PROXY_GATT_STATUS_VAL)

// suppression (info) characteristic UUID
#define BT_UUID_PROXY_GATT_SUPPRESSION_VAL \
    BT_UUID_128_ENCODE(0xb0000202, 0x67bd, 0x4eab, 0xaea8, 0xe29e731dd4cd)

#define BT_UUID_PROXY_GATT_SUPPRESSION BT_UUID_DECLARE_128(BT_UUID_PROXY_GATT_SUPPRESSION_VAL)

// sensor characteristic UUID
#define BT_UUID_PROXY_GATT_SENSOR_VAL \
    BT_UUID_128_ENCODE(0xb0000003, 0x67bd, 0x4eab, 0xaea8, 0xe29e731dd4cd)

#define BT_UUID_PROXY_GATT_SENSOR BT_UUID_DECLARE_128(BT_UUID_PROXY_GATT_SENSOR_VAL)

// name characteristic UUID
#define BT_UUID_PROXY_GATT_NAME_VAL \
    BT_UUID_128_ENCODE(0xb0000401, 0x67bd, 0x4eab, 0xaea8, 0xe29e731dd4cd)

#define BT_UUID_PROXY_GATT_NAME BT_UUID_DECLARE_128(BT_UUID_PROXY_GATT_NAME_VAL)

// threshold characteristic UUID
#define BT_UUID_PROXY_GATT_THRESHOLD_VAL \
    BT_UUID_128_ENCODE(0xb0000402, 0x67bd, 0x4eab, 0xaea8, 0xe29e731dd4cd)

#define BT_UUID_PROXY_GATT_THRESHOLD BT_UUID_DECLARE_128(BT_UUID_PROXY_GATT_THRESHOLD_VAL)

#define BT_UUID_PROXY_GATT_MANUAL_SUPPRESS_VAL \
BT_UUID_128_ENCODE(0xb0000005, 0x67bd, 0x4eab, 0xaea8, 0xe29e731dd4cd)

#define BT_UUID_PROXY_GATT_MANUAL_SUPPRESS BT_UUID_DECLARE_128(BT_UUID_PROXY_GATT_MANUAL_SUPPRESS_VAL)

typedef bool (*name_cb_t)(const char *);

extern struct bt_conn *conn_curr;

int proxy_gatt_init();
int send_sensor_notification();
int send_suppression_indication();
int send_status_indication();
void start_gatt_ad(struct k_work *work);
void enable_dfu(void);
void register_proxy_gatt(void);
void unregister_proxy_gatt(void);

#endif