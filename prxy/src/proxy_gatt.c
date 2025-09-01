#include <zephyr/types.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/conn.h>
#include <dk_buttons_and_leds.h>
#include <mbedtls/sha256.h>
#include <zephyr/fs/fs.h>
#include <stdio.h>

#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#endif

LOG_MODULE_DECLARE(main_log);

#include "proxy_gatt.h"
#include "common.h"
#include "mesh.h"

// DFU window is currently at 4 minutes 
#define DFU_TIMEOUT_MAX         240
#define DEVICE_NAME_LEN         sizeof(CONFIG_BT_DEVICE_NAME)

static const char *suppression_pw_hash = "5e884898da28047151d0e56f8dc6292773603d0d6aabbdd62a11ef721d1542d8";

// static bool  button_state;
static bool sensor_notify_enabled = 0;
static bool suppression_indicate_enabled = 0;
static bool status_indicate_enabled = 0;
static bool dfu_enabled = 0;
static struct k_timer dfu_timeout;
static struct k_work disable_dfu_work;
static struct bt_gatt_indicate_params ind_params;

struct bt_conn *conn_curr = NULL;

// response data is defined locally, to facilitate network name changes
static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_PROXY_GATT_VAL),
};

static struct bt_le_ext_adv *gatt_adv;
static const struct bt_le_adv_param gatt_adv_params = {
    .id = BT_ID_DEFAULT,
    .sid = 0,
    .options = BT_LE_ADV_OPT_CONN,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};

#ifdef CONFIG_BOOTLOADER_MCUBOOT

// update advertising data with the SMP service UUID & register the SMP service
void enable_dfu(void)
{
    if(dfu_enabled) {
        return;
    }
  
    const struct bt_data dfu_sd[] = {
        BT_DATA(BT_DATA_NAME_SHORTENED, CONFIG_BT_DEVICE_NAME, DEVICE_NAME_LEN),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
    };

    int err = bt_le_ext_adv_stop(gatt_adv);
    if (err) {
        LOG_INF("Failed to stop previously running adv (was anything here?)");
    }

    k_sleep(K_MSEC(MED_INTERVAL));
    smp_bt_register();

    err = bt_le_ext_adv_set_data(gatt_adv, ad, ARRAY_SIZE(ad), dfu_sd, ARRAY_SIZE(dfu_sd));
    if(!err){
        err = bt_le_ext_adv_start(gatt_adv, BT_LE_EXT_ADV_START_DEFAULT);
    }
    if(err) {
        LOG_WRN("Starting ext adv failed with %d", err);
    }
    
    LOG_INF("Enabled DFU");
    dfu_enabled = 1;
    dk_set_led(DFU_LED, dfu_enabled);

    k_timer_start(&dfu_timeout, K_SECONDS(DFU_TIMEOUT_MAX), K_NO_WAIT);
}

// remove the SMP service UUID from the advertising data & unregister the SMP service
void disable_dfu(struct k_work *k_work)
{
    LOG_INF("Disabling DFU");
    int err = bt_le_ext_adv_stop(gatt_adv);
    if (err) {
        LOG_INF("Failed to stop previously running adv (was anything here?)");
    }

    k_sleep(K_MSEC(MED_INTERVAL));

    const struct bt_data gatt_sd[] = {
        BT_DATA(BT_DATA_NAME_SHORTENED, CONFIG_BT_DEVICE_NAME, DEVICE_NAME_LEN),
    };

    smp_bt_unregister();

    err = bt_le_ext_adv_set_data(gatt_adv, ad, ARRAY_SIZE(ad), gatt_sd, ARRAY_SIZE(gatt_sd));
    if(!err){
        err = bt_le_ext_adv_start(gatt_adv, BT_LE_EXT_ADV_START_DEFAULT);
    }
    if(err) {
        LOG_WRN("Starting ext adv failed with %d", err);
    }

    if(err) {
        LOG_WRN("Disabling DFU advertisement failed with %d", err);
    }

    LOG_INF("Disabled DFU");
    dfu_enabled = 0;
    dk_set_led(DFU_LED, dfu_enabled);
}

// wait for DFU to timeout before disabling it
void timeout_dfu(struct k_timer *timer)
{
    k_work_submit(&disable_dfu_work);
}
#endif

// change the network name
static ssize_t write_name_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    char local_name[32];
    // unsafe, for demo purposes
    memcpy(local_name, buf, len);
    LOG_INF("New network name has been assigned by external client - now %s", local_name);
    return sizeof(local_name);
}

// change the acceptable temperature threshold
static ssize_t write_threshold_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint8_t new_threshold = *((uint8_t *)buf);

    if(new_threshold >= 10 && new_threshold <= 40) {
        LOG_INF("Recieved new temperature threshold value - now %u", new_threshold);
        memcpy(&sensor.threshold, &new_threshold, len);
    }

    else {
        LOG_WRN("New threshold was out of safe range (10<=x<=40), please try again");
    }
    return 0;
}

// compare input password hash to stored password hash - if they match, toggle suppression
static ssize_t write_manual_suppression_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    unsigned char pw_hash[32];

    char pw[len+1];
    memcpy(&pw, buf, len);
    pw[len] = '\0';
    mbedtls_sha256((unsigned char*)pw, len, pw_hash, 0);

    char pw_hash_str[65];
    for (int i = 0; i < 32; i++) {
        snprintf(&pw_hash_str[i*2], 3, "%02x", pw_hash[i]);
    }

    if(strcmp(pw_hash_str, suppression_pw_hash) == 0) {
        LOG_INF("Suppression password accepted");
        sensor.suppressed = !sensor.suppressed;
        LOG_INF("Sensor suppression changed by external client - now %s", sensor.suppressed ? "ON":"OFF");
        return 0;
    }

    LOG_INF("Suppression password hash didn't match as expected.");
    return -1;
}

// callback functions for indication and notification changes
static void status_ccc_config_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    status_indicate_enabled = (value == BT_GATT_CCC_INDICATE);
    LOG_INF("Status CCC - indicate enabled is now %u", status_indicate_enabled);
}

static void suppression_ccc_config_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    suppression_indicate_enabled = (value == BT_GATT_CCC_INDICATE);
    LOG_INF("Suppression CCC - indicate enabled is now %u", suppression_indicate_enabled);
}

static void sensor_ccc_config_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    sensor_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Sensor CCC - notify enabled is now %u", sensor_notify_enabled);
}

static ssize_t read_status_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor.status, sizeof(sensor.status));
}

static ssize_t read_suppression_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor.suppressed, sizeof(sensor.suppressed));
}

static struct bt_gatt_attr proxy_gatt_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_PROXY_GATT),
    // define status characteristic, with indicate & read permission
    BT_GATT_CHARACTERISTIC(BT_UUID_PROXY_GATT_STATUS, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE, BT_GATT_PERM_READ, read_status_value, NULL, NULL),
    // define suppression characteristic, with indicate & read permission
    BT_GATT_CHARACTERISTIC(BT_UUID_PROXY_GATT_SUPPRESSION, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE, BT_GATT_PERM_READ, read_suppression_value, NULL, NULL),
    // define system  value characteristic, with notify permission
    BT_GATT_CHARACTERISTIC(BT_UUID_PROXY_GATT_SENSOR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    // define name change, threshold change, and manual suppression characteristic, with write permission
    BT_GATT_CHARACTERISTIC(BT_UUID_PROXY_GATT_NAME, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_name_value, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_PROXY_GATT_THRESHOLD, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_threshold_value, NULL), 
    BT_GATT_CHARACTERISTIC(BT_UUID_PROXY_GATT_MANUAL_SUPPRESS, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_manual_suppression_value, NULL),
    // define callbacks for indication and notification subscription
    BT_GATT_CCC(status_ccc_config_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CCC(sensor_ccc_config_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CCC(suppression_ccc_config_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service proxy_gatt_service = BT_GATT_SERVICE(proxy_gatt_attrs);

// register the custom gatt service
// this (and unregister_proxy_gatt) are mainly left over from when GATT was toggled with each BT connection
void register_proxy_gatt(void)
{
    int err = bt_gatt_service_register((struct bt_gatt_service *)&proxy_gatt_service);
    if(err) {
        LOG_WRN("Registering custom gatt services failed with %d", err);
        return;
    }
    LOG_INF("Succesfully registered custom gatt services");
}

// unregister the custom gatt service
void unregister_proxy_gatt(void)
{
    int err = bt_gatt_service_unregister((struct bt_gatt_service *)&proxy_gatt_service);
    if(err) {
        LOG_WRN("Unregistering custom gatt services failed with %d", err);
    }

    sensor_notify_enabled = 0;
    status_indicate_enabled = 0;
    suppression_indicate_enabled=0;
    LOG_INF("Unregistered custom gatt service");
}

// send sensor notifications to a gatt connection with notify enabled
int send_sensor_notification()
{
    if(!sensor_notify_enabled || conn_curr == NULL) {
        return 0;
    }
    return bt_gatt_notify(conn_curr, &proxy_gatt_service.attrs[8], &sensor.value, sizeof(sensor.value));
}

// send status indications to a gatt connection
int send_status_indication()
{
    if(!status_indicate_enabled || conn_curr == NULL) {
        return -1;
    }

    ind_params.attr = &proxy_gatt_service.attrs[2];
    ind_params.data = &sensor.status;
    ind_params.len = sizeof(sensor.status);

    return bt_gatt_indicate(conn_curr, &ind_params);
}

// send suppression indications to a gatt connection
int send_suppression_indication()
{
    if(!suppression_indicate_enabled || conn_curr == NULL) {
        return 0;
    }

    ind_params.attr = &proxy_gatt_service.attrs[5];
    ind_params.data = &sensor.suppressed;
    ind_params.len = sizeof(sensor.suppressed);

    return bt_gatt_indicate(conn_curr, &ind_params);
}

// begin the gatt advertisiing bearer
void start_gatt_ad(struct k_work *work)
{
    const struct bt_data gatt_sd[] = {
        BT_DATA(BT_DATA_NAME_SHORTENED, CONFIG_BT_DEVICE_NAME, DEVICE_NAME_LEN),
    };
    
    int err = bt_le_ext_adv_stop(gatt_adv);
    if (err) {
        LOG_INF("Failed to stop previously running adv (was anything here?)");
    }
    
    k_sleep(K_MSEC(MED_INTERVAL));
    LOG_WRN("Starting Gatt");

    err = bt_le_ext_adv_set_data(gatt_adv, ad, ARRAY_SIZE(ad), gatt_sd, ARRAY_SIZE(gatt_sd));
    if(!err){
        err = bt_le_ext_adv_start(gatt_adv, BT_LE_EXT_ADV_START_DEFAULT);
    }
    if(err) {
        LOG_WRN("Starting ext adv failed with %d", err);
    }
    
    LOG_INF("Advertising commencing.");
}

// init for registering callbacks for characteristic polling
int proxy_gatt_init() 
{
    register_proxy_gatt();
    k_work_init(&disable_dfu_work, disable_dfu);
    k_timer_init(&dfu_timeout, timeout_dfu, NULL);

#ifdef CONFIG_BOOTLOADER_MCUBOOT
    int err = smp_bt_unregister();
    if(err) {
        LOG_WRN("Failed to unregister SMP with %d", err);
    }
#endif

    err = bt_le_ext_adv_create(&gatt_adv_params, NULL, &gatt_adv);
    if(err) {
        LOG_WRN("Failed to create adv set with %d", err);
        return err;
    }

    attach_gatt_callbacks(send_status_indication, send_suppression_indication);
    return 0;
}




