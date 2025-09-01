#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h> 
#include <zephyr/bluetooth/mesh.h>

#include <zephyr/settings/settings.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(main_log);

#include "proxy_gatt.h"
#include "provisioner.h"
#include "mesh.h"
#include "common.h"

#define DEV_UUID_LEN            16

K_SEM_DEFINE(sem_bt_init, 0, 1);

K_EVENT_DEFINE(evnt_bt_init);
#define  BT_INIT_BIT    BIT(0)

static uint8_t dev_uuid[DEV_UUID_LEN];
static struct k_work gatt_ad_work;


// --------------------------------
// GATT setup

// catch failed connections & turn connection LED on
static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if(err) {
        LOG_INF("BT connection failed with %d\n", err);
        return;
    }
    LOG_INF("BT connection established.\n");
    dk_set_led_on(CONNECTION_LED);
    
    k_sleep(K_MSEC(TINY_INTERVAL));

    // enable gatt
    k_work_submit(&gatt_ad_work);
    conn_curr = conn;

    // capture basic information about the current connection
    struct bt_conn_info info;
    err  = bt_conn_get_info(conn, &info);
    if(err<0) {
        LOG_ERR("Connection information retrieval failed with %d",err);
        return;
    }
    
    LOG_INF("Connection parameters: Interval %.2f ms, timeout %.2d ms, latency %d intervals",
    info.le.interval*1.25, info.le.timeout*10, info.le.latency);
}

// catch disconnection & turn connection LED off
static void on_disconnected(struct bt_conn *conn, uint8_t err)
{
    LOG_INF("GATT disconnected.");
    dk_set_led_off(CONNECTION_LED);

    err = bt_conn_disconnect(conn_curr, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    if(err) {
        LOG_INF("Failed to disconnect from Bluetooth client with %d", err);
    }
    else  {
        LOG_INF("Disconnected from Bluetooth client");
    }

    conn_curr = NULL;    
    bt_mesh_proxy_identity_enable();
}

// connection object has been returned to the pool
static void on_recycled(void)
{
    LOG_INF("Connection object returned to pool - disconnection complete.");
}

// connected device has updated the connection parameters
static void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    LOG_INF("Connection parameters updated: Interval %.2f ms, timeout %.2d ms, latency %d intervals",
    interval*1.25, timeout*10, latency);
}

// handle callbacks for different connection states
static struct bt_conn_cb conn_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .recycled = on_recycled,
    .le_param_updated = on_le_param_updated,
    // .le_data_len_updated = on_le_data_len_updated,
};

// data won't send if GATT isn't advertising, so there's no point wasting the resources
void send_sensor_data(void)
{
    int err;
    k_event_wait(&evnt_bt_init, BT_INIT_BIT, false, K_FOREVER);
    for(;;) {
        if(conn_curr != NULL) {
            err = send_sensor_notification();
            if(err) {
                LOG_WRN("Characteristic transmission failed with %d (are your attribute indexes correct?)", err);
            }
            
        }
       
        k_sleep(K_MSEC(LONG_INTERVAL));
    }
}

// -------------------------------------
// health client stuff

static struct bt_mesh_cfg_cli cfg_cli = {};

static void disp_health_current_status(struct bt_mesh_health_cli *cli, uint16_t addr, uint8_t test_id, uint16_t cid, uint8_t *faults, size_t fault_count)
{
    LOG_INF("Getting health status of device @ 0x%04x: test_id %u, cid 0x%04x, fault count %u", addr, test_id, cid, fault_count);
    for(int i=0; i<fault_count; i++) {
        LOG_INF("Fault %u: 0x%02x", i, faults[i]);
    }
}

static struct bt_mesh_health_cli health_cli = {
    .current_status = disp_health_current_status,
};

static struct bt_mesh_model_pub sensor_pub = {
    .addr = GROUP_ADDR,     // use multicast group address
    .msg = NET_BUF_SIMPLE(SENSOR_PUB_MSG_SIZE),
};

static const struct bt_mesh_model models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_CLI(&health_cli),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op, &sensor_pub, NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI, sensor_cli_op, NULL, NULL),
};

static const struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static struct bt_mesh_comp comp  = {
    .cid = 0x0059, // Nordic CID
    .pid = 0x0001,
    .vid = 0x0002,
    .elem_count = ARRAY_SIZE(elements),
    .elem = elements,
};

// broadcast sensor data to all nodes on the network
static void broadcast_sensor_data(void)
{
    // wait for a provisionee node to register
    k_sem_take(&sem_prov_init, K_FOREVER);
    uint32_t random_delay_ms;
    for(;;) {
        // wait between 3 and 5 seconds before broadcasting
        random_delay_ms = 2000 + (sys_rand32_get() % 3001);

        k_sleep(K_MSEC(random_delay_ms));

        int err = publish_status(models[3], sensor_pub);
        if(err) {
            LOG_WRN("Failed to send sensor status with %d", err);
        }
    }
}

// --------------------------------
// general setup

// button callback handler
static void button_pushed(uint32_t b_state, uint32_t has_changed)
{
    // when button 1 is pushed, enable DFU temporarily
    if(has_changed & BUTTON & (b_state == 1)) {
        enable_dfu();
        
    }

    // when button 2 is pushed, clear active connections & resume advertising
    if(has_changed & BUTTON2 & b_state) {
        int err = bt_conn_disconnect(conn_curr, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if(err) {
            LOG_INF("Failed to disconnect bluetooth clientwith %d", err);
            conn_curr = NULL;
        }
        bt_mesh_proxy_identity_enable();
    }

    // when button 3 is pressed, toggle air conditioning suppression
    if(has_changed & BUTTON3 & b_state) {
        sensor.suppressed = !sensor.suppressed;
        sensor.status = sensor.suppressed ? 0:calculate_sensor_status();
        LOG_INF("Air conditioning suppression status is now manually toggled: %u", sensor.suppressed);
    }
}

// --------------------------------------

static void prov_complete(uint16_t net_idx, uint16_t addr) {
    LOG_INF("Node {@ %u} has been succesfully provisioned", addr);
}

// callback for bt_enable
static void bt_done(int err) {

    static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
    .node_added = prov_node_added, 
    .unprovisioned_beacon = prov_unprovisioned_beacon,
    .complete = prov_complete,
    };

    if(err) {
        LOG_ERR("Bluetooth enable failed with %d", err);
        return;
    }

    err = bt_mesh_init(&prov, &comp);
    if(err) {
        LOG_ERR("Mesh initialisation failed with %d", err);
        return;
    }

#ifdef CONFIG_BT_SETTINGS
    LOG_INF("Loading settings...");
    settings_subsys_init();
    err = settings_load();
    if(err && err !=-ENOENT) {
        LOG_WRN("Settings were enabled, but loading failed with %d", err);
    }
#endif

    k_event_post(&evnt_bt_init, BT_INIT_BIT);
}

// initialise buttons, LEDs, bluetooth. start advertising
static int init_components(void)
{
    sys_rand_get(dev_uuid, DEV_UUID_LEN);

    // initialise board buttons & add the handler
    int err =  dk_buttons_init(button_pushed);
    if(err) {
        LOG_ERR("Button init failed with %d", err);
        return -1;
    }

    // initialise LEDs
    err = dk_leds_init();
    if(err) {
        LOG_ERR("LED init failed with %d", err);
        return -1;
    }

    // fix address as EE:A6:76:8F:AC:B9
    static bt_addr_le_t addr = {
        .type = BT_ADDR_LE_RANDOM,
        .a = {{0xB9, 0xAC, 0x8F, 0x76, 0xA6, 0xEE}}
    };

    err = bt_id_create(&addr, NULL);
    if(err) {
        LOG_ERR("Obtaining bluetooth address failed with %d", err);
    }
    
    // initialise bluetooth
    err = bt_enable(bt_done);

    if(err) {
        LOG_ERR("Bluetooth init ended with %d", err);
        return -1;
    }

    // wait for async bt initialisation to complete
    k_event_wait(&evnt_bt_init, BT_INIT_BIT, false, K_FOREVER);

    err = prov_init();
    if (err) {
        LOG_ERR("Mesh provisioning init failed with %d", err);
        return -1;
    }

    bt_conn_cb_register(&conn_callbacks);

    err = proxy_gatt_init();
    if(err) {
        LOG_ERR("GATT initialisation failed with %d", err);
        return -1;
    }

    sensor.status = 1;
    return 0;
}

// regularly check for unprovisioned beacons to provision
static void handle_provisioner(void)
{
    k_event_wait(&evnt_bt_init, BT_INIT_BIT, false, K_FOREVER);
    for(;;) {
        prov_cycle();
        k_sleep(K_MSEC(MED_INTERVAL));
    }
}

int main(void) {
    int err;
    err = init_components();
    if(err) {
        LOG_ERR("Fatal component error. Shutting down.");
        return -1;
    }

    // add a callback handler to the advertising work queue
    k_work_init(&gatt_ad_work, start_gatt_ad);

    LOG_INF("Initialisation complete.");
    // advertise the mesh network under an identity
    err = bt_mesh_proxy_identity_enable();
    LOG_INF("Enabled proxy identity");
    return 0;
}

// thread for generating sensor data values periodically
K_THREAD_DEFINE(gen_sensor_data_thread, 4096, generate_sensor_data, NULL, NULL, NULL, 7, 0, 0);
// thread for transmitting sensor data values via GATT characteristics
K_THREAD_DEFINE(send_sensor_data_thread, 4096, send_sensor_data, NULL, NULL, NULL, 7, 0, 0);
// thread for broadcasting sensor data values via mesh
K_THREAD_DEFINE(broadcast_data_thread, 4096, broadcast_sensor_data, NULL, NULL, NULL, 7, 0, 0);
// thread for provisioning nodes
K_THREAD_DEFINE(provisioning_thread, 4096, handle_provisioner, NULL, NULL, NULL, 7, 0, 0);

