#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h> 
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(main_log, CONFIG_LOG_DEFAULT_LEVEL);

#include "mesh.h"
#include "common.h"

#define ATTENTION_LED           DK_LED2
#define PROVISIONED_LED         DK_LED3

#define DEV_UUID_LEN            16

static uint8_t dev_uuid[DEV_UUID_LEN];
static struct bt_gatt_exchange_params exchange_params;

K_SEM_DEFINE(sem_prov, 0, 1);

K_EVENT_DEFINE(evnt_bt_init)
#define BT_INIT_BIT BIT(0)


// -------------------------------------
// Health server setup

static struct bt_mesh_cfg_cli cfg_cli = {
};

// node is paying attention to provisioning node
static void health_attention_on(const struct bt_mesh_model *model)
{
    dk_set_led(ATTENTION_LED, 1);

}

// node has finished paying attention to provisioning node
static void health_attention_off(const struct bt_mesh_model *model)
{
    dk_set_led(ATTENTION_LED, 0);
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_off =  health_attention_off,
    .attn_on = health_attention_on,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

// health publication context helper
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_model_pub sensor_pub = {
    .addr = GROUP_ADDR,     // multicast group address
    .msg = NET_BUF_SIMPLE(SENSOR_PUB_MSG_SIZE),
};

// define the mesh model
static const struct bt_mesh_model models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
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


// --------------------------------


void prov_complete(uint16_t net_idx, uint16_t addr) {
    LOG_INF("Node has been succesfully provisioned");
    k_sem_give(&sem_prov);
    dk_set_led(PROVISIONED_LED, 1);
}

void prov_reset(void)
{
    LOG_INF("Node has been reset, re-provisioning is required");
    dk_set_led(PROVISIONED_LED, 0);
}

// button callback handler
static void button_pushed(uint32_t b_state, uint32_t has_changed)
{
    // when button 1 is pressed, toggle air conditioning suppression
    if(has_changed & BUTTON & b_state) {
        sensor.suppressed = !sensor.suppressed;
        sensor.status = sensor.suppressed ? 0:calculate_sensor_status();
        LOG_INF("Air conditioning suppression status is now: %u", sensor.suppressed);
    }
}

// callback for bt_enable
static void bt_done(int err) {

    static const struct bt_mesh_prov prov = {
        .uuid = dev_uuid,
        .complete = prov_complete,
        .reset = prov_reset,
    };

    if(err) {
        LOG_ERR("Bluetooth enable failed with %d", err);
        return;
    }
    else {
        err = bt_mesh_init(&prov, &comp);
        if(err) {
            LOG_ERR("Mesh initialisation failed with %d", err);
            return;
        }

        err = bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
        if(err) {
            LOG_ERR("Mesh unprovisioned beaconing failed with %d", err);
            return;
        }
    }
    LOG_INF("Unprovisioned mesh beaconing started successfully.");

    // wake blocked threads
    k_event_post(&evnt_bt_init, BT_INIT_BIT);
}

static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params)
{
    if(!att_err) {
        LOG_INF("New MTU is %d bytes", bt_gatt_get_mtu(conn)-3);
    }
}

// once connected to the provisioner
static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if(err) {
        LOG_WRN("Bluetooth connection failed with %d", err);
    }

    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if(err) {
        LOG_WRN("MTU negotiation failed with %d", err);
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = on_connected,
};


// initialise buttons, LEDs, bluetooth. start advertising
static int init_components(void)
{
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

    // initialise bluetooth
    err = bt_enable(bt_done);

    if(err) {
        LOG_ERR("Bluetooth init ended with %d", err);
        return -1;
    }
    
    // wait for async bt initialisation to complete
    k_event_wait(&evnt_bt_init, BT_INIT_BIT, false, K_FOREVER);
    

    if(IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_load();
        if(err) {
            LOG_INF("Settings were enabled, but loading failed with %d", err);
        }
    }
    
    return 0;
}

int main(void) {
    sys_rand_get(dev_uuid, DEV_UUID_LEN);

    int err = init_components();
    if(err) {
        LOG_ERR("Fatal component error. Shutting down.");
        return -1;
    }

    bt_conn_cb_register(&conn_callbacks);

    LOG_INF("Initialisation complete.");

    // wait for the device to provision
    k_sem_take(&sem_prov, K_FOREVER);

    uint32_t random_delay_ms;
    for(;;) {
        // randomly delay for 3-5 seconds
        random_delay_ms = 2000 + (sys_rand32_get() % 3001);
        k_sleep(K_MSEC(random_delay_ms));

        // send status to all participating nodes
        int err = publish_status(models[3], sensor_pub);

        if(err) {
            LOG_WRN("Failed to send sensor status with %d", err);
        }
    }
}

K_THREAD_DEFINE(gen_sensor_data_thread, 1024, generate_sensor_data, NULL, NULL, NULL, 7, 0, 0);
