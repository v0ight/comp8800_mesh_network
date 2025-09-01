// #include <zephyr/bluetooth/mesh.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/random/random.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(main_log);

#include "mesh.h"
#include "common.h"


// generated temperature values will alternate between thresholds of 15 and 30, gradually increasing and decreasing
struct temp_control_t temp = {17, 30, -1, 3, 1};
// initial threshold is 20
struct sensor_t sensor = {22, 0, 0, 25};

static callback_t gatt_status_callback;
static callback_t gatt_suppression_callback;

// add callbacks so the proxy node can send indications on status change
void attach_gatt_callbacks(callback_t status_cb, callback_t suppression_cb)
{
    LOG_INF("Attaching proxy callbacks");
    gatt_status_callback = status_cb;
    gatt_suppression_callback = suppression_cb;
}

// simulate sensor data to notify the client with
// vary temperature within a given range
void generate_sensor_data(void)
{    
    for(;;) {

        if(gatt_status_callback != NULL) {
            gatt_status_callback();
        }
        if(gatt_suppression_callback != NULL) {
            gatt_suppression_callback();
        }

        if(!sensor.suppressed) {
            // switch temperature trends if a limit has been passed            
            if(sensor.value > temp.max && temp.veloc) {
                temp.veloc = 0;
            }
            else if(sensor.value < temp.min && !temp.veloc) {
                temp.veloc = 1;
            }

            if(temp.veloc) {
                sensor.value = sensor.value + (sys_rand32_get() % (temp.max_inc - temp.min_inc + 1) + temp.min_inc);
            } 
            else {
                sensor.value = sensor.value - (sys_rand32_get() % (temp.max_inc - temp.min_inc + 1) + temp.min_inc);
            }

            bool status = sensor.suppressed ? 0:calculate_sensor_status();

            if(sensor.status != status) {
                LOG_INF("Cooling is now switched %s", status ? "ON":"OFF");
                sensor.status = status;
            }

            k_sleep(K_MSEC(LONG_INTERVAL));
        }
        else {
            k_sleep(K_MSEC(LONG_INTERVAL));
        }
    }

}

// publish status to the group address
int publish_status(struct bt_mesh_model model, struct bt_mesh_model_pub sensor_pub)
{
    struct net_buf_simple *msg  = sensor_pub.msg;
    bt_mesh_model_msg_init(msg, OP_SENSOR_STATUS);
    net_buf_simple_add_u8(msg, sensor.value);
    net_buf_simple_add_u8(msg, sensor.status);
    net_buf_simple_add_u8(msg, sensor.suppressed);
    net_buf_simple_add_u8(msg, sensor.threshold);

    int err = bt_mesh_model_publish(&model);
    if(err) {
        LOG_WRN("Publish failed");
        return err;
    }
    LOG_INF("Sending sensor status: value %u, status %u, suppression %u, threshold %u", sensor.value, sensor.status, sensor.suppressed, sensor.threshold);
    return 0;
}

// respond to a client request for sensor data
int mesh_sensor_get(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
    struct net_buf_simple *msg;
    msg = NET_BUF_SIMPLE(SENSOR_PUB_MSG_SIZE);

    bt_mesh_model_msg_init(msg, OP_SENSOR_STATUS);
    net_buf_simple_add_u8(msg, sensor.value);
    net_buf_simple_add_u8(msg, sensor.status);
    net_buf_simple_add_u8(msg, sensor.suppressed);
    net_buf_simple_add_u8(msg, sensor.threshold);

    LOG_INF("Sending sensor status: value %u, status %u, suppression %u, threshold %u", sensor.value, sensor.status, sensor.suppressed, sensor.threshold);
    return bt_mesh_model_send(model, ctx, msg, NULL, NULL);
}

// recieve and handle sensor readings sent from other nodes on the network
void handle_sensor_readings(uint8_t val, bool status, bool suppressed, uint8_t threshold)
{
    sensor.value = (sensor.value + val) / 2;

    // sensor suppression has changed
    if(sensor.suppressed != suppressed) {
        LOG_INF("Sensor suppression status is now %u", suppressed);
        dk_set_led(STATUS_LED, suppressed ? 0:1);
    }

    if(gatt_suppression_callback != NULL) {
        gatt_suppression_callback();
    }
    sensor.suppressed = suppressed;

    status = sensor.suppressed ? 0:calculate_sensor_status();

    if(sensor.status != status) {
        LOG_INF("Cooling is now switched %s", status ? "ON":"OFF");
    }

    if(sensor.threshold != threshold) {
        LOG_INF("Threshold is now %u", threshold);
        sensor.threshold = threshold;
    }

    sensor.status = status;
    k_sleep(K_MSEC(MED_INTERVAL));
}

// handle sensor data from subscriptions
static int mesh_sensor_status(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
    // ignore messages sent by self
    if(ctx->addr == bt_mesh_model_elem(model)->rt->addr) {
        return 0;
    }

    uint8_t val = net_buf_simple_pull_u8(buf);
    bool status = net_buf_simple_pull_u8(buf);
    bool suppressed = net_buf_simple_pull_u8(buf);
    uint8_t threshold = net_buf_simple_pull_u8(buf);

    LOG_INF("Sensor feedback recieved from client: data %u, status %u, suppression %u, threshold %u", val, status, suppressed, threshold);
    handle_sensor_readings(val, status, suppressed, threshold);
    return 0;
}

// determine if the new sensor reading crosses the temperature threshold
int calculate_sensor_status(void)
{
    return (sensor.value > sensor.threshold) ? 1:0;
}

// opcode handlers for the sensor server
const struct bt_mesh_model_op sensor_srv_op[] = {
    {OP_SENSOR_GET, BT_MESH_LEN_MIN(2), mesh_sensor_get},
    BT_MESH_MODEL_OP_END,
};

// opcode handler for the sensor client
const struct bt_mesh_model_op sensor_cli_op[] = {
    {OP_SENSOR_STATUS, BT_MESH_LEN_MIN(1), mesh_sensor_status},
    BT_MESH_MODEL_OP_END,
};

