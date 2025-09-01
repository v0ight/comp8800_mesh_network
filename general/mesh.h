#ifndef MESH_H_
#define MESH_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/mesh/access.h>

struct sensor_t {
    uint8_t value;
    bool status;
    bool suppressed;
    uint8_t threshold;
};

struct temp_control_t {
    int min, max, min_inc, max_inc;
    bool veloc;
};

typedef int (*callback_t)();

extern const struct bt_mesh_model_op sensor_cli_op[];
extern const struct bt_mesh_model_op sensor_srv_op[];
extern struct sensor_t sensor;

#define OP_SENSOR_GET           BT_MESH_MODEL_OP_2(0x82, 0x31)
#define OP_SENSOR_STATUS        BT_MESH_MODEL_OP_1(0x52)
#define SENSOR_PUB_MSG_SIZE     32

int calculate_sensor_status();
void generate_sensor_data();
int publish_status(struct bt_mesh_model model, struct bt_mesh_model_pub sensor_pub);
void attach_gatt_callbacks(callback_t status_cb, callback_t suppression_cb);

#endif