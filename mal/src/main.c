#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/dfu/mcuboot.h>

#include <zephyr/shell/shell.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include  <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include  <zephyr/mgmt/mcumgr/mgmt/handlers.h>

#include <zephyr/sys/reboot.h>
#include "zcbor_encode.h"
#include "zcbor_decode.h"
#include <mgmt/mcumgr/util/zcbor_bulk.h>

LOG_MODULE_REGISTER(mal_log);

#define CUSTOM_MGMT_GROUP_ID        64
#define MGMT_ID_NEW_SAFE_ADDR       0
#define MGMT_ID_ROLLBACK            1
#define MGMT_ID_DISABLE_NETWORK     2
#define MGMT_ID_ENABLE_NETWORK      3

#define MED_INTERVAL                300
#define LONG_INTERVAL               5000
#define NET_IDX                     BT_MESH_NET_PRIMARY
#define APP_IDX                     0
#define DEV_UUID_LEN                16
#define MAL_NODE_CONFIGURED         2

K_SEM_DEFINE(sem_bt_init, 0, 1);
K_SEM_DEFINE(sem_unprov_beacon, 0, 1);
K_SEM_DEFINE(sem_node_added, 0, 1);

static struct k_work gatt_ad_work;
static struct bt_conn *conn_curr = NULL;
static uint16_t self_addr=1;
static uint8_t node_uuid[16];
// can just store the net key openly here
static uint8_t net_key[16];
static uint8_t dev_uuid[DEV_UUID_LEN];

static char safe_addr[17] = "47:06:D9:FE:DD:B";
static bool disable = 0;

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
};

const struct bt_data gatt_sd[] = {
    BT_DATA(BT_DATA_NAME_SHORTENED, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME)),
};


// ==============================================================================
// shell setup

// recieved SMP command with new safe address
static int write_new_safe_addr(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;

    if(!ctxt || !ctxt->reader) {
        LOG_WRN("Invalid SMP context");
    }

    struct zcbor_string zmsg;
    struct zcbor_string zval;

    if(!zcbor_map_start_decode(zsd)) {
        LOG_WRN("Map start decode failed");
    }

    int ok = zcbor_tstr_decode(zsd, &zmsg);
    if (!ok) {
        return -1;
    }

    ok = zcbor_tstr_decode(zsd, &zval);
    if (!ok) {
        return -1;
    }

    zcbor_map_end_decode(zsd);

    size_t len = MIN(zval.len, sizeof(safe_addr) - 1);
    memcpy(safe_addr, zval.value, len);
    safe_addr[len] = '\0';
    LOG_INF("New safe address is: %s", safe_addr);
    return 0;
}

// recieved SMP command to rollback the firmware
static int read_rollback(struct smp_streamer *ctxt)
{
    boot_request_upgrade(BOOT_UPGRADE_PERMANENT);
    sys_reboot(SYS_REBOOT_COLD);
    return 0;
}

const struct bt_mesh_model_op generic_op[] = {
    BT_MESH_MODEL_OP_END,
};

// define empty and generic opcodes for the models
static struct bt_mesh_cfg_cli cfg_cli = {};
static struct bt_mesh_health_srv health_srv = {};
static struct bt_mesh_model_op sensor_srv_op[] = {BT_MESH_MODEL_OP_END};
static struct bt_mesh_model_op onoff_srv_op[] = {BT_MESH_MODEL_OP_END};
static struct bt_mesh_model_op level_srv_op[] = {BT_MESH_MODEL_OP_END};
static struct bt_mesh_model_op power_onoff_srv_op[] = {BT_MESH_MODEL_OP_END};
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

// define the mesh model
// filled with models that are plausible for the network model to employ
static const struct bt_mesh_model models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op, NULL, NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, onoff_srv_op, NULL, NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_SRV, level_srv_op, NULL, NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_POWER_ONOFF_SRV, power_onoff_srv_op, NULL, NULL),
    // would add more here to cover bases - onoff, etc.
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


// raise all sensor values 
static int raise_values(void)
{
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = models[2].keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    if(ctx.app_idx == BT_MESH_KEY_UNUSED) {
        LOG_ERR("No AppKey bound to Sensor Server model, cannot send status");
        return -1;
    }

    // 0x52 is the standard Bluetooth mesh model opcode for "Sensor Status" (i.e. sending mesh sensor messages)
    BT_MESH_MODEL_BUF_DEFINE(buf, 0x52, 8);
    bt_mesh_model_msg_init(&buf, 0x52);
    net_buf_simple_add_u8(&buf, 254);
    net_buf_simple_add_u8(&buf, 254);
    net_buf_simple_add_u8(&buf, 254);
    net_buf_simple_add_u8(&buf, 254);
    net_buf_simple_add_u8(&buf, 254);
    net_buf_simple_add_u8(&buf, 254);
    net_buf_simple_add_u8(&buf, 254);
    net_buf_simple_add_u8(&buf, 254);

    LOG_INF("Sending mal message");
    return bt_mesh_model_send(&models[2], &ctx, &buf, NULL, NULL);
}

// lower all sensor values
static int lower_values(void)
{
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = models[2].keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    if(ctx.app_idx == BT_MESH_KEY_UNUSED) {
        LOG_ERR("No AppKey bound to Sensor Server model, cannot send status");
        return -1;
    }

    BT_MESH_MODEL_BUF_DEFINE(buf, 0x52, 8);
    bt_mesh_model_msg_init(&buf, 0x52);
    net_buf_simple_add_u8(&buf, 15);
    net_buf_simple_add_u8(&buf, 15);
    net_buf_simple_add_u8(&buf, 15);
    net_buf_simple_add_u8(&buf, 15);
    net_buf_simple_add_u8(&buf, 15);
    net_buf_simple_add_u8(&buf, 15);
    net_buf_simple_add_u8(&buf, 15);
    net_buf_simple_add_u8(&buf, 15);

    return bt_mesh_model_send(&models[2], &ctx, &buf, NULL, NULL);
}

static uint8_t reset_node(struct bt_mesh_cdb_node *node, void *data)
{
    // subscribe to the sensor server model
    uint8_t status = 0;
    if(node->addr != self_addr) {
        int err = bt_mesh_cfg_cli_mod_app_unbind(node->net_idx, node->addr, node->addr, APP_IDX, BT_MESH_MODEL_ID_SENSOR_SRV, &status);
        err = bt_mesh_cfg_cli_mod_app_unbind(node->net_idx, node->addr, node->addr, APP_IDX, BT_MESH_MODEL_ID_SENSOR_CLI, &status);
        LOG_INF("Unbound node with err %d (status %d)", err, status);

    }
    return BT_MESH_CDB_ITER_CONTINUE;
}

// recieved SMP command to disable the network
static int read_network_disable(struct smp_streamer *ctxt)
{
    raise_values();
    k_sleep(K_MSEC(MED_INTERVAL));

    disable = 1;
    bt_mesh_cdb_node_foreach(reset_node, NULL);

    return 0;
}

// recieved SMP command to enable the network
static int read_network_enable(struct smp_streamer *ctxt)
{
    disable = 0;

    k_sleep(K_MSEC(MED_INTERVAL));
    lower_values();

    return 0;
}

static const struct mgmt_handler custom_handlers[] = {
    [MGMT_ID_NEW_SAFE_ADDR] =  {
        .mh_write = write_new_safe_addr,
    },
    [MGMT_ID_ROLLBACK] =  {
        .mh_read = read_rollback,
    },
    [MGMT_ID_DISABLE_NETWORK] =  {
        .mh_read = read_network_disable,
    },
    [MGMT_ID_ENABLE_NETWORK] =  {
        .mh_read = read_network_enable,
    },
};

static struct mgmt_group custom_group = {
    .mg_handlers = custom_handlers,
    .mg_handlers_count = ARRAY_SIZE(custom_handlers),
    .mg_group_id = CUSTOM_MGMT_GROUP_ID,
};

static void custom_smp_reg(void)  
{
    mgmt_register_group(&custom_group);
}

SYS_INIT(custom_smp_reg, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
MCUMGR_HANDLER_DEFINE(custom_group_mcumgr, custom_smp_reg);

static struct bt_le_ext_adv *gatt_adv;
static const struct bt_le_adv_param gatt_adv_params = {
    .id = BT_ID_DEFAULT,
    .sid = 0,
    .options = BT_LE_ADV_OPT_CONN,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    LOG_INF("BT connection established.\n");

    char str_addr[30];
    bt_addr_le_to_str(bt_conn_get_dst(conn), str_addr, BT_ADDR_LE_STR_LEN);
    LOG_INF("Connected to device @%s", str_addr);


    if(memcmp(str_addr, safe_addr, sizeof(safe_addr)) != 0)  {
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
    }

    conn_curr = conn;
    return;
}

static void on_disconnected(struct bt_conn *conn, uint8_t err)
{
    k_work_submit(&gatt_ad_work);

    conn_curr = NULL;   
}

// handle callbacks for different connection states
static struct bt_conn_cb conn_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};


// ==============================================================================
// mesh setup


// an unprovisioned node has been added to the network
void prov_node_added(uint16_t net_idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem)
{
    k_sem_give(&sem_node_added);
}

// an unprovisioned beacon has been detected
void prov_unprovisioned_beacon(uint8_t uuid[16], bt_mesh_prov_oob_info_t oob_info, uint32_t *uri_hash) 
{
    
    if(memcmp(uuid, node_uuid, 16) != 0) {
        LOG_INF("Detected new unprovisioned beacon with UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7], uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
        memcpy(node_uuid, uuid, 16);
        k_sem_give(&sem_unprov_beacon);
    }
}

// configure the new CDB
static int setup_cdb(uint8_t net_key[16])
{
    uint8_t app_key[16];
    struct bt_mesh_cdb_app_key *key;

    // allocate a subnet to the net_idx
    struct bt_mesh_cdb_subnet *subnet = bt_mesh_cdb_subnet_get(NET_IDX);
    int err;
    if(subnet != NULL) {
        if(subnet->keys[0].net_key.key == 0) {
            LOG_INF("Subnet already existed, but key retrieval failed. Regenerating key...");
            err = bt_mesh_cdb_subnet_key_import(subnet, 0, net_key);
            if(err) {
                LOG_ERR("Subnet import failed with %d", err);
                return err;
            }
        }
    } 
    else {
        LOG_INF("Creating new subnet for net_idx %d", NET_IDX);
        subnet = bt_mesh_cdb_subnet_alloc(NET_IDX);

        if(subnet == NULL) {
            LOG_ERR("Subnet allocation failed (max subnet count reached?).");
            return -1;
        }
        
        // import the network key into the subnet
        err = bt_mesh_cdb_subnet_key_import(subnet, 0, net_key);
        if(err) {
            LOG_ERR("Subnet import failed with %d", err);
            return err;
        }

        if(IS_ENABLED(CONFIG_BT_SETTINGS)) {
            bt_mesh_cdb_subnet_store(subnet);
        }
    }

    // if a key using that APP_IDX already exists
    key = bt_mesh_cdb_app_key_get(APP_IDX);
    if(key != NULL) {
        LOG_INF("App key exists, using existing.");
    }
    else {
        // generate an application key by APP_IDX
        key = bt_mesh_cdb_app_key_alloc(NET_IDX, APP_IDX);
        if(key == NULL) {
            LOG_ERR("App key allocation failed.");
            return -1;
        }
    
        sys_rand_get(app_key, sizeof(app_key));
        err = bt_mesh_cdb_app_key_import(key, 0, app_key);
        if(err) {
            LOG_ERR("Appkey CBD import failed with %d", err);
            return err;
        }
        
        if(IS_ENABLED(CONFIG_BT_SETTINGS)) {
            bt_mesh_cdb_app_key_store(key);
        }
    }

    return 0;
}


int prov_init(void)
{
    uint8_t dev_key[16], net_key[16];
    int err;


    sys_rand_get(net_key, sizeof(net_key));

    err = bt_mesh_cdb_create(net_key);
    if(err == -EALREADY) {
        LOG_INF("Using stored CDB");
    }
    else if (err)  {
        LOG_INF("Initial CDB creation failed with %d", err);
        return err;
    } 
    else {
        LOG_INF("Creating new CDB");
        atomic_set_bit(bt_mesh_cdb.flags, BT_MESH_CDB_VALID);
        setup_cdb(net_key);
    }

    LOG_INF("CDB main settings have been initialised");

    // wait for initialisation to settle
    sys_rand_get(dev_key, 16);
    // since this is initialisation, assume the iv index is 0
    err = bt_mesh_provision(net_key, NET_IDX, 0, 0, self_addr, dev_key);
    if(err == -EALREADY) {
        LOG_INF("Already provisioned, using existing network key.");
    } 
    else if(err) {
        LOG_ERR("Provisioning failed with %d", err);
        return err;
    }

    return 0;
}

// used in place of configure_self when the firmware is flashed to an existing provisioner
static void setup_models(void)
{
    struct bt_mesh_cdb_app_key *key;
    uint8_t app_key[16];

    key = bt_mesh_cdb_app_key_get(APP_IDX);
    if(key == NULL) {
        LOG_ERR("No appkey assigned.");
        return;
    }

    
    int err = bt_mesh_cdb_app_key_export(key, 0, app_key);
    if(err) {
        LOG_ERR("Appkey export from CBD failed with %d", err);
        return;
    }

    printk("Appkey = ");
        for (int j = 0; j < 16; j++) {
            printk("%02x", net_key[j]);
        }
    printk("\n");

    err = bt_mesh_app_key_add(APP_IDX, NET_IDX, app_key);

    for (int i = 0; i < ARRAY_SIZE(models); i++) {
    // Skip config and health models
        if (models[i].id == BT_MESH_MODEL_ID_CFG_SRV ||
            models[i].id == BT_MESH_MODEL_ID_HEALTH_SRV) {
            continue;
        }
        ((struct bt_mesh_model *)&models[i])->keys[0] = APP_IDX;
    }
}

// get application key & bind to the node's models
static int ret_add_app_key(struct bt_mesh_cdb_node *node, bool is_remote)
{
    struct bt_mesh_cdb_app_key *key;
    uint8_t status = 0;
    uint8_t app_key[16];

    key = bt_mesh_cdb_app_key_get(APP_IDX);
    if(key == NULL) {
        LOG_ERR("No appkey assigned.");
        return -1;
    }

    int err = bt_mesh_cdb_app_key_export(key, 0, app_key);
    if(err) {
        LOG_ERR("Appkey export from CBD failed with %d", err);
        return err;
    }

    bt_mesh_cdb_subnet_key_export(bt_mesh_cdb_subnet_get(NET_IDX), 0, net_key);
    
    LOG_INF("node addr is %u, netkey is:", node->addr);
    for (int j = 0; j < 16; j++) {
            printk("%02x", net_key[j]);
        }
    printk("\n");

    if(is_remote) {
        // add subnet netkey to the node
        err = bt_mesh_cfg_cli_net_key_add(NET_IDX, node->addr, key->net_idx, net_key, &status);
        if(err && status) {
            LOG_ERR("Adding the netkey failed with %d (status %d)", err, status);
            return err;
        }
    }

    k_sleep(K_MSEC(MED_INTERVAL));

    if(is_remote) {
        // add appkey to the node
        err = bt_mesh_cfg_cli_app_key_add(NET_IDX, node->addr, key->net_idx, key->app_idx, app_key, &status);
        if(err || status) {
            LOG_ERR("Adding the appkey failed with %d (status %d)", err, status);
            return err;
        }
    }
    else {
        err = bt_mesh_app_key_add(key->app_idx, NET_IDX, app_key);
        if(err) {
            LOG_ERR("Adding the appkey failed with %d (status %d)", err, status);
            return err;
        }
    }
    
    return 0;
}

// provision self node
static int configure_self(struct bt_mesh_cdb_node *self)
{
    int err =  ret_add_app_key(self, 0);
    if(err) {
        LOG_ERR("Appkey addition to self failed with %d", err);
        return err;
    }

    for (int i = 0; i < ARRAY_SIZE(models); i++) {
    // Skip config and health models
        if (models[i].id == BT_MESH_MODEL_ID_CFG_SRV ||
            models[i].id == BT_MESH_MODEL_ID_HEALTH_SRV) {
            continue;
        }
        ((struct bt_mesh_model *)&models[i])->keys[0] = APP_IDX;
    }

    atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        bt_mesh_cdb_node_store(self);
	}

    LOG_INF("Self-configuration complete.");
    return 0;
}

// provision a managed node
static int configure_node(struct bt_mesh_cdb_node *node)
{
    NET_BUF_SIMPLE_DEFINE(buf, BT_MESH_RX_SDU_MAX);
	struct bt_mesh_comp_p0_elem elem;
	struct bt_mesh_comp_p0 comp;
	uint8_t status = 0;

    LOG_INF("Configuring node %d", node->addr);

    int err = ret_add_app_key(node, 1);
    if(err) {
        LOG_ERR("Appkey addition to node failed with %d", err);
        return err;
    }

    k_sleep(K_MSEC(MED_INTERVAL));

    LOG_INF("Retrieving composition data from node %d with NET_IDX %u", node->addr, NET_IDX);

    uint8_t rsp = 0;
    err = bt_mesh_cfg_cli_comp_data_get(NET_IDX, node->addr, 0, &rsp, &buf);
    if(err || (rsp != 0)) {
        LOG_ERR("Data composition retrieval failed with %d (response %u)", err, rsp);
        return err;
    }

    err = bt_mesh_comp_p0_get(&comp, &buf);
    if(err) {
        LOG_ERR("Composition data parsing failed with %d", err);
        return err;
    }

    int elem_addr = node->addr;
    // while there are still unbound elements
    while(bt_mesh_comp_p0_elem_pull(&comp, &elem)) {
        LOG_INF("Node %u comprises %u SIG models & %u vendor models", elem_addr, elem.nsig, elem.nvnd);

        // bind appkey to all SIG models in the element
        for(int i=0; i<elem.nsig; i++) {
            uint16_t id = bt_mesh_comp_p0_elem_mod(&elem, i);

            // choosing to not bind the configuration models
			if (id == BT_MESH_MODEL_ID_CFG_CLI ||
			    id == BT_MESH_MODEL_ID_CFG_SRV) {
				continue;
			}

            LOG_INF("Binding SIG model with id #%u", id);
			err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, node->addr, elem_addr, APP_IDX, id, &status);
			if (err || status) {
				LOG_ERR("Binding appkey to SIG model failed with %d (status %d)", err, status);
                return err;
			}
        }
    }

    atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        bt_mesh_cdb_node_store(node);
	}

    LOG_INF("Node configuration complete.");
    return 0;
}

// check if a node is unconfigured
static uint8_t check_unconfigured(struct bt_mesh_cdb_node *node, void *data)
{
    int err;
	if (!atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED)) {
		if (node->addr == self_addr) {
			err = configure_self(node);
            if(err) {
                LOG_ERR("Self-configuration failed with %d", err);
            }
		} 
        else {
            if(!disable){
                err = configure_node(node);
                if(err) {
                    LOG_WRN("Node configuration failed with %d", err);
                    bt_mesh_cdb_node_del(node, 1);
                }
            }
		}
	}
	return BT_MESH_CDB_ITER_CONTINUE;
}

// wait for nodes with unprovisioned beacons in range
void prov_cycle(void)
{
    k_sem_reset(&sem_unprov_beacon);
    k_sem_reset(&sem_node_added);
    bt_mesh_cdb_node_foreach(check_unconfigured, NULL);
    

    LOG_INF("(Provisioning...)");
    int err = k_sem_take(&sem_unprov_beacon, K_SECONDS(5));
    if (err == -EAGAIN) {
        return;
    } else if (err != 0) {
        LOG_INF("Beaconing sephamore failed with %d", err);
    }

    LOG_INF("Provisioning...");
    err = bt_mesh_provision_adv(node_uuid, NET_IDX, 0, 5);
    if(err) {
        LOG_ERR("Provisioning node failed with %d", err);
        return;
    }

    LOG_INF("(Waiting for the node to be added...)");
    err = k_sem_take(&sem_node_added, K_SECONDS(5));
    if (err) {
        LOG_INF("Node addition failed with %d", err);
        return;
    } 
    else {
        LOG_INF("Mode added succesfully.");
    }

}

// ==============================================================================
// general initialisation

// callback for bt_enable
static void bt_done(int err) {

    static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
    .node_added = prov_node_added, 
    .unprovisioned_beacon = prov_unprovisioned_beacon,
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

    k_sleep(K_MSEC(MED_INTERVAL));
#endif

    k_sem_give(&sem_bt_init);
}


// init for registering callbacks for characteristic polling
int proxy_gatt_init() 
{

    int err = bt_le_ext_adv_create(&gatt_adv_params, NULL, &gatt_adv);
    if(err) {
        LOG_WRN("Failed to create adv set with %d", err);
        return err;
    }

    err = bt_le_ext_adv_set_data(gatt_adv, ad, ARRAY_SIZE(ad), gatt_sd, ARRAY_SIZE(gatt_sd));
    err = bt_le_ext_adv_start(gatt_adv, BT_LE_EXT_ADV_START_DEFAULT);
    if(err) {
        LOG_INF("Advertising failed with %d", err);
    }

    return 0;
}

// initialise buttons, LEDs, bluetooth. start advertising
static int init_components(void)
{
    sys_rand_get(dev_uuid, DEV_UUID_LEN);

    // initialise LEDs
    int err = dk_leds_init();
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
    k_sem_take(&sem_bt_init, K_FOREVER);

    err = prov_init();
    if (err) {
        LOG_ERR("Mesh provisioning init failed with %d", err);
        return -1;
    }

    setup_models();

    bt_conn_cb_register(&conn_callbacks);
    proxy_gatt_init();
    return 0;
}

int main(void) {
    int err = boot_write_img_confirmed();
	if(err) {
		LOG_WRN("Boot confirmation failed with %d", err);
	}

    err = init_components();
    if(err) {
        LOG_ERR("Fatal component error. Shutting down.");
        return -1;
    }

    // add a callback handler to the advertising work queue

    LOG_INF("Initialisation complete.");

    // identify netkeys held previously in device settings
    for (size_t i = 0; i < ARRAY_SIZE(bt_mesh_cdb.subnets); i++) {
        struct bt_mesh_cdb_subnet *sub = bt_mesh_cdb_subnet_get(i);

        if (sub->net_idx == BT_MESH_KEY_UNUSED) {
            continue; // skip unused slots
        }

        LOG_INF("Identifying netkeys...");
        uint8_t net_key[16];
        bt_mesh_cdb_subnet_key_export(bt_mesh_cdb_subnet_get(NET_IDX), 0, net_key);
        printk("Subnet %zu: NetIdx = 0x%03x\n", i, sub->net_idx);
        printk("  Key = ");
        for (int j = 0; j < 16; j++) {
            printk("%02x", net_key[j]);
        }
        printk("\n");
    }

    for(;;) {
        prov_cycle();
    }
    return 0;
}