#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

LOG_MODULE_DECLARE(main_log);

#include "provisioner.h"
#include "common.h"

K_SEM_DEFINE(sem_unprov_beacon, 0, 1);
K_SEM_DEFINE(sem_node_added, 0, 1);
K_SEM_DEFINE(sem_prov_init, 0, 1);

static uint16_t self_addr=1;
static uint8_t node_uuid[16];
static int is_self_configured = 0;

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
        // if that subnet has a stale key
        if(subnet->keys[0].net_key.key == PSA_KEY_ID_NULL) {
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

// re-configure all active nodes
static uint8_t unflag_nodes(struct bt_mesh_cdb_node *node, void *data)
{
    atomic_clear_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);
    return 0;
}

// create (or retrieve) a network key & enable provisioning
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

    // for debug purposes
    bt_mesh_cdb_node_foreach(unflag_nodes, NULL);
    return 0;
}

// get application key by index & add to the target node
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

    uint8_t net_key[16];
    bt_mesh_cdb_subnet_key_export(bt_mesh_cdb_subnet_get(NET_IDX), 0, net_key);

    // add subnet netkey to the node
    err = bt_mesh_cfg_cli_net_key_add(NET_IDX, node->addr, key->net_idx, net_key, &status);
    if(err || status) {
        LOG_ERR("Adding the netkey failed with %d (status %d)", err, status);
        return err;
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

// provision self
static int configure_self(struct bt_mesh_cdb_node *self)
{
    uint8_t status = 0;
    
    int err =  ret_add_app_key(self, 0);
    if(err) {
        LOG_ERR("Appkey addition to self failed with %d", err);
        return err;
    }
    
    // bind the sensor client model to the appkey
    err = bt_mesh_cfg_cli_mod_app_bind(self->net_idx, self->addr, self->addr, APP_IDX, BT_MESH_MODEL_ID_SENSOR_CLI, &status);
    if(err || status) {
        LOG_ERR("Appkey binding for sensor client failed with %d (status %d)", err, status);
        return err;
    }

    // bind the sensor server model to the appkey
    err = bt_mesh_cfg_cli_mod_app_bind(self->net_idx, self->addr, self->addr, APP_IDX, BT_MESH_MODEL_ID_SENSOR_SRV, &status);
    if(err || status) {
        LOG_ERR("Appkey binding for sensor server failed with %d (status %d)", err, status);
        return err;
    }

    atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);
    is_self_configured = 1;

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        bt_mesh_cdb_node_store(self);
	}

    LOG_INF("Self-configuration complete.");

    // subscribe to the sensor server model
    err = bt_mesh_cfg_cli_mod_sub_add(self->net_idx, self->addr, self->addr, GROUP_ADDR, BT_MESH_MODEL_ID_SENSOR_SRV, &status);
    if (err || status) {
        LOG_WRN("Subscribing to sensor readings failed with %d (status %d)", err, status);
    }

    err = bt_mesh_cfg_cli_mod_sub_add(self->net_idx, self->addr, self->addr, GROUP_ADDR, BT_MESH_MODEL_ID_SENSOR_CLI, &status);
    if (err || status) {
        LOG_WRN("Subscribing to sensor readings failed with %d (status %d)", err, status);
    }

    k_sleep(K_MSEC(MED_INTERVAL));

    // wake blocked threads
    k_sem_give(&sem_prov_init);
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

    k_sleep(K_MSEC(SHORT_INTERVAL));

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

            // choosing to not bind the configuration models (they're handled by the mesh stack directly)
			if (id == BT_MESH_MODEL_ID_CFG_CLI ||
			    id == BT_MESH_MODEL_ID_CFG_SRV) {
				continue;
			}

            LOG_INF("Binding SIG model, id #%u", id);
			err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX, node->addr, elem_addr, APP_IDX, id, &status);
			if (err || status) {
				LOG_ERR("Binding application to SIG model failed with %d (status %d)", err, status);
                return err;
			}
        }

        elem_addr ++;
    }

    atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        bt_mesh_cdb_node_store(node);
	}

    // force node to subscribe to the sensor server model
    err = bt_mesh_cfg_cli_mod_sub_add(node->net_idx, node->addr, node->addr, GROUP_ADDR, BT_MESH_MODEL_ID_SENSOR_SRV, &status);
    if (err || status) {
        LOG_WRN("Node subscribing to sensor readings failed with %d (status %d)", err, status);
    }

    err = bt_mesh_cfg_cli_mod_sub_add(node->net_idx, node->addr, node->addr, GROUP_ADDR, BT_MESH_MODEL_ID_SENSOR_CLI, &status);
    if (err || status) {
        LOG_WRN("Subscribing to sensor readings failed with %d (status %d)", err, status);
    }

    k_sleep(K_MSEC(MED_INTERVAL));

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
                return BT_MESH_CDB_ITER_STOP;
            }
		} 
        else {
			err = configure_node(node);
            if(err) {
                LOG_WRN("Node configuration failed with %d", err);
                bt_mesh_cdb_node_del(node, 1);
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
    
    while(!is_self_configured) {
        LOG_INF("Self is not configured");
        k_sleep(K_MSEC(MED_INTERVAL));
        return;
    }

    LOG_INF("(Searching for unprovisioned beacons...)");
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