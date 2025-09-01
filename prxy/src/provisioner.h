#ifndef PROXY_PROVISIONER_H_
#define PROXY_PROVISIONER_H_

#include <zephyr/bluetooth/mesh.h>

#define NET_IDX BT_MESH_NET_PRIMARY

int prov_init(void);
void prov_node_added(uint16_t net_idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem);
void prov_unprovisioned_beacon(uint8_t uuid[16], bt_mesh_prov_oob_info_t oob_info, uint32_t *uri_hash);
void prov_cycle(void);

extern struct k_sem sem_prov_init;

#endif