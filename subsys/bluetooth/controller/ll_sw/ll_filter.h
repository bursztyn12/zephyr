/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define WL_SIZE        8

struct ll_wl {
	u8_t  enable_bitmask;
	u8_t  addr_type_bitmask;
	u8_t  bdaddr[WL_SIZE][BDADDR_SIZE];
	u8_t  anon;
};

void ll_filter_reset(bool init);

struct ll_wl *ctrl_wl_get(void);

bool ctrl_rl_enabled(void);
void ll_rl_rpa_update(bool timeout);

int ll_rl_idx_find(u8_t id_addr_type, u8_t *id_addr);
void ll_rl_pdu_adv_update(int idx, struct pdu_adv *pdu);
