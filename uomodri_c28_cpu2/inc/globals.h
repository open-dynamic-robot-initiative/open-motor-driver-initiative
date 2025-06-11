/*
 * File name: globals.h
 * Description: Header containing declarations for global CPU2 variables.
 */

#ifndef INC_GLOBALS_CPU2_H_
#define INC_GLOBALS_CPU2_H_

#include "daisy_chain.h"

#define FSI_COMM_STATE_VAR (gs_rw_fsi_recv_pack_buf.comm_state)

extern volatile fsi_recv_data_t gs_rw_fsi_recv_pack_buf;
extern volatile fsi_data_ring_buf_t gs_r_fsi_cla_state_ring_buf;

extern volatile uint16_t is_first_packet_tx;
extern volatile uint16_t next_packet_id;

#endif /* INC_GLOBALS_CPU2_H_ */
