#ifndef DAISY_CHAIN_H_
#define DAISY_CHAIN_H_

/***********************************************************************
 * FSI DAISY CHAIN CONSTANTS
 ***********************************************************************/
// Motor driver daisy-chain defines:
// TODO(rafael): #define FSI_BUS_ENABLE                          (1)
#define DAISY_CHAIN_DRIVER_COUNT                (3) /*!< Total number of interconnected motor drivers (incl. this one) */
#define DAISY_CHAIN_DRIVER_ID                   (1) /*!< ID (0-indexed) of this motor driver */

#if (DAISY_CHAIN_DRIVER_COUNT > 1)
#define DAISY_CHAIN_ENABLE                      (1)
#else
#define DAISY_CHAIN_ENABLE                      (0)
#endif

#define FSI_PACKET_SZ_WORDS                     (8U)
#define GS_RING_BUFFER_SIZE                     (12U)

/***********************************************************************
 * LINEAR CONTROL CONSTANTS
 ***********************************************************************/
#define ROBOT_STATE_DIMS             (4 * DAISY_CHAIN_DRIVER_COUNT)   /*!< State dimension count (positions and velocities of all connected motors) */

/***********************************************************************
 * STRUCTURES
 ***********************************************************************/
// FSI daisy chain communication state:
typedef enum __fsi_comm_state_enum__
{
    COMM_INACTIVE   = 0,
    COMM_HANDSHAKE,
    COMM_ACTIVE,
    COMM_ERROR,
} fsi_comm_state_e;

typedef struct __fsi_recv_data_t__
{
    fsi_comm_state_e comm_state;

    // TODO(rafael): Make sure this is a multiple of 2:
    struct {} __attribute__ ((aligned (FSI_PACKET_SZ_WORDS)));
    uint16_t packets[DAISY_CHAIN_DRIVER_COUNT][FSI_PACKET_SZ_WORDS];

} fsi_recv_data_t;

// Ring buffer for CPU1 to CPU2 FSITX packet:
typedef struct __fsi_data_ring_buf_t__
{
    uint16_t cur_buf;

    struct {} __attribute__ ((aligned (4)));
    uint16_t buf[GS_RING_BUFFER_SIZE][FSI_PACKET_SZ_WORDS];
} fsi_data_ring_buf_t;

#endif /* DAISY_CHAIN_H_ */
