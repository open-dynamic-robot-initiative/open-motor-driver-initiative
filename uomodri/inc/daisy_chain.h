#ifndef DAISY_CHAIN_H_
#define DAISY_CHAIN_H_

/***********************************************************************
 * FSI DAISY CHAIN CONSTANTS
 ***********************************************************************/
// Motor driver daisy-chain defines:
// TODO(rafael): #define FSI_BUS_ENABLE                          (1)
#define DAISY_CHAIN_DRIVER_COUNT                (2) /*!< Total number of interconnected motor drivers (incl. this one) */
#define DAISY_CHAIN_DRIVER_ID                   (1) /*!< ID (0-indexed) of this motor driver */

#if (DAISY_CHAIN_DRIVER_COUNT > 1)
#define DAISY_CHAIN_ENABLE                      (1)
#else
#define DAISY_CHAIN_ENABLE                      (0)
#endif

#define FSI_PACKET_SZ_WORDS                     (8U)

/***********************************************************************
 * LINEAR CONTROL CONSTANTS
 ***********************************************************************/
#define ROBOT_STATE_DIMS             (4 * DAISY_CHAIN_DRIVER_COUNT)   /*!< State dimension count (positions and velocities of all connected motors) */

/***********************************************************************
 * STRUCTURES
 ***********************************************************************/
typedef struct __fsi_recv_data_t__
{
    volatile bool error;

    // TODO(rafael): Make sure this is a multiple of 2:
    struct {} __attribute__ ((aligned (FSI_PACKET_SZ_WORDS)));
    uint16_t packets[DAISY_CHAIN_DRIVER_COUNT][FSI_PACKET_SZ_WORDS];

} fsi_recv_data_t;

#endif /* DAISY_CHAIN_H_ */
