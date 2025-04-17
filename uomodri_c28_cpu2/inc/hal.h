/*
 * Filename: hal.h
 * Description: Header file for the CPU2 Hardware Abstraction Layer
 */

#ifndef __HAL_H__
#define __HAL_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "f2838x_device.h"
#include "device.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/***********************************************************************
 * ENUMARATIONS
 ***********************************************************************/

typedef enum __fsi_Direction_enum__
{
    FSI_TX_DIR                  = 0,
    FSI_RX_DIR                  = 1
} fsi_Direction_e;

/***********************************************************************
 * HARDWARE ABSTRACTION LAYER (HAL) STRUCTURES
 ***********************************************************************/

/**
 * @struct  fsi_evt_cfg_t
 * @brief   Configuration for a single FSI event interrupt
 */
typedef struct __fsi_int_config_t__
{
    const uint16_t                      event;                    /*!< Event code to trigger interrupt (see FSI_TX_EVT_* or FSI_RX_EVT_*) */
    const FSI_InterruptNum              intNum;                   /*!< Which interrupt the event triggers (INT1 or 2) */
} fsi_evt_cfg_t;

/**
 * @struct  fsi_cfg_t
 * @brief   FSI global Hardware Abstraction Layer structure
 */
typedef struct __fsi_config_t__
{
    const uint32_t                      fsiBase;                   /*!< Peripheral address */
    const fsi_Direction_e               direction;                 /*!< Whether bus is TX or RX */
    const bool_t                        loopback;                  /*!< Whether FSI internal loopback enabled (if RX) */
    const uint32_t                      clkPrescaler;              /*!< Clock prescaler from PLLRAWCLK (if TX)*/
    const FSI_DataWidth                 dataWidth;                 /*!< Data width (1 vs 2 lane) for TX or RX operations */
    const fsi_evt_cfg_t*                eventInterrupts;           /*!< Event interrupt configuration */
} fsi_cfg_t;


/**
 * @struct  hal_cfg_t
 * @brief   General HAL structure handler for CPU2
 */
typedef struct __hal_config_t__
{
    const fsi_cfg_t*    const           p_fsiHandle;
} hal_cfg_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void HAL_ini(const hal_cfg_t*);
//void HAL_INT_ini(const int_cfg_t*, void (*)(void));

#endif /* __HAL_H__ */
