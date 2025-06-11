/*
 * Filename: hal.h
 * Description: Header file for the CPU2 Hardware Abstraction Layer
 */

#ifndef __HAL_CPU2_H__
#define __HAL_CPU2_H__

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

typedef enum __fsi_direction_enum__
{
    FSI_TX_DIR                  = 0,
    FSI_RX_DIR                  = 1
} fsi_direction_e;

/***********************************************************************
 * HARDWARE ABSTRACTION LAYER (HAL) STRUCTURES
 ***********************************************************************/

/**
 * @struct  int_cfg_t
 * @brief   Configuration of a CPU interrupt
 */
typedef struct __int_config_t__
{
    const uint32_t                      interruptNumber;        /*!< PIE Interrupt Number */
    void                                (*handler)(void);       /*!< ISR to be called when interrupt occurs */
} int_cfg_t;

/**
 * @struct  fsi_evt_cfg_t
 * @brief   Configuration of a single FSI event
 */
typedef struct __fsi_evt_config_t__
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
    const fsi_direction_e               direction;                 /*!< Whether bus is TX or RX */
    const bool_t                        rxLoopback;                /*!< Whether FSI internal loopback enabled (if RX) */
    const float32_t                     rxEOFWatchdogTimeoutUs;    /*!< Frame watchdog (EOF from SOF) timeout uSec (if RX). Set = 0 to disable */
//    const float32_t                     rxPacketTimeoutUs;         /*!< Packet watchdog timeout uSec (if RX), reset whenever any packet is received. Set = 0 to disable */
    const bool_t                        dmaTrigger;                /*!< Whether to generate DMA event on data frame reception (if RX) or transmission (if TX) */
    const uint32_t                      clkPrescaler;              /*!< Clock prescaler from PLLRAWCLK (if TX)*/
    const FSI_DataWidth                 dataWidth;                 /*!< Data width (1 vs 2 lane) for TX or RX operations */
    const fsi_evt_cfg_t*                eventInterrupts;           /*!< Event interrupt configuration */
    // TODO: Add skew compensation configuration.

} fsi_cfg_t;


/**
 * @struct  tmr_cfg_t
 * @brief   Configuration of a CPU Timer
 */
typedef struct __tmr_config_t__
{
    const uint32_t                      timerBase;                  /*!< Timer base address */
    const float32_t                     periodUs;                   /*!< Timer period (uSec) */
    const bool_t                        enableInterrupt;            /*!< Whether to enable interrupt when timer reaches zero*/
} tmr_cfg_t;

/**
 * @struct  dma_cfg_t
 * @brief   Configuration of a DMA channel
 */
typedef struct __dma_config_t__
{
    const uint32_t                      dmaBase;                    /*!< DMA channel address */
    const void                          *destAddr;                  /*!< Destination memory address */
    const void                          *srcAddr;                   /*!< Source memory address */

    // Burst settings:
    const uint16_t                      burstSize;                  /*!< Number of words per burst (16 bit word size) */
    const int16_t                       burstSrcStep;               /*!< Number of words to add to source address after each burst word */
    const int16_t                       burstDestStep;              /*!< Number of words to add to destination address after each burst word */

    // Transfer settings:
    const uint32_t                      transferSize;               /*!< Number of bursts in transfer */
    const int16_t                       transferSrcStep;            /*!< Number of words to add to source address after each burst */
    const int16_t                       transferDestStep;           /*!< Number of words to add to destination address after each burst */

    // Channel settings:
    const DMA_Trigger                   trigger;                    /*!< DMA trigger */
    const uint32_t                      config;                     /*!< Channel configuration */

    const bool_t                        interruptEnable;            /*!< Whether to enable DMA interrupt (on transfer end)*/
    const bool_t                        channelStart;               /*!< Whether to start DMA channel */
} dma_cfg_t;


/**
 * @struct  hal_cfg_t
 * @brief   General HAL structure handler for CPU2
 */
typedef struct __hal_config_t__
{
    const tmr_cfg_t*    p_tmrHandle;
    const int_cfg_t*    p_intHandle;
    const fsi_cfg_t*    p_fsiHandle;
    const dma_cfg_t*    p_dmaHandle;
} hal_cfg_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void HAL_ini(const hal_cfg_t*);

// Expose individual component HAL functions to allow reinitialization
// during runtime:
void HAL_TMR_ini(const tmr_cfg_t*);
void HAL_DMA_ini(const dma_cfg_t*);
void HAL_INT_ini(const int_cfg_t*);
void HAL_FSI_ini(const fsi_cfg_t*);

#endif /* __HAL_CPU2_H__ */
