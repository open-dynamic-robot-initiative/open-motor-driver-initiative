/*
 * Filename: hal.h
 * Description: Header file for the Hardware Abstraction Layer
 */

#ifndef __HAL_CM_H__
#define __HAL_CM_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "driverlib_cm.h"
#include "cm.h"

/************************************************************************
 * TYPDEF DEFINES
 ************************************************************************/
/**
 * @struct  uart_cfg_t
 * @brief   UART global Hardware Abstraction Layer structure for CM microcontroller.
 */
typedef struct __UART_CFG_t__
{
    uint32_t    uartCfg;
    uint32_t    baudRate;
    uint32_t    dmaFlags;
//    uint32_t    intFlags;
    bool        dmaEnable;
//    bool        intEnable;
//    void        (*p_fnHandler)(void);
} uart_cfg_t;

/**
 * @struct  udma_cfg_t
 * @brief   uDMA global Hardware Abstraction Layer structure for the CM microcontroller
 */
typedef struct __UDMA_CFG_t__
{
    uint32_t    dmaChannel;
    uint32_t    dmaCfg;
    void*       srcAddr;
    void*       dstAddr;
    uint32_t    transferMode;
    uint32_t    transferSize;
} udma_cfg_t;

/**
 * @struct  ipc_cfg_t
 * @brief   IPC global Hardware Abstraction Layer structure for CM microcontroller.
 */
typedef struct __IPC_CFG_t__
{
    const IPC_Type_t                ipcType;
    volatile IPC_MessageQueue_t*    p_ipcMsgQ;
    const uint32_t                  ipcIntMsgQL;
    const uint32_t                  ipcIntMsgQR;
    const uint32_t                  ipcFlag;
    const uint32_t                  ipcInt;
    void                            (*p_ipcIntHandler)(void);
    const bool                      ipcMsgQEnable;
    const bool                      ipcIntEnable;
} ipc_cfg_t;

/**
 * @struct  gcrc_cfg_t
 * @brief   CRC global Hardware Abstraction Layer structure for the CM microcontroller
 */
typedef struct __GCRC_CFG_t__
{
    uint32_t    poly;
    uint32_t    polysize;
    uint32_t    seed;
    uint32_t    endianness;
    uint32_t    dataType;
    uint32_t    dataMask;
    bool        bitRev;
} gcrc_cfg_t;

/**
 * @struct  hal_cm_t
 * @brief   General HAL structure handler for the CM microcontroller
 */
typedef struct __HAL_CM__
{
    const uart_cfg_t*   p_uartHandle;
    const ipc_cfg_t*    p_ipcHandle;
    const gcrc_cfg_t*   p_crcHandle;
    const udma_cfg_t*   p_udmaHandle;
} hal_cm_t;

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
void HAL_CM_init(const hal_cm_t*);

#endif
