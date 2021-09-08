/*
 * Filename: hal.h
 * Description: Header file for the Hardware Abstraction Layer
 */

#ifndef __HAL_CM_H__
#define __HAL_CM_H__

#include "driverlib_cm.h"
#include "cm.h"

/**
 * @struct  UART_TypeDef
 * @brief   UART global Hardware Abstraction Layer structure for CM microcontroller.
 */
typedef struct _UART_Cfg_TypeDef_
{
    uint32_t    uartCfg;
    uint32_t    baudRate;
    uint32_t    dmaFlags;
    bool        dmaEnable;
    bool        loopBackEnable;
} UART_TypeDef;


/**
 * @struct  uDMA_TypeDef
 * @brief   uDMA global Hardware Abstraction Layer structure for the CM microcontroller
 */
typedef struct _uDMA_Cfg_TypeDef_
{
    uint32_t    dmaChannel;
    uint32_t    dmaCfg;
//    uint32_t    flagAttr;
//    int32_t     index;
    void*       srcAddr;
    void*       dstAddr;
    uint32_t    transferMode;
    uint32_t    transferSize;
//    uint32_t    transferSize;//    uint32_t dmaRxBase;
//    uint32_t    srcWrapSize;
//    uint32_t    destWrapSize;
//    uint32_t    config;
//    uint16_t    burstSize;
//    int16_t     destStep;
//    int16_t     srcStep;
//    const void* destAddr;
//    const void* srcAddr;
//    DMA_Trigger trigger;
} uDMA_TypeDef;


/**
 * @struct  IT_TypeDef
 * @brief   Interrupt source configuration handler
 */
typedef struct _IT_TypeDef_
{
//    const ID_ENUM   motorID;
    uint32_t    intNum;
    uint32_t    intPeriphBase;
    uint16_t    intPeriphSrc;
    uint16_t    intCpuGroup;
    uint16_t    intAckGroup;
    void        (*handler)(void);
} IT_TypeDef;

typedef struct _IPC_TypeDef_
{
    const IPC_Type_t                ipcType;
    volatile IPC_MessageQueue_t*    p_msgQ;
    const uint32_t                  ipcIntMsgQL;
    const uint32_t                  ipcIntMsgQR;
    const uint32_t                  ipcInt;
    void                            (*ipcIntHandler)(void);
    const bool                      ipcMsgQEnable;
    const bool                      ipcIntEnable;
} IPC_TypeDef;

typedef struct _GCRC_TypeDef_
{
    uint32_t    poly;
    uint32_t    width;
    uint32_t    seed;
    uint32_t    endianness;
    uint32_t    dataType;
    bool        bitRev;
    uint32_t    dataMask;
} CRC_TypeDef;

/**
 * @struct  HAL_CM
 * @brief   General HAL structure handler for the CM microcontroller
 */
typedef struct __HAL_CM__
{
    const UART_TypeDef* p_uartHandler;
    const IPC_TypeDef*  p_ipcHandler;
    const CRC_TypeDef*  p_crcHandler;
    const uDMA_TypeDef* p_udmaHandler;
} HAL_CM;


/*
 * Function prototypes
 */

void HAL_CM_init(const HAL_CM*);
void HAL_CM_IT_init(const IT_TypeDef*);

// ADC interrupt service routine
//extern __interrupt void adc1ISR(void);
//extern __interrupt void adc2ISR(void);
//extern __interrupt void adc3ISR(void);
//extern __interrupt void adc4ISR(void);
//extern __interrupt void isrMotor1(void);

#endif
