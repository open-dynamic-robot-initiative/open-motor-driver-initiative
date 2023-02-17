//
// Included Files
//
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <string.h>
//#include <float.h>

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "main_cm.h"
//#include "foc.h"
#include "communication.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
inline void IPC_CpuToCmAddrSync(dbg_uart_addr_t*);

/************************************************************************
 * LOCAL VARIABLES
 ************************************************************************/
#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];
dbg_uart_msg_t dbg_uart_tx_msg =
{
 .header[0]         = 'a',
 .header[1]         = 'b',
// .header[2]         = 'c',
// .count             = 0U,
 .ia                = 0.0f,
 .ib                = 0.0f,
 .ic                = 0.0f,
// .vbus              = 0.0f,
// .ialpha            = 0.0f,
// .ibeta             = 0.0f,
 .id                = 0.0f,
 .iq                = 0.0f,
 .iqref             = 0.0f,
 .ud                = 0.0f,
 .uq                = 0.0f,
// .ualpha            = 0.0f,
// .ubeta             = 0.0f,
// .ua                = 0.0f,
// .ub                = 0.0f,
// .uc                = 0.0f,
 .pos               = 0.0f,
// .posref            = 0.0f,
 .vel               = 0.0f,
// .velref            = 0.0f,
// .itcnt             = 0U,
 .crc               = 0,
// .padding           = 0,
};

dbg_uart_addr_t dbg_uart_addr =
{
 .p_ia              = NULL,
 .p_ib              = NULL,
 .p_ic              = NULL,
 .p_vbus            = NULL,
 .p_ialpha          = NULL,
 .p_ibeta           = NULL,
 .p_id              = NULL,
 .p_iq              = NULL,
 .p_ud              = NULL,
 .p_uq              = NULL,
 .p_ualpha          = NULL,
 .p_ubeta           = NULL,
 .p_ua              = NULL,
 .p_ub              = NULL,
 .p_uc              = NULL,
 .p_pos             = NULL,
 .p_posref          = NULL,
 .p_vel             = NULL,
 .p_velref          = NULL,
 .p_iqref           = NULL,
// .p_itcnt           = NULL,
};

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
extern const hal_cm_t hal_cm;

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief   Main function running on CM processor. MCU init.
 */
void main(void)
{
    uint32_t i = 0;
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);
    // IPC address translation
    IPC_CpuToCmAddrSync(&dbg_uart_addr);
//    uint32_t command, data;
//    // Get address of ia of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ia, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ib of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ib, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ic of MOTOR_2
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ic, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of vbus of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_vbus, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ialpha of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ialpha, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ibeta of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ibeta, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of id of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_id, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of iq of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_iq, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ud of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ud, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of uq of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_uq, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ualpha of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ualpha, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ubeta of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ubeta, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ua of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ua, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of ub of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_ub, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of uc of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_uc, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of absolute encoder position of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_pos, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of PositionRef of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_posref, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of encoder speed variable of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_vel, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of SpeedRef of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_velref, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of IqRef of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_iqref, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//    // Get address of IT counter of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&dbg_uart_addr.p_itcnt, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);

    // Loop forever. Wait for IPC interrupts
    while(1)
    {
        IPC_waitForFlag(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//        DBG_PIN1_SET;
//        GPIO_writePin(EXT_DBG_SPI_SOMI, 1);
//        EXT_DBG_SPI_SOMI_SET;
        GPIO_writePin(EXT_DBG_SPI_SOMI, 1);
        // Create debug message
        COM_msgDbgTx(&dbg_uart_addr, &dbg_uart_tx_msg);
        // Acknowledge IT associated to input flag.
        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//        DBG_PIN0_CLEAR;
//        EXT_DBG_SPI_SIMO_CLEAR;
//        GPIO_writePin(EXT_DBG_SPI_SIMO, 0);
//        EXT_DBG_SPI_SOMI_CLEAR;
        GPIO_writePin(EXT_DBG_SPI_SOMI, 0);
        if((i % 4) == 0)
        {
            // Wait Tx buffer + UART Tx module are empty.
            while(UART_isBusy(UART0_BASE));
            // Force purge of Rx buffer.
            while(UART_readCharNonBlocking(UART0_BASE) != -1);
            // Get the index associated to uDMA UART0_TX configuration
            //    while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
            // Enable the uDMA UART0 Tx channel
            //        while(UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel));
            //        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
            //        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
            //        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
            //        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
            if(!UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel))
            {
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
                UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
            }
        }
        i++;
//        // Acknowledge IT associated to input flag.
//        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    }
}

inline void IPC_CpuToCmAddrSync(dbg_uart_addr_t* p_addr)
{
    uint32_t command;// = NULL;
    uint32_t addr;// = NULL;
    uint32_t data;// = NULL;

    // Get address of ia of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    p_addr->p_ia        = (float_t*)addr;
    p_addr->p_ib        = (float_t*)addr + 1;
    p_addr->p_ic        = (float_t*)addr + 2;
    p_addr->p_vbus      = (float_t*)addr + 3;
    p_addr->p_ialpha    = (float_t*)addr + 4;
    p_addr->p_ibeta     = (float_t*)addr + 5;
    p_addr->p_id        = (float_t*)addr + 6;
    p_addr->p_iq        = (float_t*)addr + 7;
    p_addr->p_ud        = (float_t*)addr + 8;
    p_addr->p_uq        = (float_t*)addr + 9;
    p_addr->p_ualpha    = (float_t*)addr + 10;
    p_addr->p_ubeta     = (float_t*)addr + 11;
    p_addr->p_ua        = (float_t*)addr + 12;
    p_addr->p_ub        = (float_t*)addr + 13;
    p_addr->p_uc        = (float_t*)addr + 14;
    p_addr->p_pos       = (float_t*)addr + 15;
    p_addr->p_posref    = (float_t*)addr + 16;
    p_addr->p_vel       = (float_t*)addr + 17;
    p_addr->p_velref    = (float_t*)addr + 18;
    p_addr->p_iqref     = (float_t*)addr + 19;
#if (0)
    uint32_t command, data;
    // Get address of ia of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ia, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ib of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ib, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ic of MOTOR_2
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ic, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of vbus of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_vbus, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ialpha of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ialpha, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ibeta of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ibeta, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of id of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_id, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of iq of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_iq, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ud of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ud, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of uq of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_uq, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ualpha of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ualpha, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ubeta of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ubeta, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ua of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ua, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of ub of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_ub, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of uc of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_uc, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of absolute encoder position of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_pos, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of PositionRef of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_posref, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of encoder speed variable of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_vel, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of SpeedRef of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_velref, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of IqRef of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_iqref, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of IT counter of MOTOR_1
//    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, (uint32_t *)&p_addr->p_itcnt, &data));
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
#endif
}

//
// End of File
//
