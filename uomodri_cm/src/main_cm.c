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

extern const hal_cm_t hal_cm;
/***********************************************************************
 * DEFINES
 ***********************************************************************/

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
 .count             = 0U,
 .pos_m1            = 0.0f,
// .vel_m1            = 0.0f,
 .pos_m2            = 0.0f,
// .vel_m2            = 0.0f,
 .pos_as5047u       = 0.0f,
 .vel_as5047u       = 0.0f,
 .crc               = 0,
};

dbg_uart_addr_t dbg_uart_addr =
{
 .p_pos_m1          = NULL,
 .p_vel_m1          = NULL,
 .p_pos_m2          = NULL,
 .p_vel_m2          = NULL,
 .p_pos_as5047u     = NULL,
 .p_vel_as5047u     = NULL,
};

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief   Main function running on CM processor. MCU init.
 */
void main(void)
{
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);

    uint32_t command, data;

    // Get address of optical encoder position variable of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE,
                           &command, (uint32_t *)&dbg_uart_addr.p_pos_m1, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of optical encoder speed variable of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE,
                           &command, (uint32_t *)&dbg_uart_addr.p_vel_m1, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of magnetic encoder position variable of MOTOR_2
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE,
                           &command, (uint32_t *)&dbg_uart_addr.p_pos_m2, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    // Get address of magnetic encoder speed variable of MOTOR_2
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE,
                           &command, (uint32_t *)&dbg_uart_addr.p_vel_m2, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);

    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE,
                           &command, (uint32_t *)&dbg_uart_addr.p_pos_as5047u, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE,
                           &command, (uint32_t *)&dbg_uart_addr.p_vel_as5047u, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);

    // Loop forever. Wait for IPC interrupts
    while(1)
    {
        IPC_waitForFlag(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
        // Create debug message
        com_msgDbgTx(&dbg_uart_addr, &dbg_uart_tx_msg);
        // Acknowledge IT associated to input flag.
        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
        // Wait Tx buffer + UART Tx module are empty.
        while(UART_isBusy(UART0_BASE));
        // Force purge of Rx buffer.
        while(UART_readCharNonBlocking(UART0_BASE) != -1);
        // Get the index associated to uDMA UART0_TX configuration
        //    while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
        // Enable the uDMA UART0 Tx channel
        if(!UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel))
        {
            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
            ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
            UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
        }
    }
}

//
// End of File
//
