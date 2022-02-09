//
// Included Files
//
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <string.h>
//#include <float.h>

/***********************************************************************.
 * INCLUDE FILES
 ************************************************************************/
#include "main_cm.h"

/***********************************************************************.
 * DEFINES
 ************************************************************************/
#define IPC_CMD_M1      0x1001
#define IPC_CMD_M2      0x2001

/************************************************************************
 * FUNCTIONS DEFINITIONS
 ************************************************************************/
/**
 * @brief   Main function running on CM processor. MCU init.
 */
void main(void)
{
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);
    // Loop forever. Wait for IPC interrupts
    while(1);
}

/**
 * @brief   IPC interrupt. When new foc structure is avaolable, create message and sned it by uart
 */
__attribute__((interrupt)) void ipc_isr0(void)
{
    uint32_t    cmd, data, addr;
    uint8_t     ipcCfgIdx = 0;
    uint8_t     udmaCfgIdx = 0;
    // Get index associated to IPC structure configuration
    while((cm_ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isr0) ? (ipcCfgIdx++) : (ipcCfgIdx));
    // Get structure address
    IPC_readCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlag,
                    IPC_ADDR_CORRECTION_ENABLE, &cmd, &addr, &data);
    // Create debug message
    com_msgDbgTx((foc_t *)addr, dbg_uart_tx_msg);
    // Wait Tx buffer + UART Tx module are empty.
    while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY) == 1);
    // Force purge of Rx buffer.
    while(UART_readCharNonBlocking(UART0_BASE) != -1);
    // Get the index associated to uDMA UART0_TX configuration
    while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
    // Enable the uDMA UART0 Tx channel
    if(!UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_TX))
    {
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= cm_udmaCfgList[udmaCfgIdx].transferMode;
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
    }
    // Acknowledge IT associated to input flag.
    IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlag);
}

//
// End of File
//
