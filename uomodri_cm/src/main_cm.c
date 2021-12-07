//
// Included Files
//
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <string.h>
//#include <float.h>

#include "main_cm.h"
//#include "uomodri_prj_config_handlers.h"

uint64_t    interruptCnt_M1 = 0;
uint64_t    interruptCnt_M2 = 0;

#define TIME_M1_S   ((float32_t)interruptCnt_M1 * dt)
#define TIME_M1_MS  ((float32_t)interruptCnt_M1 * 1000.0f * dt)
#define TIME_M2_S   ((float32_t)interruptCnt_M2 * dt)
#define TIME_M2_MS  ((float32_t)interruptCnt_M2 * 1000.0f * dt)

#define IPC_CMD_READ_MEM    0x1001
#define MSG_COM_INTER       0x2001
#define MSG_COM_ADDR        0x3001

//typedef float         float32_t;

/*********************************************************************************/
/**                                FOC STRUCTURE                                **/
/*********************************************************************************/

//
// Defines
//
//#define IPC_CMD_READ_MEM    0x1001
//#define IPC_CMD_RESP        0x2001
//
//#define TEST_PASS           0x5555
//#define TEST_FAIL           0xAAAA
//
//#define DEBUG_LED_PIN2      34
//#define DEBUG_LED_PIN2_CFG  GPIO_34_GPIO34

//__interrupt void UART_RX_IntHandler(void);



// Sine for debug
const float32_t     debugFreq = 1.0f;
const float32_t     dt = 1.0 / PWM_FREQ;



/**
 * @brief   IT associated to
 */
//__interrupt void IPC_ISR1(void)
//{
//    uint32_t    command, data, addr;
//    uint8_t     ipcCfgIndex     = 0;
//
//    // Get index associated to IPC structure configuration
//    while((cm_ipcCfgList[ipcCfgIndex].p_ipcIntHandler != &IPC_ISR1) ? (ipcCfgIndex++) : (ipcCfgIndex));
//    // Get structure address
//    IPC_readCommand(cm_ipcCfgList[ipcCfgIndex].ipcType, cm_ipcCfgList[ipcCfgIndex].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data);
//    // Extract message from Master
//    //cmd_extract((rx_msg_t*)addr, &cmd_uomodri);
//    // Acknowledge IT
//    IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIndex].ipcType, cm_ipcCfgList[ipcCfgIndex].ipcFlag);
//}

//__interrupt void UART_RX_IntHandler(void)
//{
//    uint32_t ui32Status;
//
//    // Get the interrupt status.
//    ui32Status = UART_getInterruptStatus(UART0_BASE, UART_RAW_INT);
//    // Clear the asserted interrupts.
//    UART_clearInterruptStatus(UART0_BASE, ui32Status);
//    // Loop while there are characters in the receive FIFO.
//    while(UART_isDataAvailable(UART0_BASE))
//        // Read the next character from the UART and write it back to the UART.
//        UART_writeCharNonBlocking(UART0_BASE, UART_readCharNonBlocking(UART0_BASE));
//}


/**
 * main_cm.c
 */
void main(void)
{
    uint8_t ipcCfgIdx = 0;
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);

    // Get index associated to IPC structure configuration
    while(cm_ipcCfgList[(cm_ipcCfgList[ipcCfgIdx].p_ipcIntHandler != NULL) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != NULL);
    // Share the message generated with C28-CPU1
    IPC_sendCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlagOut, IPC_ADDR_CORRECTION_DISABLE, MSG_COM_ADDR, (uint32_t)(&com_rx_cmd), 1);

    // Loop forever. Wait for IPC interrupts
    while(1);
}

__attribute__((interrupt)) void ipc_isr0(void)
{
    uint32_t    cmd, data, addr;
    uint8_t     ipcCfgIdx = 0;
    bool        com_tx_msg_done = false;
    // Get index associated to IPC structure configuration
    while(cm_ipcCfgList[(cm_ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isr0) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != &ipc_isr0);
    // Get structure address
    IPC_readCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlagIn, IPC_ADDR_CORRECTION_DISABLE, &cmd, &addr, &data);
    // Create the reply message
    com_tx_msg_done = com_msgTx((FOC_STRUCT *)addr, &com_tx_msg);
#ifdef DEBUG
    uint8_t udmaCfgIndex = 0;
    // Create debug message
    com_msgDbgTx((FOC_STRUCT *)addr, dbg_tx_msg);
    // Wait Tx buffer + UART Tx module are empty.
    while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY) == 1);
    // Force purge of Rx buffer.
    while(UART_readCharNonBlocking(UART0_BASE) != -1);
    // Get the index associated to uDMA UART0_TX configuration
    while((cm_udmaCfgList[udmaCfgIndex].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIndex++) : (udmaCfgIndex));
    // Enable the uDMA UART0 Tx channel
    if(!UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_TX))
    {
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= cm_udmaCfgList[udmaCfgIndex].transferMode;
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[udmaCfgIndex].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
    }
#endif /*DEBUG*/
    if(com_tx_msg_done == true)
        // Share the message generated with C28-CPU1
        IPC_sendCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlagOut, IPC_ADDR_CORRECTION_DISABLE, MSG_COM_INTER, (uint32_t)(&com_tx_msg), 1);
    // Acknowledge IT associated to input flag.
    IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlagIn);
}

/**
 * @brief   IT function active on a new message sent by Master.
 */
__attribute__((interrupt)) void ipc_isr1(void)
{
    uint32_t    cmd, data, addr;
    uint8_t     ipcCfgIdx = 0;
    bool        com_rx_msg_valid = false;
    // Get index associated to IPC structure configuration
    while(cm_ipcCfgList[(cm_ipcCfgList[ipcCfgIdx].p_ipcIntHandler != &ipc_isr1) ? (ipcCfgIdx++) : (ipcCfgIdx)].p_ipcIntHandler != &ipc_isr1);
    // Get structure address
    IPC_readCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlagIn, IPC_ADDR_CORRECTION_DISABLE, &cmd, &addr, &data);
    // Extract data from the command message
    com_rx_msg_valid = com_cmdRx((rx_msg_t *)addr, &com_rx_cmd);
    if(com_rx_msg_valid == true)
        // Share the message generated with C28-CPU1
        IPC_sendCommand(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlagOut, IPC_ADDR_CORRECTION_DISABLE, MSG_COM_INTER, (uint32_t)(&com_rx_cmd), 1);
    // Acknowledge IT associated to input flag.
    IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlagIn);
}

//
// End of File
//
