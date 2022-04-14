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

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define POS_M1_LSB      (0)
#define POS_M1_MSB      (1)
#define CMD_IQREF       (2)
#define FOC_IQ_SENS     (3)
#define FOC_ID_SENS     (4)
#define FOC_DTC_U       (5)
#define FOC_DTC_V       (6)
#define FOC_DTC_W       (7)

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

    uint32_t    command = POS_M1_LSB, addr, data;

    while(command < FOC_DTC_W)
    {
        while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data));
        if(command == POS_M1_LSB)
            dbg_uart_addr.position_lsb          = (uint16_t *)addr;
        else if(command == POS_M1_MSB)
            dbg_uart_addr.position_msb          = (uint16_t *)addr;
        else if(command == CMD_IQREF)
            dbg_uart_addr.iqRef                 = (float32_t *)addr;
        else if(command == FOC_IQ_SENS)
            dbg_uart_addr.iqMeas                = (float32_t *)addr;
        else if(command == FOC_ID_SENS)
            dbg_uart_addr.idMeas                = (float32_t *)addr;
        else if(command == FOC_DTC_U)
            dbg_uart_addr.dtc_u                 = (float32_t *)addr;
        else if(command == FOC_DTC_V)
            dbg_uart_addr.dtc_v                 = (float32_t *)addr;
        else if(command == FOC_DTC_W)
            dbg_uart_addr.dtc_w                 = (float32_t *)addr;
//        else if(command == FOC_FSM_M2)
//            dbg_uart_addr.stateMachine[MOTOR_2] = (fsm_reg_t *)addr;
        // Acknowledge the flag
        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    }

    // Loop forever. Wait for IPC interrupts
    while(1)
    {
        IPC_waitForFlag(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
        // Create debug message
//        com_msgDbgTx((foc_t *)foc_m1_addr, dbg_uart_tx_msg);
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

/**
 * @brief   IPC interrupt. When new foc structure is available, create message and send it by uart
 */
//__attribute__((interrupt)) void ipc_isr0(void)
//{
//    const ipc_cfg_t*        p_ipcCfg        = &cm_ipcCfgList[0];
//    const udma_cfg_t*       p_udmaCfg       = &cm_udmaCfgList[0];
////    const IPC_Instance_t*   p_ipcInstance   = &IPC_Instance[p_ipcCfg->ipcType];
//    uint32_t*               p_foc_m1        = NULL;//, p_foc_m2 = NULL, p_msg_status = NULL;
//    uint32_t*               p_foc_m2        = NULL;//, p_msg_status = NULL;
//    uint32_t*               p_msg_status    = NULL;
////    uint32_t    cmd, data, addr;
//    // Get structure address
////    *p_foc_m1       = p_ipcInstance->IPC_MsgRam_RtoL + IPC_ADDR_OFFSET_CORR(addrReg, p_ipcInstance->IPC_Offset_Corr);
////    *p_foc_m2       = IPC_Instance[p_ipcCfg->ipcType].IPC_MsgRam_RtoL + IPC_ADDR_OFFSET_CORR(addrReg, IPC_Instance[p_ipcCfg->ipcType].IPC_Offset_Corr);
////    *p_msg_status   = IPC_Instance[p_ipcCfg->ipcType].IPC_MsgRam_RtoL + IPC_ADDR_OFFSET_CORR(addrReg, IPC_Instance[p_ipcCfg->ipcType].IPC_Offset_Corr);
//    IPC_readCommand(p_ipcCfg->ipcType, p_ipcCfg->ipcFlag, IPC_ADDR_CORRECTION_ENABLE, p_foc_m1, p_foc_m2, p_msg_status);
//    // Create debug message
//    com_msgDbgTx((foc_t *)p_foc_m1, dbg_uart_tx_msg);
//
//    // Wait Tx buffer + UART Tx module are empty.
//    while(UART_isBusy(UART0_BASE));
//    // Force purge of Rx buffer.
//    while(UART_readCharNonBlocking(UART0_BASE) != -1);
//    // Get the index associated to uDMA UART0_TX configuration
//    //    while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
//    // Enable the uDMA UART0 Tx channel
//    if(!UDMA_isChannelEnabled(UDMA_BASE, p_udmaCfg->dmaChannel))
//    {
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((p_udmaCfg->transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
//        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((p_udmaCfg->transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
//        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
//    }
//    // Acknowledge IT associated to input flag.
//    IPC_ackFlagRtoL(p_ipcCfg->ipcType, p_ipcCfg->ipcFlag);
//    //IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIdx].ipcType, cm_ipcCfgList[ipcCfgIdx].ipcFlag);
//}

//
// End of File
//
