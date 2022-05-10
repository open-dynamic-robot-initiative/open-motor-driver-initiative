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
 .count             = 0,
 .position          = 0.0,
 .iqRef             = 0,
 .iq                = 0,
 .id                = 0,
 .ia                = 0,
 .ib                = 0,
 .ic                = 0,
 .crc               = 0,
};

//dbg_uart_addr_t dbg_uart_addr =
//{
// .p_position        = NULL,
// .p_iqRef           = NULL,
// .p_iq              = NULL,
// .p_id              = NULL,
// .p_uq              = NULL,
// .p_ud              = NULL,
// .p_ia              = NULL,
// .p_ib              = NULL,
// .p_ic              = NULL,
//};

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
    com_cpu1_to_cm_t* p_cpu1_to_cm_addr = NULL;
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE,
                           &command, (uint32_t *)&p_cpu1_to_cm_addr, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);

    dbg_uart_addr_t dbg_uart_addr =
    {
     .p_position    = (float*)p_cpu1_to_cm_addr[19].data_addr,
     .p_iqRef       = (float*)p_cpu1_to_cm_addr[170].data_addr,
     .p_iq          = (float*)p_cpu1_to_cm_addr[41].data_addr,
     .p_id          = (float*)p_cpu1_to_cm_addr[40].data_addr,
     .p_uq          = (float*)p_cpu1_to_cm_addr[43].data_addr,
     .p_ud          = (float*)p_cpu1_to_cm_addr[42].data_addr,
     .p_ia          = (float*)p_cpu1_to_cm_addr[11].data_addr,
     .p_ib          = (float*)p_cpu1_to_cm_addr[12].data_addr,
     .p_ic          = (float*)p_cpu1_to_cm_addr[13].data_addr,
     //.p_speed       = (float*)p_cpu1_to_cm_addr[26].data_addr;
     //.p_speedMech   = (float*)p_cpu1_to_cm_addr[28].data_addr;
    };

//    dbg_uart_addr.p_position    = (float32_t *)p_cpu1_to_cm_addr[19].data_addr;
//    dbg_uart_addr.p_iqRef       = (float32_t *)p_cpu1_to_cm_addr[170].data_addr;
//    dbg_uart_addr.p_iq          = (float32_t *)p_cpu1_to_cm_addr[41].data_addr;
//    dbg_uart_addr.p_id          = (float32_t *)p_cpu1_to_cm_addr[40].data_addr;
//    dbg_uart_addr.p_uq          = (float32_t *)p_cpu1_to_cm_addr[43].data_addr;
//    dbg_uart_addr.p_ud          = (float32_t *)p_cpu1_to_cm_addr[42].data_addr;
//    dbg_uart_addr.p_ia          = (float32_t *)p_cpu1_to_cm_addr[11].data_addr;
//    dbg_uart_addr.p_ib          = (float32_t *)p_cpu1_to_cm_addr[12].data_addr;
//    dbg_uart_addr.p_ic          = (float32_t *)p_cpu1_to_cm_addr[13].data_addr;

//    dbg_uart_addr.p_speed       = (float32_t *)p_cpu1_to_cm_addr[26].data_addr;
//    dbg_uart_addr.p_speedMech   = (float32_t *)p_cpu1_to_cm_addr[28].data_addr;

    // Loop forever. Wait for IPC interrupts
    while(1)
    {
        IPC_waitForFlag(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
        GPIO_writePin(DBG_PIN2, 1);
        // Create debug message
        com_msgDbgTx(&dbg_uart_addr, &dbg_uart_tx_msg);
        // Acknowledge IT associated to input flag.
        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
        GPIO_writePin(DBG_PIN2, 0);
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
