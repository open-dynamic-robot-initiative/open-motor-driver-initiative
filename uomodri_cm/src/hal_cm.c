/*
 * File name: hal.c
 * Description: Source file containing Hardware Abstraction Layer source code
 */
/***********************************************************************.
 * INCLUDE FILES
 ************************************************************************/
#include "hal_cm.h"
#include "uomodri_cm_user_defines.h"

/************************************************************************
 * LOCAL FUNCTIONS DECLARATION
 ************************************************************************/
void HAL_CM_UART_ini(const uart_cfg_t*);
void HAL_CM_uDMA_ini(const udma_cfg_t*);
void HAL_CM_IPC_ini(const ipc_cfg_t*);
void HAL_CM_CRC_ini(const gcrc_cfg_t*);
void HAL_CM_USB_ini(usb_cfg_t*);
//void HAL_INT_ini(const int_cfg_t*);

extern UDMA_ControlTable ucControlTable[32];

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Initialize all peripherals necessaries.
 * @param[in]   *p_hal  Pointer on the general HAL structure definition
 */
void HAL_CM_ini(const hal_cm_t* p_hal_cm)
{
    if(p_hal_cm != NULL)
    {
        // Initialize DMA
        HAL_CM_uDMA_ini(p_hal_cm->p_udmaHandle);
        // Initialize CM_UART
        HAL_CM_UART_ini(p_hal_cm->p_uartHandle);
        // Initialize CM_CRC
        HAL_CM_CRC_ini(p_hal_cm->p_crcHandle);
        // Initialize IPC communication
        HAL_CM_IPC_ini(p_hal_cm->p_ipcHandle);
        // Initialize  USB
        HAL_CM_USB_ini(p_hal_cm->p_usbHandle);
    }

    return;
}

/**
 * @brief       DMA initialization function.
 * @param[in]   *p_dmaCfg   Pointer on the DMA configuration structure
 */
void HAL_CM_uDMA_ini(const udma_cfg_t* p_udmaCfg)
{
    if(p_udmaCfg != NULL)
    {
        uint32_t dmaChannel = p_udmaCfg->dmaChannel;

        // Enable uDMA
        UDMA_enable(UDMA_BASE);
        // Point at the control table to use for channel control structures.
        UDMA_setControlBase(UDMA_BASE, ucControlTable);

        while(dmaChannel != UINT32_MAX)
        {
            /* Put the attributes in a known state for the uDMA software channel.
             * These should already be disabled by default. */            //
            UDMA_disableChannelAttribute(UDMA_BASE, dmaChannel, UDMA_CH_ATTR_ALL);
//          UDMA_CH_ATTR_ALTSELECT | UDMA_CH_ATTR_HIGH_PRIORITY | UDMA_CH_ATTR_REQMASK);
            //UDMA_enableChannelAttribute(UDMA_BASE, dmaChannel, UDMA_CH_ATTR_USEBURST);
            /* Configure the control parameters for the UART TX/RX channels
             * Tx channel will be used to transfer data from the buffer to the UART Data register.
             * Data size = 8
             * Source address increment = 8      (data to be read from 8-bit buffer)
             * Destination address increment = 0 (data to be copied to a single address location)
             *
             * The arbitration size will be set to 1, which causes the uDMA controller
             * to re-arbitrate after 1 item is transferred. */
            UDMA_setChannelControlParams(UDMA_BASE, dmaChannel, p_udmaCfg->dmaCfg);
            UDMA_setChannelTransferParams(UDMA_BASE, dmaChannel, p_udmaCfg->srcAddr, \
                                          p_udmaCfg->dstAddr, p_udmaCfg->transferMode, \
                                          p_udmaCfg->transferSize);
            UDMA_enableChannel(UDMA_BASE, dmaChannel);
            dmaChannel = (++p_udmaCfg)->dmaChannel;
        }
    }

    return;
}

/**
 * @brief       UART initialization function.
 *              There is no need to increase uartbase as we have only one UART periph on CM.
 * @param[in]   *p_uart     Pointer on the UART structure definition
 */
void HAL_CM_UART_ini(const uart_cfg_t* p_uartCfg)
{
    if(p_uartCfg != NULL)
    {
        UART_setFIFOLevel(UART0_BASE, UART_FIFO_TX6_8, UART_FIFO_RX6_8);
        UART_enableFIFO(UART0_BASE);
        UART_disableLoopback(UART0_BASE);
        UART_enableDMA(UART0_BASE, p_uartCfg->dmaFlags);
        // Configure UART & Enable the module.
        UART_setConfig(UART0_BASE, UART_CLK_FREQ, p_uartCfg->baudRate, p_uartCfg->uartCfg);
        UART_clearRxError(UART0_BASE);
        // Enable the UART0 module.
        UART_enableModule(UART0_BASE);
    }

    return;
}

/**
 * @brief       IPC (InterProcessor Communications) initialization function.
 * @param[in]   *p_ipcCfg   Pointer on the IPC configuration structure
 */
void HAL_CM_IPC_ini(const ipc_cfg_t* p_ipcCfg)
{
    if(p_ipcCfg != NULL)
    {
        IPC_Type_t  ipcType     = p_ipcCfg->ipcType;
        /* (0) - IPC_CM_L_CPU1_R, (1) - IPC_CM_L_CPU2_R */
        uint8_t     ipcSyncDone = 0;
        while(ipcType != IPC_TOTAL_NUM)
        {
            // Enable IPC interrupts
            if(p_ipcCfg->ipcIntEnable)
                IPC_registerInterrupt(ipcType, \
                                      p_ipcCfg->ipcInt, \
                                      p_ipcCfg->p_ipcIntHandler);
            // Initialize message queue
            if(p_ipcCfg->ipcMsgQEnable)
                IPC_initMessageQueue(ipcType, \
                                     p_ipcCfg->p_ipcMsgQ, \
                                     p_ipcCfg->ipcIntMsgQL, \
                                     p_ipcCfg->ipcIntMsgQR);
            // Synchronize both the cores.
            if(!(ipcSyncDone & (1 << ipcType)))
            {
                // Clear any IPC flags if set already
                IPC_clearFlagLtoR(ipcType, IPC_FLAG_ALL);
                // Synchronize both the cores.
                IPC_sync(ipcType, IPC_SYNC_FLAG);
                ipcSyncDone |= (1 << ipcType);
            }
            ipcType = (++p_ipcCfg)->ipcType;
        }
    }

    return;
}

/**
 * @brief       Generic Cyclic Redundancy Check (GCRC) initialization function.
 * @param[in]   *p_crcCfg   Pointer on the CRC configuration structure
 */
void HAL_CM_CRC_ini(const gcrc_cfg_t* p_crcCfg)
{
    if(p_crcCfg != NULL)
    {
        // Configure the GCRC module
        GCRC_setPolynomial(GCRC_BASE, p_crcCfg->poly, p_crcCfg->polysize);
        GCRC_setDataType(GCRC_BASE, p_crcCfg->dataType);
        GCRC_setDataEndianness(GCRC_BASE, p_crcCfg->endianness);
        GCRC_setDataMask(GCRC_BASE, p_crcCfg->dataMask);
        GCRC_enableBitReverse(GCRC_BASE, p_crcCfg->bitRev);
        GCRC_setSeedValue(GCRC_BASE, p_crcCfg->seed);
    }

    return;
}

/**
 * @brief       USB initialization function.
 *              There is no need to increase usbbase as we have only one USB periph on CM.
 * @param[in]   *p_crcCfg   Pointer on the CRC configuration structure
 */
void HAL_CM_USB_ini(usb_cfg_t* p_usbCfg)
{
    if(p_usbCfg != NULL)
    {
//    p_usbCfg->usbCfgDone = false;
// Register the interrupt handler, for USB.
//    Interrupt_registerHandler(INT_USB0, &CM_USB0DeviceIntHandler);
// Initialize the transmit and receive buffers for first serial device.
        USBBufferInit(p_usbCfg->p_usbTxBuf);
        USBBufferInit(p_usbCfg->p_usbRxBuf);
        // Set the USB stack mode to Device mode with VBUS monitoring.
        USBStackModeSet(0, eUSBModeForceDevice, 0);//eUSBModeDevice
        // Pass our device information to the USB library and place the device on the bus.
        USBDCDCInit(0, p_usbCfg->p_usbCdcDevice);

//    p_usbCfg->usbCfgDone = true;
    }

    return;
}

/**
 * @brief       Interrupts initialization function.
 * @param[in]   *p_intCfg   Pointer on the interrupt configuration list structure
 */
void HAL_INT_ini(const int_cfg_t* p_intCfg)
{
    if(p_intCfg != NULL)
    {
        uint32_t intNum = p_intCfg->intNum;

        // Clear & set IT flags & enable IT bits in a known state.
        while(intNum != UINT32_MAX)
        {
            // Attach ISR to interrupt
            Interrupt_registerHandler(p_intCfg->intNum, p_intCfg->p_intHandler);
            // Enable Interrupts
            if(p_intCfg->intEnable)
                Interrupt_enable(p_intCfg->intNum);

            intNum = (++p_intCfg)->intNum;
        }

        Interrupt_enableInProcessor();
    }

    return;
}
