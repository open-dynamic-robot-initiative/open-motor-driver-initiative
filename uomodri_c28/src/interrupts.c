/*
 * File name: interrupts.c
 * Description: Source file containing IT functions running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "driverlib.h"
#include "device.h"
#include "board.h"

#include "uomodri_user_defines.h"
#include "config/uomodri_math.h"

#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
#include "config/uomodri_1/uomodri_1_handlers.h"    // uOmodri v1.x configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
#include "config/uomodri_2/uomodri_2_handlers.h"    // uOmodri v2.x configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
#include "config/uomodri_3/uomodri_3_handlers.h"    // uOmodri v3.x configuration include file
#endif

#include "communication.h"
//#include "motor_ctrl.h"
//#include "main.h"
//#include "hal.h"

/***********************************************************************
 * VARIABLES IN CLA SPACE MEMORY
 ***********************************************************************/
#pragma DATA_ALIGN(motor_m1, 4)
#pragma DATA_SECTION(motor_m1, "ClaDataMotor1");
motor_t     motor_m1;

#pragma DATA_ALIGN(motor_m2, 4)
#pragma DATA_SECTION(motor_m2, "ClaDataMotor2");
motor_t     motor_m2;

#pragma DATA_ALIGN(robot_state, 4)
#pragma DATA_SECTION(robot_state, "ClaDataRobotState");
robot_state_t recv_robot_state;

/***********************************************************************
 * VARIABLES IN CPU TO CLA SPACE MEMORY
 ***********************************************************************/
#pragma DATA_ALIGN(cmd_uomodri, 4)
#pragma DATA_SECTION(cmd_uomodri, "MsgRamCpuToCla")
cmd_t       cmd_uomodri[2];

/***********************************************************************
 * VAIRIABLES IN CPU TO CM MEMORY
 ***********************************************************************/
#pragma DATA_ALIGN(dbg_cpu2cm_m1, 128U)
#pragma DATA_SECTION(dbg_cpu2cm_m1, "MsgRamCpuToCm")
debug_t     dbg_cpu2cm_m1 = {.motor_id = MOTOR_1};

#pragma DATA_ALIGN(dbg_cpu2cm_m2, 128U)
#pragma DATA_SECTION(dbg_cpu2cm_m2, "MsgRamCpuToCm")
debug_t     dbg_cpu2cm_m2 = {.motor_id = MOTOR_2};

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
#pragma DATA_ALIGN(com_spi_rx, 4)
#pragma DATA_SECTION(com_spi_rx, "ramgs0")
spi_rx_t    com_spi_rx;

#pragma DATA_ALIGN(com_spi_tx, 4)
#pragma DATA_SECTION(com_spi_tx, "ramgs0")
spi_tx_t    com_spi_tx      = {.msg_cr = 0U, .msg_nx = 1U};

#pragma DATA_ALIGN(com_rs485_rx, 4)
#pragma DATA_SECTION(com_rs485_rx, "ramgs0")
rs485_rx_t  com_rs485_rx    = {.index = 0U, \
                               .addr1 = false, .addr2 = false};

#pragma DATA_ALIGN(com_rs485_tx, 4)
#pragma DATA_SECTION(com_rs485_tx, "ramgs0")
rs485_tx_t  com_rs485_tx    = {.index = 0U, \
                               .msg_cr = 0U, .msg_nx = 1U, \
                               .msg[0].crc.u16_msb = 0x8CA0, \
                               .msg[0].crc.u16_lsb = 0x22CF, \
                               .msg[1].crc.u16_msb = 0x8CA0, \
                               .msg[1].crc.u16_lsb = 0x22CF };

#pragma DATA_ALIGN(com_canfd_tx, 4)
#pragma DATA_SECTION(com_canfd_tx, "ramgs0")
canfd_tx_t  com_canfd_tx;

#pragma DATA_ALIGN(com_canfd_rx, 4)
#pragma DATA_SECTION(com_canfd_rx, "ramgs0")
canfd_rx_t  com_canfd_rx;

#pragma DATA_ALIGN(led_spi_tx, 4)
#pragma DATA_SECTION(led_spi_tx, "ramgs0")
uint16_t    led_spi_tx[LED_MSG_TX_16BIT_LENGTH];

#pragma DATA_ALIGN(cpu_tmr_cnt_ms, 4)
#pragma DATA_SECTION(cpu_tmr_cnt_ms, "ramgs0")
uint32_t    cpu_tmr_cnt_ms = 0U;

#pragma DATA_ALIGN(errata_cnt, 4)
#pragma DATA_SECTION(errata_cnt, "ramgs0")
uint32_t        errata_cnt   = 0U;

#pragma DATA_ALIGN(mcan_error, 4)
#pragma DATA_SECTION(mcan_error, "ramgs0")
int32_t         mcan_error = 0;

#pragma DATA_ALIGN(slv2mst_msg, 4)
#pragma DATA_SECTION(slv2mst_msg, "ramgs0")
slv2mst_msg_t   slv2mst_msg       = {.status      = 0U, \
                                   .timeStamp   = 0U, \
                                   .position    = {0U, 0U}, \
                                   .velocity    = {0, 0}, \
                                   .current     = {0, 0}, \
                                   .coilRes     = {0U, 0U}, \
                                   .adcSamples  = {0U, 0U}, \
                                   .lastCmdIndex= 0U};

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
extern com_bus_e        com_bus_select;

/***********************************************************************
 * CONSTANTS IN FLASH
 ***********************************************************************/
#pragma DATA_ALIGN(com_msg_rs485_tx_master_address, 4)
#pragma DATA_SECTION(com_msg_rs485_tx_master_address, "ConstFlashCom")
uint16_t com_msg_rs485_tx_master_address = COM_MSG_RS485_TX_MASTER_ADDRESS;

#pragma DATA_ALIGN(com_msg_canfd_tx_master_id, 4)
#pragma DATA_SECTION(com_msg_canfd_tx_master_id, "ConstFlashCom")
uint16_t com_msg_canfd_tx_master_id = COM_MSG_CANFD_TX_MASTER_ID;


/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void DBG_copyStruct(motor_t*, debug_t*);
inline void LED_dimming(motor_t*, motor_t*, uint16_t*);

/***********************************************************************
 * INTERRUPT HANDLING FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Generate part of the Tx message associated after MOTOR_1 control.
 */
__attribute__((interrupt)) void cla_isrMotor1(void)
{
    DBG_CLA_M1_ISR_SET;
    // Fill of the M1 motor's elements in the Tx message
    COM_msgCreate(&motor_m1, &slv2mst_msg, &cpu_tmr_cnt_ms);
    // Copy elements of motor_1 into debug structure for CM core
    DBG_copyStruct(&motor_m1, &dbg_cpu2cm_m1);
    // Inform CM core that new data are available
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, \
                    IPC_ADDR_CORRECTION_ENABLE, IPC_NO_CMD, \
                    (uint32_t) &dbg_cpu2cm_m1, IPC_NO_DATA);
    // Clear IT source CLA task
    CLA_clearTaskFlags(CLA1_BASE, CLA_MOTOR1_TASK_FLAG);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR1_GROUP);
    DBG_CLA_M1_ISR_UNSET;
}

/**
 * @brief       Generate part of the Tx message associated after MOTOR_2 control.
 *              Compute message CRC.
 */
__attribute__((interrupt)) void cla_isrMotor2(void)
{
    DBG_CLA_M2_ISR_SET;
    // Fill of the M2 motor's elements in the Tx message
    COM_msgCreate(&motor_m2, &slv2mst_msg, &cpu_tmr_cnt_ms);
    // Finalize the Tx message. Aggregate MOTOR_2 values + computed CRC
    switch(com_bus_select)
    {
        case COM_USE_SPI_BUS:
        {
            // Is DMA transfer (SPI Tx) in progress or SPI CS low? (SPI active?)
            if((!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR)) && COM_SPI_CS_READ)
            {
                spi_tx_msg_t* p_msg_spi = &com_spi_tx.msg[com_spi_tx.msg_nx];
                p_msg_spi->data         = slv2mst_msg;
                uint32_t msg_crc        = COM_msgCRC((uint16_t *) &p_msg_spi->data, \
                                                     COM_MSG_SPI_TX_16BIT_PAYLOAD_LENGTH);
                // swap words in message - Required for Solo
                p_msg_spi->crc.u16_msb  = LSB_32(msg_crc);
                p_msg_spi->crc.u16_lsb  = MSB_32(msg_crc);
                // New message complete
                // Is SPI already transmitting (SPI Tx is slave - no control when Tx start)?
                if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
                {
                    // swap active message array
                    com_spi_tx.msg_cr   = !com_spi_tx.msg_cr;
                    com_spi_tx.msg_nx   = !com_spi_tx.msg_cr;
                    // Reinitialize the SPI Tx FIFO & force first write in SPI_O_DAT register
                    SPI_resetTxFIFO(COM_SPI_BASE);
                    COM_SPI_DAT(p_msg_spi->data.status.all);
                    // Send remaining word through DMA
                    DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, \
                                        (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, \
                                        (uint16_t *) &p_msg_spi->data.timeStamp);
                    DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
                }
            }
            break;
        }

#if (USE_UOMODRI_REV >= USE_UOMODRI_V3)
        case COM_USE_RS485_BUS:
        {
            // Is RS485 Tx transfer in progress?
            if(!com_rs485_tx.index)
            {
                rs485_tx_msg_u* p_msg_rs485 = &com_rs485_tx.msg[com_rs485_tx.msg_nx];
                p_msg_rs485->header.addr    = com_msg_rs485_tx_master_address;
                p_msg_rs485->header.type    = COM_MSG_RS485_TX_TYPE_STANDARD;
                p_msg_rs485->header.size    = COM_MSG_RS485_TX_8BIT_FULL_LENGTH;
                p_msg_rs485->data           = slv2mst_msg;
                uint32_t msg_crc            = COM_msgCRC((uint16_t *) &p_msg_rs485->header, \
                                                         COM_MSG_RS485_TX_16BIT_PAYLOAD_LENGTH);
                // swap words in message - Required for Solo
                p_msg_rs485->crc.u16_msb    = LSB_32(msg_crc);
                p_msg_rs485->crc.u16_lsb    = MSB_32(msg_crc);
                // Split 16bits words into 2x 8bits words.
                RS485_msgWordToBytes(&com_rs485_tx);
                // New message complete. If we are here RS485 has not started.
                com_rs485_tx.msg_cr         = !com_rs485_tx.msg_cr;
                com_rs485_tx.msg_nx         = !com_rs485_tx.msg_cr;
            }
            break;
        }

        case COM_USE_CANFD_BUS:
        {
            canfd_tx_t* p_msg_canfd = &com_canfd_tx;
            p_msg_canfd->buf.id     = CANFD_ID_REG(com_msg_canfd_tx_master_id, \
                                                   COM_MSG_CANFD_SUB_ID_STATUS);// CANFD_ID(0U, 1U);
            p_msg_canfd->buf.rtr    = MCAN_RTR_TX_DATA_FRAME;       // Transmit data frame.
            p_msg_canfd->buf.xtd    = MCAN_ID_TYPE_11_BIT;          // 11-bit standard identifier.
            p_msg_canfd->buf.esi    = MCAN_ESI_PASSIVE_ERROR_FLAG;  // ESI bit in CAN FD format depends only on error passive flag.
            p_msg_canfd->buf.dlc    = MCAN_DLC_PAYLOAD_LENGTH_32B;  // CAN + CAN FD: transmit frame has 0-8 data bytes.
            p_msg_canfd->buf.brs    = MCAN_BIT_RATE_SWITCHING_MODE; // CAN FD frames transmitted with bit rate switching.
            p_msg_canfd->buf.fdf    = MCAN_FLEXIBLE_DATA_RATE_MODE; // Frame transmitted in CAN FD format.
            p_msg_canfd->buf.efc    = 1U;                           // Store Tx events.
            p_msg_canfd->buf.mm     = 0xAAU;                        // Message Marker.
            p_msg_canfd->data       = slv2mst_msg;
            // Split 16bits words into 2x 8bits words.
            CANFD_msgWordToBytes(&com_canfd_tx);                    // CANFD_msgTransmit(&com_canfd_tx);
            // New message ready to be transmitted.
            MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, \
                             MCAN_TX_BUF_0_INDEX, &p_msg_canfd->buf);
            break;
        }
#endif
        default:
            break;

    }
    // Copy elements of motor_2 into debug structure for CM core
    DBG_copyStruct(&motor_m2, &dbg_cpu2cm_m2);
    // Inform CM core that new data are available
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, \
                    IPC_ADDR_CORRECTION_ENABLE, IPC_NO_CMD, \
                    (uint32_t) &dbg_cpu2cm_m2, IPC_NO_DATA);
    // Clear IT source CLA task
    CLA_clearTaskFlags(CLA1_BASE, CLA_MOTOR2_TASK_FLAG);
    // Clear IT
    Interrupt_clearACKGroup(INT_CLA_MOTOR2_GROUP);
    DBG_CLA_M2_ISR_UNSET;
}

/**
 * @brief       Create the message for LED dimming + increment the timeout counter.
 */
__attribute__((interrupt)) void tmr_isrLedCmd(void)
{
    DBG_TIMER_ISR_SET;
    cpu_tmr_cnt_ms++;
    // LED dimming control
    if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
    {
        LED_dimming(&motor_m1, &motor_m2, &led_spi_tx[0]);
        DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    }
    DRV_readStatusFSM(&drv_cfg[MOTOR_1], &drv_reg[MOTOR_1], \
                      &drv_cfg[MOTOR_2], &drv_reg[MOTOR_2]);
    // Timeout management
    cmd_uomodri[MOTOR_1].cptTimeout++;
    cmd_uomodri[MOTOR_2].cptTimeout++;
    // Force an update of the motors command structures (Timeout value)
    CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
    // Clear IT
    Interrupt_clearACKGroup(INT_TIMER_LED_CMD_GROUP);
    DBG_TIMER_ISR_UNSET;
}

/**
 * @brief       Check & extract the received message from SPI.
 */
__attribute__((interrupt)) void gpio_isrSpiComRx(void)
{
    DBG_GPIO_SPI_RX_ISR_SET;
    if(com_bus_select == COM_USE_SPI_BUS)
    {
        if(!DMA_getTransferStatusFlag(DMA_COM_SPI_RX_BASE_ADDR))
        {
            // Compare computed CRC with the one received in message.
            spi_rx_msg_t* p_msg_spi = &com_spi_rx.msg;
            uint32_t msg_crc        = COM_msgCRC((uint16_t *) &p_msg_spi->data, \
                                                 COM_MSG_SPI_RX_16BIT_PAYLOAD_LENGTH);
            if(CRC_COMP(p_msg_spi->crc.u16_msb, p_msg_spi->crc.u16_lsb, msg_crc))
            {
                // Extract received message & send new commands to FOC structures.
                COM_msgExtract(&p_msg_spi->data, \
                               &cmd_uomodri[MOTOR_1], \
                               &cmd_uomodri[MOTOR_2]);
                // Force an update of the motors command structures (commands values)
                CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
            }
        }
        // Flush DMA & restart reception & reset the SPI Rx fifo.
        DMA_triggerSoftReset(DMA_COM_SPI_RX_BASE_ADDR);
        DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
        SPI_resetRxFIFO(COM_SPI_BASE);
    }
    // Clear IT flag
    Interrupt_clearACKGroup(INT_GPIO_SPI_CS_GROUP);
    DBG_GPIO_SPI_RX_ISR_UNSET;
}

#if (USE_UOMODRI_REV >= USE_UOMODRI_V3)
/**
 * @brief       SCI Tx FIFO level reached - ready to receive new data.
 */
__attribute__((interrupt)) void sci_isrTx_RS485(void)
{
    DBG_SCI_RX485_TX_ISR_SET;
    // FIFO IT source test.
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
    // FIFO TX IT?
    if(sci_it_src_test & SCI_INT_TXFF)
    {
        RS485_msgTransmit(COM_SCI_BASE, &com_rs485_tx);
        if(com_rs485_tx.index >= COM_MSG_RS485_TX_8BIT_FULL_LENGTH)
        {
            // Disable FIFO Tx interrupts source
            HWREGH(COM_SCI_FIFO_TX_REG_ADDR)   &= ~SCI_FFTX_TXFFIENA;
            com_rs485_tx.index = 0;
        }
        // / Clear FIFO Tx flag
        HWREGH(COM_SCI_FIFO_TX_REG_ADDR)       |= SCI_FFTX_TXFFINTCLR;
    }
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_TX_MSG_GROUP);
    DBG_SCI_RX485_TX_ISR_UNSET;
}

/**
 *@brief        SCI Rx FIFO level reached + slave address valid (IT enable).
 */
__attribute__((interrupt)) void sci_isrRx_RS485(void)
{
    DBG_SCI_RX485_RX_ISR_SET;
    // FIFO IT source test.
    uint32_t sci_it_src_test = SCI_getInterruptStatus(COM_SCI_BASE);
    // FIFO RX IT?
    if(sci_it_src_test & SCI_INT_RXFF)
    {
        if(com_rs485_rx.addr1 || com_rs485_rx.addr2)
            // Get contain of FIFO Rx
            RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
        else
        {
            HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   &= ~(SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFIENA);
            HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   |= (SCI_FFRX_RXFIFORESET);
        }
        // Clear IT sources (FIFO IT source + FIFO overflow IT source)
        HWREGH(COM_SCI_FIFO_RX_REG_ADDR)  |= (SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
    }
    /*
     * RX ERROR :
     * FE - SCI framing-error flag.
     * OE - SCI overrun-error flag.
     * PE - SCI parity-error flag.
     */
    if(sci_it_src_test & SCI_INT_RXERR)
    {
        // Software reset
        HWREGH(COM_SCI_CTRL1_REG_ADDR)     &= ~SCI_CTL1_SWRESET;
        HWREGH(COM_SCI_CTRL1_REG_ADDR)     |= SCI_CTL1_SWRESET;
    }
    // Clear global IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_MSG_GROUP);
    DBG_SCI_RX485_RX_ISR_UNSET;
}

/**
 * @brief       CLB - Valid address & EOT ITs.
 */
__attribute__((interrupt)) void clb_isrTxRx_RS485(void)
{
    DBG_CLB_RS485_TX_RX_ISR_SET;
    if(com_bus_select == COM_USE_RS485_BUS)
    {
        uint16_t clb_tag_test   = HWREGH(CLB_INT_REG_ADDR);

        switch(clb_tag_test)
        {
            // RS485 address valid
            case CLB_INT_RS485_RX_ADDRESS_1_VALID:
            case CLB_INT_RS485_RX_ADDRESS_2_VALID:
            {
                // Set flags
                com_rs485_rx.index  = 0;
                if(clb_tag_test == CLB_INT_RS485_RX_ADDRESS_1_VALID)
                {
                    com_rs485_rx.addr1  = true;
                    com_rs485_rx.addr2  = false;
                }
                else
                {
                    com_rs485_rx.addr1  = false;
                    com_rs485_rx.addr2  = true;
                }
                // First FIFO read.
                RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
                // Clear IT source & Enable FIFO Rx & Rx error IT sources.
                HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   |= (SCI_FFRX_RXFFIENA | SCI_FFRX_RXFFINTCLR | SCI_FFRX_RXFFOVRCLR);
                HWREGH(COM_SCI_CTRL1_REG_ADDR)     |= SCI_CTL1_RXERRINTENA;
                break;
            }

            // RS485 End-Of-Reception detection
            case CLB_INT_RS485_RX_END_OF_RECEPTION:
            {
                // Disable immediately SCI FIFO Rx & errors ITs while a valid address is not received.
                HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   &= ~SCI_FFRX_RXFFIENA;
                HWREGH(COM_SCI_CTRL1_REG_ADDR)     &= ~SCI_CTL1_RXERRINTENA;
                // Purge pending ITs.
                HWREGH(PIECTRL_BASE + PIE_O_IFR8)  &= ~((uint16_t)INT_RS485_RX_PIE_FLAG);
                HWREGH(PIECTRL_BASE + PIE_O_ACK)    = INT_RS485_RX_MSG_GROUP;
                // Is it a valid address ?
                if(com_rs485_rx.addr1 || com_rs485_rx.addr2)
                {
                    // Last FIFO read.
                    RS485_msgReceive(COM_SCI_BASE, &com_rs485_rx);
                    rs485_rx_msg_u* p_msg_rs485     = &com_rs485_rx.msg;
                    uint32_t msg_crc                = COM_msgCRC((uint16_t *) &p_msg_rs485->header, \
                                                                 COM_MSG_RS485_RX_16BIT_PAYLOAD_LENGTH);
                    if((com_bus_select == COM_USE_RS485_BUS) && \
                            CRC_COMP(p_msg_rs485->crc.u16_msb, p_msg_rs485->crc.u16_lsb, msg_crc))
                    {
                        // Message valid - Extract it & transfer to motors structures.
                        COM_msgExtract(&p_msg_rs485->data, \
                                       &cmd_uomodri[MOTOR_1], \
                                       &cmd_uomodri[MOTOR_2]);
                        CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
                        // prepare reply to master - reset transmit index
                        com_rs485_tx.index          = 0;
                        p_msg_rs485->crc.u16_lsb    = 0;
                        p_msg_rs485->crc.u16_msb    = 0;
                        // prepare reply to master - reset RS485 transmit FIFO & enable RS485 SCI Tx ITs.
                        HWREGH(COM_SCI_FIFO_TX_REG_ADDR)   &= ~SCI_FFTX_TXFIFORESET;
                        HWREGH(COM_SCI_FIFO_TX_REG_ADDR)   |= (SCI_FFTX_TXFIFORESET | SCI_FFTX_TXFFIENA | SCI_FFTX_TXFFINTCLR);
                        // Inform CLB that next message is transmitted by slave
                        HWREG(CLB_GP_REG_ADDR)             |= CLB_GPREG_RS485_TX_ENABLE;
                        HWREG(CLB_GP_REG_ADDR)             &= ~CLB_GPREG_RS485_TX_ENABLE;
                    }
                }
                // Reset of SCI Rx FIFO & Clear IT sources.
                HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   &= ~(SCI_FFRX_RXFIFORESET);
                HWREGH(COM_SCI_FIFO_RX_REG_ADDR)   |= (SCI_FFRX_RXFIFORESET | SCI_FFRX_RXFFOVRCLR | SCI_FFRX_RXFFINTCLR);
                HWREGH(COM_SCI_CTRL1_REG_ADDR)     &= ~SCI_CTL1_SWRESET;
                HWREGH(COM_SCI_CTRL1_REG_ADDR)     |= SCI_CTL1_SWRESET;
                // Reset flags & index. Waiting for a new valid message.
                com_rs485_rx.index  = 0;
                com_rs485_rx.addr1  = false;
                com_rs485_rx.addr2  = false;
                break;
            }

            default:
            {
                com_rs485_rx.index  = 0;
                com_rs485_rx.addr1  = false;
                com_rs485_rx.addr2  = false;
                break;
            }
        }
    }
    // Clear CLB Flag
    CLB_clearInterruptTag(RS485_RX_TX_CLB_EOT_BASE);
    // Clear IT flag
    Interrupt_clearACKGroup(INT_RS485_RX_TX_CLB_GROUP);
    DBG_CLB_RS485_TX_RX_ISR_UNSET;
}

/**
 * @brief       MCAN - Extract the received message & send the pending one.
 */
__attribute__((interrupt)) void mcan_isr_canfd(void)
{
    DBG_MCAN_CANFD_TX_RX_ISR_SET;
    MCAN_RxNewDataStatus newData= {.statusLow = 0, .statusHigh = 0};
    uint32_t intrStatus         = MCAN_getIntrStatus(MCANA_DRIVER_BASE);
    // Clear the interrupt Status.
    MCAN_clearIntrStatus(MCANA_DRIVER_BASE, intrStatus);
    //  Clearing the interrupt lineNum
    MCAN_clearInterrupt(MCANA_DRIVER_BASE, MCAN_RX_LINE_IT_CLEAR);
    //  Check if interrupt is caused by a message being received in dedicated RX Buffers
    if((com_bus_select == COM_USE_CANFD_BUS) && (intrStatus & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG))
    {
        // Read the NewData registers
        MCAN_getNewDataStatus(MCANA_DRIVER_BASE, &newData);
        // If message is received in buffer element 0
        if(newData.statusLow & MCAN_RX_BUF_0_POS)
        {
            canfd_rx_t* p_msg_canfd = &com_canfd_rx;
            MCAN_readMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, \
                            MCAN_RX_BUF_0_INDEX, MCAN_RX_FIFO_NUM_0, \
                            &p_msg_canfd->buf);
            CANFD_msgBytesToWord(p_msg_canfd);  // CANFD_msgReceive(p_msg_canfd);
            // Extract received message & send new commands to FOC structures. Define SPI as communication bus by default.
            COM_msgExtract(&p_msg_canfd->data, \
                           &cmd_uomodri[MOTOR_1], \
                           &cmd_uomodri[MOTOR_2]);
            // Force an update of the motors command structures (commands values)
            CLA_forceTasks(CLA1_BASE, CLA_CMD_NEW_TASK_FLAG);
            // Add request for transmission.
            MCAN_txBufAddReq(MCANA_DRIVER_BASE, MCAN_TX_BUF_0_INDEX);
        }
    }
    else
        //  Interrupt handling for other interrupt sources goes here
        mcan_error++;

    //  Clearing the NewData registers
    MCAN_clearNewDataStatus(MCANA_DRIVER_BASE, &newData);
    // Acknowledge this interrupt located in group 9
    Interrupt_clearACKGroup(INT_MCAN_RX_TX_MSG_GROUP);
    DBG_MCAN_CANFD_TX_RX_ISR_UNSET;
}
#endif

/**
 * @brief       Errata workaround interrupt according errata note (sprz458.pdf)
 */
__attribute__((interrupt)) void errata_isr(void)
{
    errata_cnt = (errata_cnt < UINT32_MAX) ? (errata_cnt + 1) : (0U);
}
