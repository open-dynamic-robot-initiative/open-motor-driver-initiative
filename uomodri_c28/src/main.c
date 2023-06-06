/*
 * File name: main.c
 * Description: Source file containing main function and IT running on CPU1 - C28 processor.
 */

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <uomodri_shared.h>
#include "main.h"
#include "hal.h"
#include "drv8353.h"
#include "ws2812b.h"
#include "encoder.h"
#include "foc.h"
#include "motor.h"
#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"
#include "uomodri_prj_config_handlers.h"

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
#if (CLA_CORE_ENABLE)
extern motor_t          motor_m1;
extern motor_t          motor_m2;
#else
extern motor_t          motor[2];
#endif
extern const hal_cfg_t  hal;
extern cmd_t            cmd_uOmodri[2];

#if (defined DEBUG) && (CM_CORE_ENABLE)
#pragma DATA_SECTION(transfer_array, "MSGRAM_CPU_TO_CM");
float32_t               transfer_array[20];
#endif

uint8_t tx_msg_cr       = 0U;
uint8_t tx_msg_nx       = 1U;
bool_t tx_msg_full      = false;
bool_t tmr_timeout      = false;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static void MCU_ini(void);
bool_t COM_msgSpiReception(mst2slv_msg_t*, cmd_t*, cmd_t*, bool_t);
//bool_t COM_msgSpiReception(mst2slv_msg_t*, motor_t*, motor_t*, bool_t);
inline void LED_dimming(motor_t*, motor_t*, uint16_t*);
inline void IPC_CpuToCmAddrSync(void);

/***********************************************************************
 * FUNCTIONS DEFINITION
 ***********************************************************************/
/**
 * @brief   Main function running on CPU1/C28. MCU init, DRV init, ADC init \& calibration, IT init.
 */
void main(void)
{
    // uOmodri low level device initialization
    MCU_ini();
    // Hardware Abstraction Layer initialization
    HAL_ini(&hal);
//#if (CLA_CORE_ENABLE)
//    // Force task 8, the one time initialization task
//    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_8);
//#endif
    // DRV8353 initialization
    DRV_ini(&drv_cfg[MOTOR_1], &drv_reg[MOTOR_1]);
    DRV_ini(&drv_cfg[MOTOR_2], &drv_reg[MOTOR_2]);
    // ADC offset calibration for debug
    HAL_ADC_offsetCalib(&periph_motor_cfg[MOTOR_1]);
    HAL_ADC_offsetCalib(&periph_motor_cfg[MOTOR_2]);
#if (!CLA_CORE_ENABLE)
    // Force a reset of FOC structures
    FOC_resetStruct(motor[MOTOR_1].p_foc);
    FOC_resetStruct(motor[MOTOR_2].p_foc);
#endif
    // Force a reset of the intermediate command structures. Working structures are embedded in FOC structures.
    COM_resetCmdStruct(&cmd_uOmodri[MOTOR_1]);
    COM_resetCmdStruct(&cmd_uOmodri[MOTOR_2]);
#if (!CLA_CORE_ENABLE)
    // DMA initialization for memory to memory transfer
    HAL_DMA_ini(dmaMem2MemCfgList);
#endif
    // Interrupts initialization
    HAL_INT_ini(InterruptList);
#if (CM_CORE_ENABLE)
//    IPC_CpuToCmAddrSync();
#endif

    static uint8_t cptTimeout   = 0U;
    bool_t rx_msg_valid         = false;

    // Infinite loop
    while(1)
    {
        /*
         * SPI COM RX management - Receive & extract message if valid
         * DMA CMD TO FOC struct - Propagate new commands (message extract or timeout inc)
         */
#if (CLA_CORE_ENABLE)
//        COM_msgSpiReception(&cmd_msg, &cmd_uOmodri[MOTOR_1], &cmd_uOmodri[MOTOR_2], timeout_flag);
        if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS_NUM))
        {
            // Prepare SPI and associated DMA for a new receive
            SPI_resetRxFIFO(COM_SPI_BASE);
            DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
            // Extract received message & send new commands to FOC structures if valid
            rx_msg_valid                    = COM_msgExtract(&cmd_msg, &cmd_uOmodri[MOTOR_1], &cmd_uOmodri[MOTOR_2]);
            // Restart DMA reception
            DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
        }
        cptTimeout                          = (tmr_timeout)     ? (cptTimeout + 1)  : (cptTimeout);
        cptTimeout                          = (rx_msg_valid)    ? (0U)              : (cptTimeout);
        if(tmr_timeout || rx_msg_valid)
        {
//            if(rx_msg_valid)
//                cptTimeout                  = 0U;
//            else
//                cptTimeout                 += 1U;
            cmd_uOmodri[MOTOR_1].cptTimeout = cptTimeout;
            cmd_uOmodri[MOTOR_2].cptTimeout = cptTimeout;
            CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_3);
            tmr_timeout                     = false;
            rx_msg_valid                    = false;
        }
        //COM_msgSpiReception(&cmd_msg, &motor[MOTOR_1], &motor[MOTOR_2], timeout_flag);
//        cptTimeout                      = (timeout_flag) ? (cptTimeout + 1) : (cptTimeout);
//        cmd_uOmodri[MOTOR_1].cptTimeout = cptTimeout;
//        cmd_uOmodri[MOTOR_2].cptTimeout = cptTimeout;
//        if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS_NUM))
//        {
//            // Prepare SPI and associated DMA for a new receive
//            SPI_resetRxFIFO(COM_SPI_BASE);
//            DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
//            // Extract received message & send new commands to FOC structures if valid
//            cmd_msg_valid   = COM_msgExtract(&cmd_msg, &cmd_uOmodri[MOTOR_1], &cmd_uOmodri[MOTOR_2]);
//            // Restart DMA reception
//            DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//        }
//        // Send new commands to FOC structures if extract valid
//        if(cmd_msg_valid)
//        {
//            if((cmd_uOmodri[MOTOR_1].cmdArray.all & 0x0B) == 0x0B)
//                cmd_uOmodri[MOTOR_1].posRef += ((foc_t *)(motor[MOTOR_1].p_foc.ptr))->motor_enc.thetaIndexAbsolute;
//            if((cmd_uOmodri[MOTOR_2].cmdArray.all & 0x0B) == 0x0B)
//                cmd_uOmodri[MOTOR_2].posRef += ((foc_t *)(motor[MOTOR_2].p_foc.ptr))->motor_enc.thetaIndexAbsolute;
//            cptTimeout = 0U;
//        }
//        cmd_msg_valid       = false;
//        timeout_flag        = false;
#else
        if(!DMA_getTransferStatusFlag(DMA_CMD_2_FOC_BASE_ADDR))
        {
            cptTimeout = (timeout_flag) ? (cptTimeout + 1) : (cptTimeout);
            cmd_uOmodri[MOTOR_1].cptTimeout = cptTimeout;
            cmd_uOmodri[MOTOR_2].cptTimeout = cptTimeout;
            if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS_NUM))
            {
                // Prepare SPI and associated DMA for a new receive
                SPI_resetRxFIFO(COM_SPI_BASE);
                DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
                // Extract received message & send new commands to FOC structures if valid
                cmd_msg_valid       = COM_msgExtract(&cmd_msg, &cmd_uOmodri[MOTOR_1], &cmd_uOmodri[MOTOR_2]);
                // Restart DMA reception
                DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
            }
            // Send new commands to FOC structures if extract valid
            if(cmd_msg_valid || timeout_flag)
            {
                if(cmd_msg_valid && ((cmd_uOmodri[MOTOR_1].cmdArray.all & 0x0B) == 0x0B))
                    cmd_uOmodri[MOTOR_1].posRef += motor[MOTOR_1].p_foc->motor_enc.thetaIndexAbsolute;
                if(cmd_msg_valid && ((cmd_uOmodri[MOTOR_2].cmdArray.all & 0x0B) == 0x0B))
                    cmd_uOmodri[MOTOR_2].posRef += motor[MOTOR_2].p_foc->motor_enc.thetaIndexAbsolute;
                DMA_forceTrigger(DMA_CMD_2_FOC_BASE_ADDR);
            }
            cmd_msg_valid           = false;
            timeout_flag            = false;
        }
#endif
        /*
         * SPI COM TX management - Create message & send it
         */
#if (CLA_CORE_ENABLE)
        if(tx_msg_full && !DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR) && COM_SPI_CS_READ)
        {
            DINT;//__disable_interrupts();//
            tx_msg_cr   = (tx_msg_cr) ? (0U) : (1U);
            tx_msg_nx   = (tx_msg_cr) ? (0U) : (1U);
            EINT;//__enable_interrupts();//

            DMA_configAddresses(DMA_COM_SPI_TX_BASE_ADDR, (uint16_t *) COM_SPI_TX_BASE_REG_ADDR, (uint16_t *) &status_msg[tx_msg_cr].timeStamp);
            // Force first write in SPI_O_DAT
            COM_SPI_DAT(status_msg[tx_msg_cr].status.all);
            SPI_resetTxFIFO(COM_SPI_BASE);
            DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
            tx_msg_full = false;
        }

//            if(!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG))
//            {
#if (0)
                cpu2cm_dbg_msg.itcnt.uint32b    = (uint32_t)MSB_64(motor[MOTOR_1].itCnt);
                cpu2cm_dbg_msg.ia.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ia;
                cpu2cm_dbg_msg.ib.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ib;
                cpu2cm_dbg_msg.ic.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ic;
                cpu2cm_dbg_msg.vbus.float32b    = motor[MOTOR_1].p_motorFOC->motor_acq.vbus;
                cpu2cm_dbg_msg.ialpha.float32b  = motor[MOTOR_1].p_motorFOC->ialpha;
                cpu2cm_dbg_msg.ibeta.float32b   = motor[MOTOR_1].p_motorFOC->ibeta;
                cpu2cm_dbg_msg.id.float32b      = motor[MOTOR_1].p_motorFOC->id;
                cpu2cm_dbg_msg.iq.float32b      = motor[MOTOR_1].p_motorFOC->iq;
                cpu2cm_dbg_msg.iqRef.float32b   = motor[MOTOR_1].p_motorFOC->iqRef;
                cpu2cm_dbg_msg.ud.float32b      = motor[MOTOR_1].p_motorFOC->ud;
                cpu2cm_dbg_msg.uq.float32b      = motor[MOTOR_1].p_motorFOC->uq;
                cpu2cm_dbg_msg.ualpha.float32b  = motor[MOTOR_1].p_motorFOC->ualpha;
                cpu2cm_dbg_msg.ubeta.float32b   = motor[MOTOR_1].p_motorFOC->ubeta;
                cpu2cm_dbg_msg.ua.float32b      = motor[MOTOR_1].p_motorFOC->ua;
                cpu2cm_dbg_msg.ub.float32b      = motor[MOTOR_1].p_motorFOC->ub;
                cpu2cm_dbg_msg.uc.float32b      = motor[MOTOR_1].p_motorFOC->uc;
                cpu2cm_dbg_msg.theta.float32b   = motor[MOTOR_1].p_motorFOC->motor_enc.thetaAbsolute;
                cpu2cm_dbg_msg.posRef.float32b  = motor[MOTOR_1].p_motorFOC->motor_cmd.posRef;
                cpu2cm_dbg_msg.speed.float32b   = motor[MOTOR_1].p_motorFOC->motor_enc.speed.speedMech;
                cpu2cm_dbg_msg.velRef.float32b  = motor[MOTOR_1].p_motorFOC->motor_cmd.velRef;
                cpu2cm_dbg_msg.pd_kp.float32b   = motor[MOTOR_1].p_motorFOC->motor_cmd.kpCoeff;
                cpu2cm_dbg_msg.pd_kd.float32b   = motor[MOTOR_1].p_motorFOC->motor_cmd.kdCoeff;
                cpu2cm_dbg_msg.iqff.float32b    = motor[MOTOR_1].p_motorFOC->motor_cmd.iqff;
                cpu2cm_dbg_msg.isat.float32b    = motor[MOTOR_1].p_motorFOC->motor_cmd.iSat;
                cpu2cm_dbg_msg.status.uint32b   = (uint32_t)motor[MOTOR_1].p_motorFOC->motor_cmd.enableReg.all;
                //        IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
#endif
//                transfer_array[0]   = motor[MOTOR_2].p_motorFOC->motor_acq.ia;
//                transfer_array[1]   = motor[MOTOR_2].p_motorFOC->motor_acq.ib;
//                transfer_array[2]   = motor[MOTOR_2].p_motorFOC->motor_acq.ic;
//                transfer_array[3]   = motor[MOTOR_2].p_motorFOC->motor_acq.vbus;
//                transfer_array[4]   = motor[MOTOR_2].p_motorFOC->ialpha;
//                transfer_array[5]   = motor[MOTOR_2].p_motorFOC->ibeta;
//                transfer_array[6]   = motor[MOTOR_2].p_motorFOC->id;
//                transfer_array[7]   = motor[MOTOR_2].p_motorFOC->iq;
//                transfer_array[8]   = motor[MOTOR_2].p_motorFOC->ud;
//                transfer_array[9]   = motor[MOTOR_2].p_motorFOC->uq;
//                transfer_array[10]  = motor[MOTOR_2].p_motorFOC->ualpha;
//                transfer_array[11]  = motor[MOTOR_2].p_motorFOC->ubeta;
//                transfer_array[12]  = motor[MOTOR_2].p_motorFOC->ua;
//                transfer_array[13]  = motor[MOTOR_2].p_motorFOC->ub;
//                transfer_array[14]  = motor[MOTOR_2].p_motorFOC->uc;
//                transfer_array[15]  = motor[MOTOR_2].p_motorFOC->motor_enc.thetaAbsolute;
//                transfer_array[16]  = motor[MOTOR_2].p_motorFOC->motor_cmd.posRef;
//                transfer_array[17]  = motor[MOTOR_2].p_motorFOC->motor_enc.speed.speedMech;
//                transfer_array[18]  = motor[MOTOR_2].p_motorFOC->motor_cmd.velRef;
//                transfer_array[19]  = motor[MOTOR_2].p_motorFOC->iqRef;
//
//                IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
//            }

#else
        if(motor[MOTOR_2].itDone && motor[MOTOR_1].itDone)
        {
            if(!DMA_getRunStatusFlag(DMA_COM_SPI_TX_BASE_ADDR))
            {
                COM_msgCreate(&motor[MOTOR_1], &motor[MOTOR_2], &status_msg);
                if(COM_SPI_CS_READ)
                {
                    // Force first write in SPI_O_DAT
                    COM_SPI_DAT(status_msg.status.all);
                    SPI_resetTxFIFO(COM_SPI_BASE);
                    DMA_startChannel(DMA_COM_SPI_TX_BASE_ADDR);
                }
            }
//            if(!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG))
//            {
#if (0)
                cpu2cm_dbg_msg.itcnt.uint32b    = (uint32_t)MSB_64(motor[MOTOR_1].itCnt);
                cpu2cm_dbg_msg.ia.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ia;
                cpu2cm_dbg_msg.ib.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ib;
                cpu2cm_dbg_msg.ic.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ic;
                cpu2cm_dbg_msg.vbus.float32b    = motor[MOTOR_1].p_motorFOC->motor_acq.vbus;
                cpu2cm_dbg_msg.ialpha.float32b  = motor[MOTOR_1].p_motorFOC->ialpha;
                cpu2cm_dbg_msg.ibeta.float32b   = motor[MOTOR_1].p_motorFOC->ibeta;
                cpu2cm_dbg_msg.id.float32b      = motor[MOTOR_1].p_motorFOC->id;
                cpu2cm_dbg_msg.iq.float32b      = motor[MOTOR_1].p_motorFOC->iq;
                cpu2cm_dbg_msg.iqRef.float32b   = motor[MOTOR_1].p_motorFOC->iqRef;
                cpu2cm_dbg_msg.ud.float32b      = motor[MOTOR_1].p_motorFOC->ud;
                cpu2cm_dbg_msg.uq.float32b      = motor[MOTOR_1].p_motorFOC->uq;
                cpu2cm_dbg_msg.ualpha.float32b  = motor[MOTOR_1].p_motorFOC->ualpha;
                cpu2cm_dbg_msg.ubeta.float32b   = motor[MOTOR_1].p_motorFOC->ubeta;
                cpu2cm_dbg_msg.ua.float32b      = motor[MOTOR_1].p_motorFOC->ua;
                cpu2cm_dbg_msg.ub.float32b      = motor[MOTOR_1].p_motorFOC->ub;
                cpu2cm_dbg_msg.uc.float32b      = motor[MOTOR_1].p_motorFOC->uc;
                cpu2cm_dbg_msg.theta.float32b   = motor[MOTOR_1].p_motorFOC->motor_enc.thetaAbsolute;
                cpu2cm_dbg_msg.posRef.float32b  = motor[MOTOR_1].p_motorFOC->motor_cmd.posRef;
                cpu2cm_dbg_msg.speed.float32b   = motor[MOTOR_1].p_motorFOC->motor_enc.speed.speedMech;
                cpu2cm_dbg_msg.velRef.float32b  = motor[MOTOR_1].p_motorFOC->motor_cmd.velRef;
                cpu2cm_dbg_msg.pd_kp.float32b   = motor[MOTOR_1].p_motorFOC->motor_cmd.kpCoeff;
                cpu2cm_dbg_msg.pd_kd.float32b   = motor[MOTOR_1].p_motorFOC->motor_cmd.kdCoeff;
                cpu2cm_dbg_msg.iqff.float32b    = motor[MOTOR_1].p_motorFOC->motor_cmd.iqff;
                cpu2cm_dbg_msg.isat.float32b    = motor[MOTOR_1].p_motorFOC->motor_cmd.iSat;
                cpu2cm_dbg_msg.status.uint32b   = (uint32_t)motor[MOTOR_1].p_motorFOC->motor_cmd.enableReg.all;
                //        IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
#endif
//                transfer_array[0]   = motor[MOTOR_2].p_motorFOC->motor_acq.ia;
//                transfer_array[1]   = motor[MOTOR_2].p_motorFOC->motor_acq.ib;
//                transfer_array[2]   = motor[MOTOR_2].p_motorFOC->motor_acq.ic;
//                transfer_array[3]   = motor[MOTOR_2].p_motorFOC->motor_acq.vbus;
//                transfer_array[4]   = motor[MOTOR_2].p_motorFOC->ialpha;
//                transfer_array[5]   = motor[MOTOR_2].p_motorFOC->ibeta;
//                transfer_array[6]   = motor[MOTOR_2].p_motorFOC->id;
//                transfer_array[7]   = motor[MOTOR_2].p_motorFOC->iq;
//                transfer_array[8]   = motor[MOTOR_2].p_motorFOC->ud;
//                transfer_array[9]   = motor[MOTOR_2].p_motorFOC->uq;
//                transfer_array[10]  = motor[MOTOR_2].p_motorFOC->ualpha;
//                transfer_array[11]  = motor[MOTOR_2].p_motorFOC->ubeta;
//                transfer_array[12]  = motor[MOTOR_2].p_motorFOC->ua;
//                transfer_array[13]  = motor[MOTOR_2].p_motorFOC->ub;
//                transfer_array[14]  = motor[MOTOR_2].p_motorFOC->uc;
//                transfer_array[15]  = motor[MOTOR_2].p_motorFOC->motor_enc.thetaAbsolute;
//                transfer_array[16]  = motor[MOTOR_2].p_motorFOC->motor_cmd.posRef;
//                transfer_array[17]  = motor[MOTOR_2].p_motorFOC->motor_enc.speed.speedMech;
//                transfer_array[18]  = motor[MOTOR_2].p_motorFOC->motor_cmd.velRef;
//                transfer_array[19]  = motor[MOTOR_2].p_motorFOC->iqRef;
//
//                IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
//            }
            motor[MOTOR_1].itDone   = false;
            motor[MOTOR_2].itDone   = false;
        }
#endif
        /*
         * SPI LED TX management - Create message & send it @ 1ms
         * SPI DRV TX/RX management - Read/Write to get DRV status in case of fault @ 1ms
         */
//        if(CPUTimer_getTimerOverflowStatus(CPU_TIMER_0_BASE))
//        {
//            // LED dimming control
//            if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
//            {
//#if (CLA_CORE_ENABLE)
//                LED_dimming(&motor_m1, &motor_m2, &led_msg[0]);
//#else
//                LED_dimming(&motor[MOTOR_1], &motor[MOTOR_2], &led_msg[0]);
//#endif
//                DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
//            }
//            // Read DRV data to get status registers (MOTOR_1)
////            if(motor[MOTOR_1].motor_error.drv_fault)
////                DRV_readStatus(motor[MOTOR_1].p_motorDRV);
////            // Read DRV data to get status registers (MOTOR_2)
////            if(motor[MOTOR_2].motor_error.drv_fault)
////                DRV_readStatus(motor[MOTOR_2].p_motorDRV);
//            // Timeout management
//            timeout_flag            = true;
//            // Clear CPU timer flag
//            CPUTimer_clearOverflowFlag(CPU_TIMER_0_BASE);
//        }
    }
}

/**
 * @brief           Microcontroller initialization function
 *                  - Configure master clock to 200MHz
 *                  - Configure auxiliary clock (CM core) to 125MHz (USB à 60MHz)
 *                  - Enable GPIO clocking
 *                  - Configure the interruption vector table
 *                  - Boot CM from flash or RAM
 */
static void MCU_ini(void)
{
#if (defined USB_BUS_ENABLE_ON_CPU1) || ((defined USB_BUS_ENABLE_ON_CM) && (defined CM_CORE_ENABLE))
    /* Redefined to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
     * AUXPLLCLK = 25MHz (XTAL_OSC) * 48 (IMULT) / (2 (REFDIV) * 5 (ODIV) * 2(AUXPLLDIV) )
     * AUXPLLCLK require a 60MHz clock for USB. AUPLLRAWCLK require a 120MHz clock for CM. */
#undef DEVICE_AUXSETCLOCK_CFG
#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL  | \
                                     SYSCTL_AUXPLL_IMULT(48)    | \
                                     SYSCTL_AUXPLL_REFDIV(2U)   | \
                                     SYSCTL_AUXPLL_ODIV(5U)     | \
                                     SYSCTL_AUXPLL_DIV_2        | \
                                     SYSCTL_AUXPLL_ENABLE       | \
                                     SYSCTL_DCC_BASE_0)
#undef DEVICE_AUXCLK_FREQ
#define DEVICE_AUXCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 48) / (2 * 5 * 2))
#endif
    /* Initialize clock and peripherals */
    Device_init();
    /* Redefine LSPCLK to run at 200MHz (for SPI & SCI bit clocks) */
//#undef DEVICE_LSPCLK_FREQ
//#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 1)
    /* Set LSPCLK divider */
//    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_1);
//    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == UOMODRI_LSPCLK_FREQ);
#if ((defined USB_BUS_ENABLE_ON_CM) && (defined CM_CORE_ENABLE))
    /* Allocate Shared Peripheral USB to the CM Side. */
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);
#endif
    /* Boot CM core */
#if (defined _FLASH) && (defined CM_CORE_ENABLE)
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#elif (defined CM_CORE_ENABLE)
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
    // Initialize GPIO
    Device_initGPIO();
    // Initialize PIE control registers and flags. Disables CPU interrupts.
    Interrupt_initModule();
    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    Interrupt_initVectorTable();

    return;
}

bool_t COM_msgSpiReception(mst2slv_msg_t* p_msg, cmd_t* p_cmd_m1, cmd_t* p_cmd_m2, bool_t timeout_flag)
{
    static uint8_t cptTimeout   = 0U;
    bool_t cmd_msg_valid        = false;

    if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS_NUM))
    {
        // Prepare SPI and associated DMA for a new receive
        SPI_resetRxFIFO(COM_SPI_BASE);
        DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
        // Extract received message & send new commands to FOC structures if valid
        cmd_msg_valid           = COM_msgExtract(p_msg, p_cmd_m1, p_cmd_m2);
        // Restart DMA reception
        DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
    }
    cptTimeout                  = (timeout_flag)    ? (cptTimeout + 1)  : (cptTimeout);
    cptTimeout                  = (cmd_msg_valid)   ? (0U)              : (cptTimeout);
    p_cmd_m1->cptTimeout        = cptTimeout;
    p_cmd_m2->cptTimeout        = cptTimeout;
    (cmd_msg_valid) ? (CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_3)) : (0);

    return(cmd_msg_valid);

}

//bool_t COM_msgSpiReception(mst2slv_msg_t* p_msg, motor_t* p_motor_m1, motor_t* p_motor_m2, bool_t timeout_flag)
//{
//#if (CLA_CORE_ENABLE)
//    foc_t* p_foc_m1             = (foc_t *)p_motor_m1->p_foc.ptr;
//    foc_t* p_foc_m2             = (foc_t *)p_motor_m2->p_foc.ptr;
//    cmd_t* p_cmd_m1             = &p_foc_m1->motor_cmd;
//    cmd_t* p_cmd_m2             = &p_foc_m2->motor_cmd;
//    static uint8_t cptTimeout   = 0;
//    bool_t cmd_msg_valid        = false;
//
//    cptTimeout                  = (timeout_flag) ? (cptTimeout + 1) : (cptTimeout);
//    p_cmd_m1->cptTimeout        = cptTimeout;
//    p_cmd_m2->cptTimeout        = cptTimeout;
//
//    if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS_NUM))
//    {
//        // Prepare SPI and associated DMA for a new receive
//        SPI_resetRxFIFO(COM_SPI_BASE);
//        DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
//        // Extract received message & send new commands to FOC structures if valid
//        cmd_msg_valid           = COM_msgExtract(p_msg, p_cmd_m1, p_cmd_m2);
//        // Restart DMA reception
//        DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//    }
//    if(cmd_msg_valid)
//    {
//        p_cmd_m1->posRef       += ((p_cmd_m1->cmdArray.all & 0x0B) == 0x0B) ? (p_foc_m1->motor_enc.thetaIndexAbsolute) : (0.0f);
//        p_cmd_m2->posRef       += ((p_cmd_m2->cmdArray.all & 0x0B) == 0x0B) ? (p_foc_m2->motor_enc.thetaIndexAbsolute) : (0.0f);
//        cptTimeout              = 0U;
//        // Force task 2, Copy content of reception structure into motor structure.
//        CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_2);
//    }
//    return(cmd_msg_valid);
//#else
//    if(!DMA_getTransferStatusFlag(DMA_CMD_2_FOC_BASE_ADDR))
//    {
//        foc_t* p_foc_m1             = p_motor_m1->p_foc;
//        foc_t* p_foc_m2             = p_motor_m2->p_foc;
//        cmd_t* p_cmd_m1             = &p_foc_m1->motor_cmd;
//        cmd_t* p_cmd_m2             = &p_foc_m2->motor_cmd;
//        static uint8_t cptTimeout   = 0;
//        bool_t cmd_msg_valid        = false;
//
//        cptTimeout                  = (timeout_flag) ? (cptTimeout + 1) : (cptTimeout);
//        p_cmd_m1->cptTimeout        = cptTimeout;
//        p_cmd_m2->cptTimeout        = cptTimeout;
//
//        if(!DMA_getRunStatusFlag(DMA_COM_SPI_RX_BASE_ADDR) && GPIO_readPin(COM_SPI_CS_NUM))
//        {
//            // Prepare SPI and associated DMA for a new receive
//            SPI_resetRxFIFO(COM_SPI_BASE);
//            DMA_clearTriggerFlag(DMA_COM_SPI_RX_BASE_ADDR);
//            // Extract received message & send new commands to FOC structures if valid
//            cmd_msg_valid       = COM_msgExtract(p_msg, p_cmd_m1, p_cmd_m2);
//            // Restart DMA reception
//            DMA_startChannel(DMA_COM_SPI_RX_BASE_ADDR);
//        }
//        // Send new commands to FOC structures if extract valid
//        if(cmd_msg_valid || timeout_flag)
//        {
//            if(cmd_msg_valid && ((cmd_uOmodri[MOTOR_1].cmdArray.all & 0x0B) == 0x0B))
//                cmd_uOmodri[MOTOR_1].posRef += motor[MOTOR_1].p_foc->motor_enc.thetaIndexAbsolute;
//            if(cmd_msg_valid && ((cmd_uOmodri[MOTOR_2].cmdArray.all & 0x0B) == 0x0B))
//                cmd_uOmodri[MOTOR_2].posRef += motor[MOTOR_2].p_foc->motor_enc.thetaIndexAbsolute;
//            DMA_forceTrigger(DMA_CMD_2_FOC_BASE_ADDR);
//        }
//        cmd_msg_valid           = false;
//        timeout_flag            = false;
//    }
//    return (true);
//#endif
//}

/**
 * @brief           Control the led dimming
 * @param[in]       p_motor Pointer on data structure array used for motor control.
 * @param[out]      p_msg   Pointer on the message to be sent to led via SPI.
 */
inline void LED_dimming(motor_t* p_motor1, motor_t* p_motor2, uint16_t* p_msg)
{
    static uint16_t timer_cpt_ms    = 0;
    float32_t       timer_fade_blink= (float32_t)((timer_cpt_ms < 500)  ? (timer_cpt_ms)    : (1000 - timer_cpt_ms)) / 500.0f;
    float32_t       timer_blink     = (timer_cpt_ms < 500)              ? (0.0f)            : (1.0f);
    timer_cpt_ms                    = (timer_cpt_ms < 1000)             ? (timer_cpt_ms + 1): (0);
    uint8_t         motor_state     = MOTOR_STATE_START;

    if((p_motor1->motor_state == MOTOR_STATE_START)         || (p_motor2->motor_state == MOTOR_STATE_START))
        motor_state = MOTOR_STATE_START;
    else if((p_motor1->motor_state == MOTOR_STATE_INIT_FW)  || (p_motor1->motor_state == MOTOR_STATE_INIT_REV)  ||
            (p_motor1->motor_state == MOTOR_STATE_INIT_TEST)|| (p_motor1->motor_state == MOTOR_STATE_INIT_FIX)  ||
            (p_motor2->motor_state == MOTOR_STATE_INIT_FW)  || (p_motor2->motor_state == MOTOR_STATE_INIT_REV)  ||
            (p_motor2->motor_state == MOTOR_STATE_INIT_TEST)|| (p_motor2->motor_state == MOTOR_STATE_INIT_FIX))
        motor_state = MOTOR_STATE_INIT_FIX;
    else if((p_motor1->motor_state == MOTOR_STATE_READY)    || (p_motor1->motor_state == MOTOR_STATE_STOP)      ||
            (p_motor2->motor_state == MOTOR_STATE_READY)    || (p_motor2->motor_state == MOTOR_STATE_STOP))
        motor_state = MOTOR_STATE_READY;
    if((p_motor1->motor_state == MOTOR_STATE_ERROR)         || (p_motor2->motor_state == MOTOR_STATE_ERROR))
        motor_state = MOTOR_STATE_ERROR;

    switch(motor_state)
    {
    case MOTOR_STATE_INIT_FW:
    case MOTOR_STATE_INIT_REV:
    case MOTOR_STATE_INIT_TEST:
    case MOTOR_STATE_INIT_FIX:
        ws2812b_writeLed((uint16_t)(0xff * timer_fade_blink), 0, (uint16_t)(0xff * timer_fade_blink), p_msg); //Magenta fade,
        break;
    case MOTOR_STATE_READY:
    case MOTOR_STATE_STOP:
        ws2812b_writeLed(0, (uint16_t)(0xff * timer_fade_blink), 0, p_msg);//Green fade, Active control
        break;
    case MOTOR_STATE_ERROR:
        ws2812b_writeLed((uint16_t)(0xff * timer_blink), 0, 0, p_msg);//red fade, error control
        break;
    case MOTOR_STATE_START:
    default:
        ws2812b_writeLed(0,  0, (uint16_t)(0xff * timer_fade_blink), p_msg);//blue fade
    }

    return;
}

/**
 * @brief       Transmit the data address to be broadcast by CM.
 */
inline void IPC_CpuToCmAddrSync(void)
{
#if (1)
    // Send address of ia of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[0], 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
#endif
#if (0)
    // Send address of ia of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&cpu2cm_dbg_msg, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
#endif
#if (0)
    // Send address of ia of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->motor_acq.ia, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ib of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->motor_acq.ib, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ic of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->motor_acq.ic, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of vbus of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->motor_acq.vbus, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ialpha of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->ialpha, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ibeta of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->ibeta, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of id of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->id, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of iq of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->iq, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ud of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->ud, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of uq of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->uq, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ualpha of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->ualpha, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ubeta of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->ubeta, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ua of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->ua, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ub of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->ub, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of uc of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->uc, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of absolute encoder position of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->motor_enc.thetaAbsolute, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of PositionRef of MOTOR_1
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&cmd_uOmodri_cpu2cm[MOTOR_1].posRef, 0);//motor[MOTOR_1].p_motorFOC->motor_cmd.posRef, 0);
//    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of encoder speed variable of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->motor_enc.speed.speedMech, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of SpeedRef of MOTOR_1
//    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&cmd_uOmodri_cpu2cm[MOTOR_1].velRef, 0);//motor[MOTOR_1].p_motorFOC->motor_cmd.velRef, 0);
//    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of IqRef of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].p_motorFOC->iqRef, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of IqRef of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&motor[MOTOR_1].itCnt, 0);
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
#endif
#if (0)
    // Send address of ia of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[0], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ib of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[1], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ic of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[2], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of vbus of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[3], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of ialpha of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[4], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ibeta of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[5], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of id of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[6], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of iq of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[7], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of ud of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[8], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of uq of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[9], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of ualpha of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[10], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ubeta of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[11], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of ua of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[12], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of ub of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[13], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of uc of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[14], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of absolute encoder position of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[15], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of reference position of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[16], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    // Send address of encoder speed variable of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[17], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
    // Send address of reference speed of MOTOR_1
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[18], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);

    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, 0, (uint32_t)&transfer_array[19], sizeof(float32_t));
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_CM_TO_CPU1_FLAG);
#endif
    return;
}

/**
 * @brief           ADC interrupt for MOTOR_1. Read currents \& voltages and compute control.
 */
__attribute__((interrupt)) void adc_isrMotor1(void)
{
#if (UOMODRI_V1_0_ENABLE)
    EXT_DBG_SPI_CLK_SET;
#elif (UOMODRI_V2_0_ENABLE)
//    DBG_PIN1_SET;
#endif
    while(DMA_getTransferStatusFlag(DMA_CMD_2_FOC_BASE_ADDR));
    // Motor control
#if (!CLA_CORE_ENABLE)
    motor[MOTOR_1].itDone = MOT_runControl(&motor[MOTOR_1]);
#endif
#if (defined DEBUG) && (CM_CORE_ENABLE) && (0)
    if(!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG))
    {
        transfer_array[0]   = motor[MOTOR_1].p_motorFOC->motor_acq.ia;
        transfer_array[1]   = motor[MOTOR_1].p_motorFOC->motor_acq.ib;
        transfer_array[2]   = motor[MOTOR_1].p_motorFOC->motor_acq.ic;
        transfer_array[3]   = motor[MOTOR_1].p_motorFOC->motor_acq.vbus;
        transfer_array[4]   = motor[MOTOR_1].p_motorFOC->ialpha;
        transfer_array[5]   = motor[MOTOR_1].p_motorFOC->ibeta;
        transfer_array[6]   = motor[MOTOR_1].p_motorFOC->id;
        transfer_array[7]   = motor[MOTOR_1].p_motorFOC->iq;
        transfer_array[8]   = motor[MOTOR_1].p_motorFOC->ud;
        transfer_array[9]   = motor[MOTOR_1].p_motorFOC->uq;
        transfer_array[10]  = motor[MOTOR_1].p_motorFOC->ualpha;
        transfer_array[11]  = motor[MOTOR_1].p_motorFOC->ubeta;
        transfer_array[12]  = motor[MOTOR_1].p_motorFOC->ua;
        transfer_array[13]  = motor[MOTOR_1].p_motorFOC->ub;
        transfer_array[14]  = motor[MOTOR_1].p_motorFOC->uc;
        transfer_array[15]  = motor[MOTOR_1].p_motorFOC->motor_enc.thetaAbsolute;
        transfer_array[16]  = motor[MOTOR_1].p_motorFOC->motor_cmd.posRef;
        transfer_array[17]  = motor[MOTOR_1].p_motorFOC->motor_enc.speed.speedMech;
        transfer_array[18]  = motor[MOTOR_1].p_motorFOC->motor_cmd.velRef;
        transfer_array[19]  = motor[MOTOR_1].p_motorFOC->iqRef;
        IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
    }
#endif
    // Set flag to indicate new data available
    if(!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG))
        IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IA_ADC_ADDR, MOTOR1_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR1_IA_INT_ACK_GROUP);
#if (UOMODRI_V1_0_ENABLE)
    EXT_DBG_SPI_CLK_CLEAR;
#elif ( UOMODRI_V2_0_ENABLE)
//    DBG_PIN1_CLEAR;
#endif
}

/**
 * @brief           ADC interrupt for MOTOR_2. Read currents \& voltages and compute control.
 */
__attribute__((interrupt)) void adc_isrMotor2(void)
{
#if (UOMODRI_V1_0_ENABLE)
    EXT_DBG_SPI_CLK_SET;
#elif (UOMODRI_V2_0_ENABLE)
//    DBG_PIN2_SET;
#endif
    while(DMA_getTransferStatusFlag(DMA_CMD_2_FOC_BASE_ADDR));
    // Motor control
#if (!CLA_CORE_ENABLE)
    motor[MOTOR_2].itDone = MOT_runControl(&motor[MOTOR_2]);
#endif
//    if(!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG))
//    {
//#if (0)
//        cpu2cm_dbg_msg.itcnt.uint32b    = (uint32_t)MSB_64(motor[MOTOR_1].itCnt);
//        cpu2cm_dbg_msg.ia.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ia;
//        cpu2cm_dbg_msg.ib.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ib;
//        cpu2cm_dbg_msg.ic.float32b      = motor[MOTOR_1].p_motorFOC->motor_acq.ic;
//        cpu2cm_dbg_msg.vbus.float32b    = motor[MOTOR_1].p_motorFOC->motor_acq.vbus;
//        cpu2cm_dbg_msg.ialpha.float32b  = motor[MOTOR_1].p_motorFOC->ialpha;
//        cpu2cm_dbg_msg.ibeta.float32b   = motor[MOTOR_1].p_motorFOC->ibeta;
//        cpu2cm_dbg_msg.id.float32b      = motor[MOTOR_1].p_motorFOC->id;
//        cpu2cm_dbg_msg.iq.float32b      = motor[MOTOR_1].p_motorFOC->iq;
//        cpu2cm_dbg_msg.iqRef.float32b   = motor[MOTOR_1].p_motorFOC->iqRef;
//        cpu2cm_dbg_msg.ud.float32b      = motor[MOTOR_1].p_motorFOC->ud;
//        cpu2cm_dbg_msg.uq.float32b      = motor[MOTOR_1].p_motorFOC->uq;
//        cpu2cm_dbg_msg.ualpha.float32b  = motor[MOTOR_1].p_motorFOC->ualpha;
//        cpu2cm_dbg_msg.ubeta.float32b   = motor[MOTOR_1].p_motorFOC->ubeta;
//        cpu2cm_dbg_msg.ua.float32b      = motor[MOTOR_1].p_motorFOC->ua;
//        cpu2cm_dbg_msg.ub.float32b      = motor[MOTOR_1].p_motorFOC->ub;
//        cpu2cm_dbg_msg.uc.float32b      = motor[MOTOR_1].p_motorFOC->uc;
//        cpu2cm_dbg_msg.theta.float32b   = motor[MOTOR_1].p_motorFOC->motor_enc.thetaAbsolute;
//        cpu2cm_dbg_msg.posRef.float32b  = motor[MOTOR_1].p_motorFOC->motor_cmd.posRef;
//        cpu2cm_dbg_msg.speed.float32b   = motor[MOTOR_1].p_motorFOC->motor_enc.speed.speedMech;
//        cpu2cm_dbg_msg.velRef.float32b  = motor[MOTOR_1].p_motorFOC->motor_cmd.velRef;
//        cpu2cm_dbg_msg.pd_kp.float32b   = motor[MOTOR_1].p_motorFOC->motor_cmd.kpCoeff;
//        cpu2cm_dbg_msg.pd_kd.float32b   = motor[MOTOR_1].p_motorFOC->motor_cmd.kdCoeff;
//        cpu2cm_dbg_msg.iqff.float32b    = motor[MOTOR_1].p_motorFOC->motor_cmd.iqff;
//        cpu2cm_dbg_msg.isat.float32b    = motor[MOTOR_1].p_motorFOC->motor_cmd.iSat;
//        cpu2cm_dbg_msg.status.uint32b   = (uint32_t)motor[MOTOR_1].p_motorFOC->motor_cmd.enableReg.all;
//#endif
//#if (CLA_CORE_ENABLE)
////        transfer_array[0]   = ((foc_t *)(motor_m2.p_foc.ptr))->motor_acq.ia;
////        transfer_array[1]   = ((foc_t *)(motor_m2.p_foc.ptr))->motor_acq.ib;
////        transfer_array[2]   = ((foc_t *)(motor_m2.p_foc.ptr))->motor_acq.ic;
////        transfer_array[3]   = ((foc_t *)(motor_m2.p_foc.ptr))->motor_acq.vbus;
////        transfer_array[4]   = ((foc_t *)(motor_m2.p_foc.ptr))->ialpha;
////        transfer_array[5]   = ((foc_t *)(motor_m2.p_foc.ptr))->ibeta;
////        transfer_array[6]   = ((foc_t *)(motor_m2.p_foc.ptr))->id;
////        transfer_array[7]   = ((foc_t *)(motor_m2.p_foc.ptr))->iq;
////        transfer_array[8]   = ((foc_t *)(motor_m2.p_foc.ptr))->ud;
////        transfer_array[9]   = ((foc_t *)(motor_m2.p_foc.ptr))->uq;
////        transfer_array[10]  = ((foc_t *)(motor_m2.p_foc.ptr))->ualpha;
////        transfer_array[11]  = ((foc_t *)(motor_m2.p_foc.ptr))->ubeta;
////        transfer_array[12]  = ((foc_t *)(motor_m2.p_foc.ptr))->ua;
////        transfer_array[13]  = ((foc_t *)(motor_m2.p_foc.ptr))->ub;
////        transfer_array[14]  = ((foc_t *)(motor_m2.p_foc.ptr))->uc;
////        transfer_array[15]  = ((foc_t *)(motor_m2.p_foc.ptr))->motor_enc.thetaAbsolute;
////        transfer_array[16]  = ((foc_t *)(motor_m2.p_foc.ptr))->motor_cmd.posRef;
////        transfer_array[17]  = ((foc_t *)(motor_m2.p_foc.ptr))->motor_enc.speed.speedMech[0];
////        transfer_array[18]  = ((foc_t *)(motor_m2.p_foc.ptr))->motor_cmd.velRef;
////        transfer_array[19]  = ((foc_t *)(motor_m2.p_foc.ptr))->iqRef;
//#else
//        transfer_array[0]   = motor[MOTOR_2].p_foc->motor_acq.ia;
//        transfer_array[1]   = motor[MOTOR_2].p_foc->motor_acq.ib;
//        transfer_array[2]   = motor[MOTOR_2].p_foc->motor_acq.ic;
//        transfer_array[3]   = motor[MOTOR_2].p_foc->motor_acq.vbus;
//        transfer_array[4]   = motor[MOTOR_2].p_foc->ialpha;
//        transfer_array[5]   = motor[MOTOR_2].p_foc->ibeta;
//        transfer_array[6]   = motor[MOTOR_2].p_foc->id;
//        transfer_array[7]   = motor[MOTOR_2].p_foc->iq;
//        transfer_array[8]   = motor[MOTOR_2].p_foc->ud;
//        transfer_array[9]   = motor[MOTOR_2].p_foc->uq;
//        transfer_array[10]  = motor[MOTOR_2].p_foc->ualpha;
//        transfer_array[11]  = motor[MOTOR_2].p_foc->ubeta;
//        transfer_array[12]  = motor[MOTOR_2].p_foc->ua;
//        transfer_array[13]  = motor[MOTOR_2].p_foc->ub;
//        transfer_array[14]  = motor[MOTOR_2].p_foc->uc;
//        transfer_array[15]  = motor[MOTOR_2].p_foc->motor_enc.thetaAbsolute;
//        transfer_array[16]  = motor[MOTOR_2].p_foc->motor_cmd.posRef;
//        transfer_array[17]  = motor[MOTOR_2].p_foc->motor_enc.speed.speedMech[0];
//        transfer_array[18]  = motor[MOTOR_2].p_foc->motor_cmd.velRef;
//        transfer_array[19]  = motor[MOTOR_2].p_foc->iqRef;
//#endif
//        IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_CPU1_TO_CM_FLAG);
//    }
    // Clear the interrupt overflow flag
    ADC_clearInterruptOverflowStatus(MOTOR12_IA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Clear the interrupt flag
    ADC_clearInterruptStatus(MOTOR12_IA_ADC_ADDR, MOTOR2_IA_INT_NUM);
    // Acknowledge PWM and CLA interrupt flag
    Interrupt_clearACKGroup(MOTOR2_IA_INT_ACK_GROUP);
#if (UOMODRI_V1_0_ENABLE)
    EXT_DBG_SPI_CLK_CLEAR;
#elif (UOMODRI_V2_0_ENABLE)
//    DBG_PIN2_CLEAR;
#endif
}

__attribute__((interrupt)) void cla_isrMotor1(void)
{
    tx_msg_full = COM_msgCreateLite(&motor_m1, &status_msg[tx_msg_nx]);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);//InterruptList[0].intAckGroup);
}

__attribute__((interrupt)) void cla_isrMotor2(void)
{
    tx_msg_full = COM_msgCreateLite(&motor_m2, &status_msg[tx_msg_nx]);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);//InterruptList[1].intAckGroup);
}

__attribute__((interrupt)) void tmr_isrLed(void)
{
//    if(CPUTimer_getTimerOverflowStatus(CPU_TIMER_0_BASE))
//     {
    // LED dimming control
    if(!DMA_getRunStatusFlag(DMA_LED_SPI_TX_BASE_ADDR))
    {
        LED_dimming(&motor_m1, &motor_m2, &led_msg[0]);
        DMA_startChannel(DMA_LED_SPI_TX_BASE_ADDR);
    }
    // Timeout management
    tmr_timeout = true;
    // Clear CPU timer flag
//    CPUTimer_clearOverflowFlag(CPU_TIMER_0_BASE);
//}
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
