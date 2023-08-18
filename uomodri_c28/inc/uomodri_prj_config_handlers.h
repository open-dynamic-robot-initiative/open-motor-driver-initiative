#ifndef __UOMODRI_PRJ_CONFIG_HANDLERS_H__
#define __UOMODRI_PRJ_CONFIG_HANDLERS_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
//#include "f2838x_device.h"
//#include "f2838x_examples.h"
#include <stdint.h>
#include <driverlib.h>

#include "hal.h"
#include "drv8353.h"
#include "foc.h"
#include "motor.h"
#include "communication.h"
#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define CPU1TOCMMSGRAM0_BASE                    (0x20080000U)
#define IPC_CPU1_TO_CM_ADDR_TRANSLATE(addr_cpu) ((((addr_cpu) - CPUXTOCMMSGRAM0_BASE) * 2U) + CPU1TOCMMSGRAM0_BASE)

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void cla_isrMotor1(void);
__attribute__((interrupt)) void cla_isrMotor2(void);
__attribute__((interrupt)) void tmr_isrLed(void);
__attribute__((interrupt)) void dma_isrComRx(void);

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   InterruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t InterruptList[] =
{
 {
  .intNum           = INT_CLA_MOTOR1,
  .intAckGroup      = INT_CLA_MOTOR1_GROUP,
  .p_intHandler     = INT_CLA_MOTOR1_HANDLER
 },
 {
  .intNum           = INT_CLA_MOTOR2,
  .intAckGroup      = INT_CLA_MOTOR2_GROUP,
  .p_intHandler     = INT_CLA_MOTOR2_HANDLER
 },
 {
  .intNum           = INT_TIMER_LED,
  .intAckGroup      = INT_TIMER_LED_GROUP,
  .p_intHandler     = INT_TIMER_LED_HANDLER
 },
 {
  .intNum           = INT_DMA_RX_MSG,
  .intAckGroup      = INT_DMA_RX_MSG_GROUP,
  .p_intHandler     = INT_DMA_RX_MSG_HANDLER
 },
 // END OF ARRAY
 {.intNum           = UINT32_MAX},
};

/** @var    DRV_Cfg     DrvCfgList[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
#pragma DATA_SECTION(drv_cfg, "ramgs0")
static const drv_cfg_t  drv_cfg[] =
{
 {
  .spiHandle        = DRV_SPI_BASE,
  .gpioNumber_CS    = MOTOR1_DRV_SPI_CS_NUM,
  .gpioNumber_EN    = MOTOR1_DRV_EN_NUM,
  .gpioNumber_FAULT = MOTOR1_DRV_FAULT_NUM,
 },
 {
  .spiHandle        = DRV_SPI_BASE,
  .gpioNumber_CS    = MOTOR2_DRV_SPI_CS_NUM,
  .gpioNumber_EN    = MOTOR2_DRV_EN_NUM,
  .gpioNumber_FAULT = MOTOR2_DRV_FAULT_NUM,
 },
};

/** @var    DRV_Reg     DrvRegList[]
 *  @brief  List all configuration registers for each DRVs.
 */
#pragma DATA_SECTION(drv_reg, "ramgs0")
static drv_reg_t drv_reg[] =
{
 {
  .ctrl_reg_02.bit.CLR_FLT      = false,
  .ctrl_reg_02.bit.BRAKE        = false,
  .ctrl_reg_02.bit.COAST        = false,
  .ctrl_reg_02.bit.PWM1_DIR     = false,
  .ctrl_reg_02.bit.PWM1_COM     = false,
  .ctrl_reg_02.bit.PWM_MODE     = DRV_PWMMODE_6,
  .ctrl_reg_02.bit.OTW_REP      = true, // Over Temperature Warning will raise nFault
  .ctrl_reg_02.bit.DIS_GDF      = false,
  .ctrl_reg_02.bit.DIS_GDUV     = false,
  .ctrl_reg_02.bit.OCP_ACT      = true,
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.bit.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.bit.IDRIVEP_HS   = DRV_ISOUR_HS_0P150_A,
  .ctrl_reg_03.bit.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.bit.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.bit.IDRIVEP_LS   = DRV_ISOUR_LS_0P150_A,
  .ctrl_reg_04.bit.TDRIVE       = DRV_TSOUR_1000_NS,
  .ctrl_reg_04.bit.CBC          = false,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.bit.VDS_LVL      = DRV_VDS_LVL_0P080_V,
  .ctrl_reg_05.bit.OCP_DEG      = DRV_OCPDEG_4_US,
  .ctrl_reg_05.bit.OCP_MODE     = DRV_LATCHED_SHUTDOWN,
  .ctrl_reg_05.bit.DEAD_TIME    = DRV_DEADTIME_50_NS,
  .ctrl_reg_05.bit.TRETRY       = true,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.bit.SEN_LVL      = DRV_SENLVL_0P25_V,
  .ctrl_reg_06.bit.CSA_CAL_C    = false,
  .ctrl_reg_06.bit.CSA_CAL_B    = false,
  .ctrl_reg_06.bit.CSA_CAL_A    = false,
  .ctrl_reg_06.bit.DIS_SEN      = false,
  .ctrl_reg_06.bit.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.bit.LS_REF       = false,
  .ctrl_reg_06.bit.VREF_DIV     = true,
  .ctrl_reg_06.bit.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
  .ctrl_reg_07.bit.CAL_MODE     = false,
 },
 {
  .ctrl_reg_02.bit.CLR_FLT      = false,
  .ctrl_reg_02.bit.BRAKE        = false,
  .ctrl_reg_02.bit.COAST        = false,
  .ctrl_reg_02.bit.PWM1_DIR     = false,
  .ctrl_reg_02.bit.PWM1_COM     = false,
  .ctrl_reg_02.bit.PWM_MODE     = DRV_PWMMODE_6,
  .ctrl_reg_02.bit.OTW_REP      = true, // Over Temperature Warning will raise nFault
  .ctrl_reg_02.bit.DIS_GDF      = false,
  .ctrl_reg_02.bit.DIS_GDUV     = false,
  .ctrl_reg_02.bit.OCP_ACT      = true,
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.bit.IDRIVEN_HS   = DRV_ISINK_HS_0P600_A,//DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.bit.IDRIVEP_HS   = DRV_ISOUR_HS_0P300_A,//DRV_ISOUR_HS_0P150_A,
  .ctrl_reg_03.bit.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.bit.IDRIVEN_LS   = DRV_ISINK_LS_0P600_A,//DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.bit.IDRIVEP_LS   = DRV_ISOUR_LS_0P300_A,//DRV_ISOUR_LS_0P150_A,
  .ctrl_reg_04.bit.TDRIVE       = DRV_TSOUR_1000_NS,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.bit.VDS_LVL      = DRV_VDS_LVL_0P080_V,
  .ctrl_reg_05.bit.OCP_DEG      = DRV_OCPDEG_4_US,
  .ctrl_reg_05.bit.OCP_MODE     = DRV_LATCHED_SHUTDOWN,
  .ctrl_reg_05.bit.DEAD_TIME    = DRV_DEADTIME_50_NS,
  .ctrl_reg_05.bit.TRETRY       = true,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.bit.SEN_LVL      = DRV_SENLVL_0P25_V,
  .ctrl_reg_06.bit.CSA_CAL_C    = false,
  .ctrl_reg_06.bit.CSA_CAL_B    = false,
  .ctrl_reg_06.bit.CSA_CAL_A    = false,
  .ctrl_reg_06.bit.DIS_SEN      = false,
  .ctrl_reg_06.bit.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.bit.LS_REF       = false,
  .ctrl_reg_06.bit.VREF_DIV     = true,
  .ctrl_reg_06.bit.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
  .ctrl_reg_07.bit.CAL_MODE     = false,
 },
};

//static const periph_motor_cfg_t periph_motor_cfg[] =
//{
// {
//  .p_pwmCntCmp[CHANNEL_1]   = &PwmCounterCompareCfgList[0],
//  .p_pwmCntCmp[CHANNEL_2]   = &PwmCounterCompareCfgList[1],
//  .p_pwmCntCmp[CHANNEL_3]   = &PwmCounterCompareCfgList[2],
//  .p_iAcq[CHANNEL_1]        = &AdcAcqList[0],
//  .p_iAcq[CHANNEL_2]        = &AdcAcqList[2],
//  .p_iAcq[CHANNEL_3]        = &AdcAcqList[4],
//  .p_intAcq                 = NULL,//&AdcIntList[0],
// },
// {
//  .p_pwmCntCmp[CHANNEL_1]   = &PwmCounterCompareCfgList[3],
//  .p_pwmCntCmp[CHANNEL_2]   = &PwmCounterCompareCfgList[4],
//  .p_pwmCntCmp[CHANNEL_3]   = &PwmCounterCompareCfgList[5],
//  .p_iAcq[CHANNEL_1]        = &AdcAcqList[1],
//  .p_iAcq[CHANNEL_2]        = &AdcAcqList[3],
//  .p_iAcq[CHANNEL_3]        = &AdcAcqList[5],
//  .p_intAcq                 = NULL,//&AdcIntList[1],
// },
//};

#pragma DATA_ALIGN(cmd_uomodri, 4)
#pragma DATA_SECTION(cmd_uomodri, "MSGRAM_CPU_TO_CLA")
cmd_t cmd_uomodri[2];

/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
#pragma DATA_ALIGN(motor_m1, 4)
#pragma DATA_SECTION(motor_m1, "ClaDataRam0");
motor_t motor_m1;
#pragma DATA_ALIGN(motor_m2, 4)
#pragma DATA_SECTION(motor_m2, "ClaDataRam1");
motor_t motor_m2;

#endif  /*__UOMODRI_PRJ_CONFIG_HANDLERS_H__*/
