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
//#include "uomodri_hal_config_handlers.h"
#include "config/uomodri_3_1/uomodri_3_1_hal_handlers.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define CPU1TOCMMSGRAM0_BASE                    (0x20080000U)
#define IPC_CPU1_TO_CM_ADDR_TRANSLATE(addr_cpu) ((((addr_cpu) - CPUXTOCMMSGRAM0_BASE) * 2U) + CPU1TOCMMSGRAM0_BASE)

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void cla_ClearTask(void);
__attribute__((interrupt)) void cla_isrMotor1(void);
__attribute__((interrupt)) void cla_isrMotor2(void);
__attribute__((interrupt)) void tmr_isrLedCmd(void);
__attribute__((interrupt)) void dma_isrSpiComRx(void);
__attribute__((interrupt)) void sci_isrRx_RS485(void);
__attribute__((interrupt)) void sci_isrTx_RS485(void);
__attribute__((interrupt)) void clb_isrTxRx_RS485(void);

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   InterruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t InterruptList[] =
{
 {
  .intNum           = INT_CLA_ADC_CALIB,
  .intAckGroup      = INT_CLA_ADC_CALIB_GROUP,
  .p_intHandler     = INT_CLA_ADC_HANDLER_HANDLER
 },
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
  .intAckGroup      = INT_TIMER_LED_CMD_GROUP,
  .p_intHandler     = INT_TIMER_LED_CMD_HANDLER
 },
 {
  .intNum           = INT_DMA_RX_MSG,
  .intAckGroup      = INT_DMA_RX_MSG_GROUP,
  .p_intHandler     = INT_DMA_RX_MSG_HANDLER
 },
#if (RS485_BUS_ENABLE) && (!UOMODRI_V1_0_ENABLE)
 {
  .intNum           = INT_RS485_TX_MSG,
  .intAckGroup      = INT_RS485_TX_MSG_GROUP,
  .p_intHandler     = INT_RS485_TX_MSG_HANDLER
 },
 {
  .intNum           = INT_RS485_RX_MSG,
  .intAckGroup      = INT_RS485_RX_MSG_GROUP,
  .p_intHandler     = INT_RS485_RX_MSG_HANDLER
 },
 {
  .intNum           = INT_RS485_Rx_Tx_CLB,
  .intAckGroup      = INT_RS485_RX_Tx_CLB_GROUP,
  .p_intHandler     = INT_RS485_RX_Tx_CLB_HANDLER
 },
#endif
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
  .ctrl_reg_02.CLR_FLT      = false,
  .ctrl_reg_02.BRAKE        = false,
  .ctrl_reg_02.COAST        = false,
  .ctrl_reg_02.PWM1_DIR     = false,
  .ctrl_reg_02.PWM1_COM     = false,
  .ctrl_reg_02.PWM_MODE     = DRV_PWMMODE_6,
  .ctrl_reg_02.OTW_REP      = true, // Over Temperature Warning will raise nFault
  .ctrl_reg_02.DIS_GDF      = false,
  .ctrl_reg_02.DIS_GDUV     = false,
  .ctrl_reg_02.OCP_ACT      = true,
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.IDRIVEP_HS   = DRV_ISOUR_HS_0P150_A,
  .ctrl_reg_03.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.IDRIVEP_LS   = DRV_ISOUR_LS_0P150_A,
  .ctrl_reg_04.TDRIVE       = DRV_TSOUR_1000_NS,
  .ctrl_reg_04.CBC          = false,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.VDS_LVL      = DRV_VDS_LVL_0P080_V,
  .ctrl_reg_05.OCP_DEG      = DRV_OCPDEG_4_US,
  .ctrl_reg_05.OCP_MODE     = DRV_LATCHED_SHUTDOWN,
  .ctrl_reg_05.DEAD_TIME    = DRV_DEADTIME_50_NS,
  .ctrl_reg_05.TRETRY       = true,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.SEN_LVL      = DRV_SENLVL_0P25_V,
  .ctrl_reg_06.CSA_CAL_C    = false,
  .ctrl_reg_06.CSA_CAL_B    = false,
  .ctrl_reg_06.CSA_CAL_A    = false,
  .ctrl_reg_06.DIS_SEN      = false,
  .ctrl_reg_06.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.LS_REF       = false,
  .ctrl_reg_06.VREF_DIV     = true,
  .ctrl_reg_06.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
  .ctrl_reg_07.CAL_MODE     = false,
 },
 {
  .ctrl_reg_02.CLR_FLT      = false,
  .ctrl_reg_02.BRAKE        = false,
  .ctrl_reg_02.COAST        = false,
  .ctrl_reg_02.PWM1_DIR     = false,
  .ctrl_reg_02.PWM1_COM     = false,
  .ctrl_reg_02.PWM_MODE     = DRV_PWMMODE_6,
  .ctrl_reg_02.OTW_REP      = true, // Over Temperature Warning will raise nFault
  .ctrl_reg_02.DIS_GDF      = false,
  .ctrl_reg_02.DIS_GDUV     = false,
  .ctrl_reg_02.OCP_ACT      = true,
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.IDRIVEN_HS   = DRV_ISINK_HS_0P600_A,//DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.IDRIVEP_HS   = DRV_ISOUR_HS_0P300_A,//DRV_ISOUR_HS_0P150_A,
  .ctrl_reg_03.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.IDRIVEN_LS   = DRV_ISINK_LS_0P600_A,//DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.IDRIVEP_LS   = DRV_ISOUR_LS_0P300_A,//DRV_ISOUR_LS_0P150_A,
  .ctrl_reg_04.TDRIVE       = DRV_TSOUR_1000_NS,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.VDS_LVL      = DRV_VDS_LVL_0P080_V,
  .ctrl_reg_05.OCP_DEG      = DRV_OCPDEG_4_US,
  .ctrl_reg_05.OCP_MODE     = DRV_LATCHED_SHUTDOWN,
  .ctrl_reg_05.DEAD_TIME    = DRV_DEADTIME_50_NS,
  .ctrl_reg_05.TRETRY       = true,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.SEN_LVL      = DRV_SENLVL_0P25_V,
  .ctrl_reg_06.CSA_CAL_C    = false,
  .ctrl_reg_06.CSA_CAL_B    = false,
  .ctrl_reg_06.CSA_CAL_A    = false,
  .ctrl_reg_06.DIS_SEN      = false,
  .ctrl_reg_06.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.LS_REF       = false,
  .ctrl_reg_06.VREF_DIV     = true,
  .ctrl_reg_06.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
  .ctrl_reg_07.CAL_MODE     = false,
 },
};

//#pragma DATA_ALIGN(cmd_uomodri, 4)
//#pragma DATA_SECTION(cmd_uomodri, "MSGRAM_CPU_TO_CLA")
//cmd_t cmd_uomodri[2];

///** @var    MOTOR_STRUCT    motor[]
// *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
// */
//#pragma DATA_ALIGN(motor_m1, 4)
//#pragma DATA_SECTION(motor_m1, "ClaDataMotor1");
//motor_t motor_m1;
//#pragma DATA_ALIGN(motor_m2, 4)
//#pragma DATA_SECTION(motor_m2, "ClaDataMotor2");
//motor_t motor_m2;

#endif  /*__UOMODRI_PRJ_CONFIG_HANDLERS_H__*/
