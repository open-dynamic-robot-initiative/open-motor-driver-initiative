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
//__attribute__((interrupt)) void adc_isrMotor1(void);
//__attribute__((interrupt)) void adc_isrMotor2(void);
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
  .intNum           = INT_CLA1_2,
  .intAckGroup      = INTERRUPT_ACK_GROUP11,
  .p_intHandler     = &cla_isrMotor1
 },
 {
  .intNum           = INT_CLA1_3,
  .intAckGroup      = INTERRUPT_ACK_GROUP11,
  .p_intHandler     = &cla_isrMotor2
 },
 {
  .intNum           = INT_TIMER0,
  .intAckGroup      = INTERRUPT_ACK_GROUP1,
  .p_intHandler     = &tmr_isrLed
 },
 {
  .intNum           = INT_DMA_CH6,
  .intAckGroup      = INTERRUPT_ACK_GROUP7,
  .p_intHandler     = &dma_isrComRx
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

static const periph_motor_cfg_t periph_motor_cfg[] =
{
 {
  .p_pwmCntCmp[CHANNEL_1]   = &PwmCounterCompareCfgList[0],
  .p_pwmCntCmp[CHANNEL_2]   = &PwmCounterCompareCfgList[1],
  .p_pwmCntCmp[CHANNEL_3]   = &PwmCounterCompareCfgList[2],
  .p_iAcq[CHANNEL_1]        = &AdcAcqList[0],
  .p_iAcq[CHANNEL_2]        = &AdcAcqList[2],
  .p_iAcq[CHANNEL_3]        = &AdcAcqList[4],
  .p_intAcq                 = &AdcIntList[0],
 },
 {
  .p_pwmCntCmp[CHANNEL_1]   = &PwmCounterCompareCfgList[3],
  .p_pwmCntCmp[CHANNEL_2]   = &PwmCounterCompareCfgList[4],
  .p_pwmCntCmp[CHANNEL_3]   = &PwmCounterCompareCfgList[5],
  .p_iAcq[CHANNEL_1]        = &AdcAcqList[1],
  .p_iAcq[CHANNEL_2]        = &AdcAcqList[3],
  .p_iAcq[CHANNEL_3]        = &AdcAcqList[5],
  .p_intAcq                 = &AdcIntList[1],
 },
};

#if (CLA_CORE_ENABLE)
#pragma DATA_ALIGN(cmd_uOmodri, 4)
#pragma DATA_SECTION(cmd_uOmodri, "MSGRAM_CPU_TO_CLA")
cmd_t cmd_uOmodri[2];
#else
#pragma DATA_ALIGN(cmd_uOmodri, 4)
#pragma DATA_SECTION(cmd_uOmodri, "ramgs0");
static cmd_t cmd_uOmodri[2] =

{
 {
  .posRef           = 0.0f,
  .velRef           = 0.0f,
  .iqff             = 0.0f,
  .kpCoeff          = 0.0f,
  .kdCoeff          = 0.0f,
  .iSat             = 0.0f,
  .timeoutRef       = 0U,
  .cptTimeout       = 0U,
  .index            = 0U,
  .cmdArray.all     = 0U,
 },
 {
  .posRef           = 0.0f,
  .velRef           = 0.0f,
  .iqff             = 0.0f,
  .kpCoeff          = 0.0f,
  .kdCoeff          = 0.0f,
  .iSat             = 0.0f,
  .timeoutRef       = 0U,
  .cptTimeout       = 0U,
  .index            = 0U,
  .cmdArray.all     = 0U,
 },
};
#endif

/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
#if (CLA_CORE_ENABLE)
#pragma DATA_ALIGN(motor_m1, 4)
#pragma DATA_SECTION(motor_m1, "ClaDataRam0");
motor_t motor_m1;
#pragma DATA_ALIGN(motor_m2, 4)
#pragma DATA_SECTION(motor_m2, "ClaDataRam1");
motor_t motor_m2;
#else
#pragma DATA_ALIGN(motor, 4)
#pragma DATA_SECTION(motor, "ramgs0");
static motor_t motor[2] =
{
 {
  //--- MOTOR_1 ---------------------------------------------------------------
  .motor_id                                 = MOTOR_1,
  //--- FOC structure address -------------------------------------------------
  .p_foc                                    = &motor_foc[MOTOR_1],
  //--- FSM motor initial state -----------------------------------------------
  .motor_state                              = MOTOR_STATE_START,
  //--- Error message ---------------------------------------------------------
  .motor_error.all                          = 0,
  //---------------------------------------------------------------------------
  .clCycleNb                                = 0,
  .itCnt                                    = 0U,
  .itDone                                   = false,
  //--- DRV IO Config ---------------------------------------------------------
  .drvFaultNum                              = MOTOR1_DRV_FAULT_NUM,
  .drvEnNum                                 = MOTOR1_DRV_EN_NUM,
  //---- PWM config address ---------------------------------------------------
  .epwmBaseAddr[CHANNEL_1]                  = MOTOR1_PWM1_BASE,
  .epwmBaseAddr[CHANNEL_2]                  = MOTOR1_PWM2_BASE,
  .epwmBaseAddr[CHANNEL_3]                  = MOTOR1_PWM3_BASE,
  //---- PWM command address --------------------------------------------------
  .p_cmdReg[CHANNEL_1]                      = (volatile uint16_t *)(MOTOR1_PWM1_CMD_ADDR),
  .p_cmdReg[CHANNEL_2]                      = (volatile uint16_t *)(MOTOR1_PWM2_CMD_ADDR),
  .p_cmdReg[CHANNEL_3]                      = (volatile uint16_t *)(MOTOR1_PWM3_CMD_ADDR),
 },
 {
  //--- MOTOR_1 ---------------------------------------------------------------
  .motor_id                                 = MOTOR_2,
  //--- FOC structure address -------------------------------------------------
  .p_foc                                    = &motor_foc[MOTOR_2],
  //--- FSM motor initial state -----------------------------------------------
  .motor_state                              = MOTOR_STATE_START,
  //--- Error message ---------------------------------------------------------
  .motor_error.all                          = 0,
  //---------------------------------------------------------------------------
  .clCycleNb                                = 0,
  .itCnt                                    = 0U,
  .itDone                                   = false,
  //--- DRV IO Config ---------------------------------------------------------
  .drvFaultNum                              = MOTOR2_DRV_FAULT_NUM,
  .drvEnNum                                 = MOTOR2_DRV_EN_NUM,
  //---- PWM periph config address --------------------------------------------
  .epwmBaseAddr[CHANNEL_1]                  = MOTOR2_PWM1_BASE,
  .epwmBaseAddr[CHANNEL_2]                  = MOTOR2_PWM2_BASE,
  .epwmBaseAddr[CHANNEL_3]                  = MOTOR2_PWM3_BASE,
  //---- PWM command reg address ----------------------------------------------
  .p_cmdReg[CHANNEL_1]                      = (volatile uint16_t *)(MOTOR2_PWM1_CMD_ADDR),
  .p_cmdReg[CHANNEL_2]                      = (volatile uint16_t *)(MOTOR2_PWM2_CMD_ADDR),
  .p_cmdReg[CHANNEL_3]                      = (volatile uint16_t *)(MOTOR2_PWM3_CMD_ADDR),
 },
};
#endif

static const dma_cfg_t  dmaMem2MemCfgList[] =
{
 // DMA base register initialization
 // DMA to duplicate message from uOmodri command struct to foc struct.
#if (!CLA_CORE_ENABLE)
 {
  .dmaChBase        = DMA_CMD_2_FOC_BASE_ADDR,
  .p_dstAddr        = (uint16_t *)&motor_foc[MOTOR_1].motor_cmd,
  .dstStep          = 1,
  .p_srcAddr        = (uint16_t *)&cmd_uOmodri[MOTOR_1],
  .srcStep          = 1,
  .burstSize        = sizeof(cmd_t),
  .transferSize     = 2,//DMA_MEM_TO_MEM_M1_16BIT_TRANSFER_EVT,
  .dstWrapStep      = sizeof(foc_t),
  .dstWrapSize      = 1,
  .srcWrapStep      = sizeof(cmd_t),
  .srcWrapSize      = 1,
  .config           = DMA_CFG_ONESHOT_ENABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT,
  .trigger          = DMA_TRIGGER_SOFTWARE,
  .intMode          = DMA_INT_AT_END,
  .intEnable        = false,
 },
#endif
 // END OF ARRAY
 {.dmaChBase        = UINT32_MAX},
};

#endif  /*__UOMODRI_PRJ_CONFIG_HANDLERS_H__*/
