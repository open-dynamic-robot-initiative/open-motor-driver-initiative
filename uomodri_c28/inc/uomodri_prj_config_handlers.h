#ifndef __UOMODRI_PRJ_CONFIG_HANDLERS_H__
#define __UOMODRI_PRJ_CONFIG_HANDLERS_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
//#include "f2838x_device.h"
//#include "f2838x_examples.h"
#include <stdint.h>
#include "driverlib.h"

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
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   InterruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t InterruptList[] =
{
 {
  .intNum           = MOTOR1_IA_INT_CH,
  .intAckGroup      = MOTOR1_IA_INT_ACK_GROUP,
.p_intHandler       = &adc_isrMotor1
 },
 {
  .intNum           = MOTOR2_IA_INT_CH,
.intAckGroup        = MOTOR2_IA_INT_ACK_GROUP,
.p_intHandler       = &adc_isrMotor2
 },
 // END OF ARRAY
 {.intNum           = UINT32_MAX},
};

/** @var    DRV_Cfg     DrvCfgList[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
static const drv_cfg_t  DrvCfgList[] =
{
 {
  .spiHandle        = DRV_SPI_BASE,
  .gpioNumber_CS    = MOTOR1_DRV_SPI_CS,
  .gpioNumber_EN    = MOTOR1_DRV_GPIO_EN,
  .gpioNumber_FAULT = MOTOR1_DRV_GPIO_NFAULT,
 },
 {
  .spiHandle        = DRV_SPI_BASE,
  .gpioNumber_CS    = MOTOR2_DRV_SPI_CS,
  .gpioNumber_EN    = MOTOR2_DRV_GPIO_EN,
  .gpioNumber_FAULT = MOTOR2_DRV_GPIO_NFAULT,
 },
};

/** @var    DRV_Reg     DrvRegList[]
 *  @brief  List all configuration registers for each DRVs.
 */
static drv_reg_t        DrvRegList[] =
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
  .ctrl_reg_03.bit.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.bit.IDRIVEP_HS   = DRV_ISOUR_HS_0P150_A,
  .ctrl_reg_03.bit.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.bit.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.bit.IDRIVEP_LS   = DRV_ISOUR_LS_0P150_A,
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

/** @var    drv8353_t   drv_cfg[]
 *  @brief  General DRV8353 configuration structure.
 */
static drv8353_t drv_cfg[] =
{
 {
  .p_drvCfgHandler  = &DrvCfgList[MOTOR_1],
  .p_drvRegHandler  = &DrvRegList[MOTOR_1],
 },
 {
  .p_drvCfgHandler  = &DrvCfgList[MOTOR_2],
  .p_drvRegHandler  = &DrvRegList[MOTOR_2],
 },
};

static const hal_motor_cfg_t hal_motor_cfg[] =
{
 {
  .p_pwmCntCmp[0]   = &PwmCounterCompareCfgList[0],
  .p_pwmCntCmp[1]   = &PwmCounterCompareCfgList[1],
  .p_pwmCntCmp[2]   = &PwmCounterCompareCfgList[2],
  .p_iAcq[0]        = &AdcAcqList[0],
  .p_iAcq[1]        = &AdcAcqList[2],
  .p_iAcq[2]        = &AdcAcqList[4],
  .p_intAcq         = &AdcIntList[0],
 },
 {
  .p_pwmCntCmp[0]   = &PwmCounterCompareCfgList[3],
  .p_pwmCntCmp[1]   = &PwmCounterCompareCfgList[4],
  .p_pwmCntCmp[2]   = &PwmCounterCompareCfgList[5],
  .p_iAcq[0]        = &AdcAcqList[1],
  .p_iAcq[1]        = &AdcAcqList[3],
  .p_iAcq[2]        = &AdcAcqList[5],
  .p_intAcq         = &AdcIntList[1],
 },
};

#pragma DATA_ALIGN(cmd_uOmodri, 8)
#pragma DATA_SECTION(cmd_uOmodri, "MSGRAM_CPU_TO_CM")
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
  .enableReg.all    = 0U,
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
  .enableReg.all    = 0U,
 },
};

#pragma DATA_ALIGN(motor_foc, 8)
#if (CM_CORE_ENABLE)
#pragma DATA_SECTION(motor_foc, "MSGRAM_CPU_TO_CM");
#else
#pragma DATA_SECTION(motor_foc, "ramgs1");
#endif
static foc_t motor_foc[] =
{
 {
  //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
  .motor_cfg.Rs                         = MOTOR1_RS,
  .motor_cfg.Ls                         = MOTOR1_LS,
  .motor_cfg.kv                         = MOTOR1_KV,
  .motor_cfg.ke                         = MOTOR1_KE,
  .motor_cfg.ki                         = MOTOR1_KI,
  .motor_cfg.polePairs                  = MOTOR1_POLES_PAIRS,
  //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
  .motor_acq.p_vExtMeasReg              = (volatile uint16_t *)(VEXT1_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg              = (volatile uint16_t *)(VBUS_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iaMeasReg                = (volatile int16_t *)(MOTOR1_IA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_ibMeasReg                = (volatile int16_t *)(MOTOR1_IB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_icMeasReg                = (volatile int16_t *)(MOTOR1_IC_ADC_PPB_RESULT_ADDR),
  .motor_acq.vbus                       = 0.0f,
  .motor_acq.vExt                       = 0.0f,
  .motor_acq.ia                         = 0.0f,
  .motor_acq.ib                         = 0.0f,
  .motor_acq.ic                         = 0.0f,
  .motor_acq.vBusFlt.a                  = VBUS_LPF_ALPHA,
  .motor_acq.vBusFlt.one_minus_a        = VBUS_LPF_ONE_M_ALPHA,
  .motor_acq.vExtFlt.a                  = VEXT_LPF_ALPHA,
  .motor_acq.vExtFlt.one_minus_a        = VEXT_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.p_qepHandle                = (&QepCfgList[MOTOR_1]),
  .motor_enc.polePairs                  = MOTOR1_POLES_PAIRS,
  .motor_enc.thetaElec                  = 0.0f,
  .motor_enc.thetaMech[NEW]             = 0.0f,
  .motor_enc.thetaMech[OLD]             = 0.0f,
  .motor_enc.thetaMechScaler            = (float32_t)MOTOR1_ENC_RESOLUTION_SCALE,
  .motor_enc.thetaIndex                 = 0.0f,
//  .motor_enc.thetaMechIndex             = 0.0f,
  .motor_enc.thetaAbsolute              = 0.0f,
//  .motor_enc.thetaAbsIndex              = 0.0f,
  .motor_enc.turnNb                     = 0,
  .motor_enc.thetaDir                   = 0,
  .motor_enc.indexDetect                = false,
  .motor_enc.indexOffset                = false,
  .motor_enc.indexToggle                = false,
  .motor_enc.indexError                 = false,
  .motor_enc.rollOverError              = false,
  .motor_enc.speed.speedHighScaler      = (float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speed.speedHigh            = 0.0f,
  .motor_enc.speed.speedLowScaler       = (float32_t)MOTOR1_ENC_SPEED_LOW_SCALE,
  .motor_enc.speed.speedLow             = 0.0f,
  .motor_enc.speed.alpha                = 0.0f,
  //  .motor_enc.speed.speedRef             = 0.0f,
  .motor_enc.speed.speedMech            = 0.0f,
  .motor_enc.speed.speedElec            = 0.0f,
  .motor_enc.speed.theta[NEW]           = 0.0f,
  .motor_enc.speed.theta[OLD]           = 0.0f,
  .motor_enc.speed.speedFlt.a           = (float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speed.speedFlt.one_minus_a = (float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA,
  //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
  .motor_cmd.posRef                     = 0.0f,
  .motor_cmd.velRef                     = 0.0f,
  .motor_cmd.iqff                       = 0.0f,
  .motor_cmd.kpCoeff                    = 0.0f,
  .motor_cmd.kdCoeff                    = 0.0f,
  .motor_cmd.iSat                       = MOTOR1_CURRENT_CMD_SAT_MAX,
  .motor_cmd.timeoutRef                 = 0U,
  .motor_cmd.cptTimeout                 = 0U,
  .motor_cmd.index                      = 0U,
  .motor_cmd.enableReg.all              = 0U,

  //--- Maximum current alignment value ---------------------------------------
  .iAlignMax                            = MOTOR1_CURRENT_ALIGN_MAX,
  //---------------------------------------------------------------------------
  .iParkFlt.a                           = MOTOR1_CURRENT_LPF_ALPHA,
  .iParkFlt.one_minus_a                 = MOTOR1_CURRENT_LPF_ONE_M_ALPHA,
  //--- FOC - Sine / Cosine computation of angular position -------------------
  .sinTheta                             = 0.0f,
  .cosTheta                             = 0.0f,
  //--- FOC - Clarke transform results ----------------------------------------
  .ialpha                               = 0.0f,
  .ibeta                                = 0.0f,
  //--- FOC - Park transform results ------------------------------------------
  .id                                   = 0.0f,
  .iq                                   = 0.0f,
  //--- FOC - Inverse Park transform results ----------------------------------
  .ud                                   = 0.0f,
  .uq                                   = 0.0f,
  //--- FOC - Inverse Clarke transform results --------------------------------
  .ualpha                               = 0.0f,
  .ubeta                                = 0.0f,
  //--- FOC - Phase commands --------------------------------------------------
  .ua                                   = 0.0f,
  .ub                                   = 0.0f,
  .uc                                   = 0.0f,
  //--- FOC - Max current and voltage -----------------------------------------
  .vmax                                 = 0.0f,
//  .imax                                 = MOTOR12_IMAX,
  //--- FOC - Initialize duty cycle saturation --------------------------------
  .dtcMax                               = MOTOR1_DTC_MAX,
  .dtcMin                               = MOTOR1_DTC_MIN,
  //--- FOC - Duty cycles -----------------------------------------------------
  .dtc_u                                = 0.0f,
  .dtc_v                                = 0.0f,
  .dtc_w                                = 0.0f,
  //--- FOC - PI setpoint values ----------------------------------------------
  .idRef                                = 0.0f,
  .iqRef                                = 0.0f,
  //--- FOC - Initialize PI controller (Id) -----------------------------------
  .piId.p_fb                            = (float32_t *)(&motor_foc[MOTOR_1].id),
  .piId.p_set                           = (float32_t *)(&motor_foc[MOTOR_1].idRef),
  .piId.p_sat                           = (float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus),
  .piId.err                             = 0.0f,
  .piId.kp                              = (float32_t)MOTOR1_PI_ID_KP_COEF,
  .piId.ki                              = (float32_t)MOTOR1_PI_ID_KI_COEF,
  .piId.integral                        = 0.0f,
  .piId.ff                              = 0.0f,
  .piId.out                             = 0.0f,
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.p_fb                            = (float32_t *)(&motor_foc[MOTOR_1].iq),
  .piIq.p_set                           = (float32_t *)(&motor_foc[MOTOR_1].iqRef),
  .piIq.p_sat                           = (float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus),
  .piIq.err                             = 0.0f,
  .piIq.kp                              = (float32_t)MOTOR1_PI_IQ_KP_COEF,
  .piIq.ki                              = (float32_t)MOTOR1_PI_IQ_KI_COEF,
  .piIq.integral                        = 0.0f,
  .piIq.ff                              = 0.0f,
  .piIq.out                             = 0.0f,
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPosVel.p_fbTheta                   = (float32_t *)(&motor_foc[MOTOR_1].motor_enc.thetaAbsolute),
  .pdPosVel.p_setTheta                  = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.posRef),
  .pdPosVel.errTheta                    = 0.0f,
  .pdPosVel.p_fbSpeed                   = (float32_t *)(&motor_foc[MOTOR_1].motor_enc.speed.speedMech),
  .pdPosVel.p_setSpeed                  = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.velRef),
  .pdPosVel.errSpeed                    = 0.0f,
  .pdPosVel.p_kp                        = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.kpCoeff),
  .pdPosVel.p_kd                        = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.kdCoeff),
  .pdPosVel.derivative                  = 0.0f,
  .pdPosVel.p_ff                        = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.iqff),
  .pdPosVel.p_sat                       = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.iSat),
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resEst                               = 1.0f,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstFlt.a                          = MOTOR1_STATOR_RESISTOR_LPF_ALPHA,
  .resEstFlt.one_minus_a                = MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
 },
 //---------------------------------------------------------------------------
 //---------------------------------------------------------------------------
 {
  //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
  .motor_cfg.Rs                         = MOTOR2_RS,
  .motor_cfg.Ls                         = MOTOR2_LS,
  .motor_cfg.kv                         = MOTOR2_KV,
  .motor_cfg.ke                         = MOTOR2_KE,
  .motor_cfg.ki                         = MOTOR2_KI,
  .motor_cfg.polePairs                  = MOTOR2_POLES_PAIRS,
  //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
  .motor_acq.p_vExtMeasReg              = (volatile uint16_t *)(VEXT2_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg              = (volatile uint16_t *)(VBUS_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iaMeasReg                = (volatile int16_t *)(MOTOR2_IA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_ibMeasReg                = (volatile int16_t *)(MOTOR2_IB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_icMeasReg                = (volatile int16_t *)(MOTOR2_IC_ADC_PPB_RESULT_ADDR),
  .motor_acq.vbus                       = 0.0f,
  .motor_acq.vExt                       = 0.0f,
  .motor_acq.ia                         = 0.0f,
  .motor_acq.ib                         = 0.0f,
  .motor_acq.ic                         = 0.0f,
  .motor_acq.vBusFlt.a                  = VBUS_LPF_ALPHA,
  .motor_acq.vBusFlt.one_minus_a        = VBUS_LPF_ONE_M_ALPHA,
  .motor_acq.vExtFlt.a                  = VEXT_LPF_ALPHA,
  .motor_acq.vExtFlt.one_minus_a        = VEXT_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.p_qepHandle                = (&QepCfgList[MOTOR_2]),
  .motor_enc.polePairs                  = MOTOR2_POLES_PAIRS,
  .motor_enc.thetaElec                  = 0.0f,
  .motor_enc.thetaMech[NEW]             = 0.0f,
  .motor_enc.thetaMech[OLD]             = 0.0f,
  .motor_enc.thetaMechScaler            = (float32_t)MOTOR2_ENC_RESOLUTION_SCALE,
  .motor_enc.thetaIndex                 = 0.0f,
//  .motor_enc.thetaMechIndex             = 0.0f,
  .motor_enc.thetaAbsolute              = 0.0f,
//  .motor_enc.thetaAbsIndex              = 0.0f,
  .motor_enc.turnNb                     = 0,
  .motor_enc.thetaDir                   = 0,
  .motor_enc.indexDetect                = false,
  .motor_enc.indexOffset                = false,
  .motor_enc.indexToggle                = false,
  .motor_enc.indexError                 = false,
  .motor_enc.rollOverError              = false,
  .motor_enc.speed.speedHighScaler      = (float32_t)MOTOR2_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speed.speedHigh            = 0.0f,
  .motor_enc.speed.speedLowScaler       = (float32_t)MOTOR2_ENC_SPEED_LOW_SCALE,
  .motor_enc.speed.speedLow             = 0.0f,
  .motor_enc.speed.alpha                = 0.0f,
  //  .motor_enc.speed.speedRef             = 0.0f,
  .motor_enc.speed.speedMech            = 0.0f,
  .motor_enc.speed.speedElec            = 0.0f,
  .motor_enc.speed.theta[NEW]           = 0.0f,
  .motor_enc.speed.theta[OLD]           = 0.0f,
  .motor_enc.speed.speedFlt.a           = (float32_t)MOTOR2_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speed.speedFlt.one_minus_a = (float32_t)MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA,
  //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
  .motor_cmd.posRef                     = 0.0f,
  .motor_cmd.velRef                     = 0.0f,
  .motor_cmd.iqff                       = 0.0f,
  .motor_cmd.kpCoeff                    = 0.0f,
  .motor_cmd.kdCoeff                    = 0.0f,
  .motor_cmd.iSat                       = MOTOR2_CURRENT_CMD_SAT_MAX,
  .motor_cmd.timeoutRef                 = 0U,
  .motor_cmd.cptTimeout                 = 0U,
  .motor_cmd.index                      = 0U,
  .motor_cmd.enableReg.all              = 0U,
//  .motor_cmd[ODD].posRef                = 0.0f,
//  .motor_cmd[ODD].velRef                = 0.0f,
//  .motor_cmd[ODD].iqRef                 = 0.0f,
//  .motor_cmd[ODD].kpCoeff               = 0.0f,
//  .motor_cmd[ODD].kdCoeff               = 0.0f,
//  .motor_cmd[ODD].iSat                  = MOTOR2_CURRENT_CMD_SAT_MAX,
//  .motor_cmd[ODD].timeoutRef            = 0U,
//  .motor_cmd[ODD].cptTimeout            = 0U,
//  .motor_cmd[ODD].index                 = 0U,
//  .motor_cmd[ODD].enableReg.all         = 0U,
//  .motor_cmd[EVEN].posRef               = 0.0f,
//  .motor_cmd[EVEN].velRef               = 0.0f,
//  .motor_cmd[EVEN].iqRef                = 0.0f,
//  .motor_cmd[EVEN].kpCoeff              = 0.0f,
//  .motor_cmd[EVEN].kdCoeff              = 0.0f,
//  .motor_cmd[EVEN].iSat                 = MOTOR2_CURRENT_CMD_SAT_MAX,
//  .motor_cmd[EVEN].timeoutRef           = 0U,
//  .motor_cmd[EVEN].cptTimeout           = 0U,
//  .motor_cmd[EVEN].index                = 0U,
//  .motor_cmd[EVEN].enableReg.all        = 0U,
  //--- Maximum current alignment value ---------------------------------------
  .iAlignMax                            = MOTOR2_CURRENT_ALIGN_MAX,
  //---------------------------------------------------------------------------
  .iParkFlt.a                           = MOTOR2_CURRENT_LPF_ALPHA,
  .iParkFlt.one_minus_a                 = MOTOR2_CURRENT_LPF_ONE_M_ALPHA,
  //--- FOC - Sine / Cosine computation of angular position -------------------
  .sinTheta                             = 0.0f,
  .cosTheta                             = 0.0f,
  //--- FOC - Clarke transform results ----------------------------------------
  .ialpha                               = 0.0f,
  .ibeta                                = 0.0f,
  //--- FOC - Park transform results ------------------------------------------
  .id                                   = 0.0f,
  .iq                                   = 0.0f,
  //--- FOC - Inverse Park transform results ----------------------------------
  .ud                                   = 0.0f,
  .uq                                   = 0.0f,
  //--- FOC - Inverse Clarke transform results --------------------------------
  .ualpha                               = 0.0f,
  .ubeta                                = 0.0f,
  //--- FOC - Phase commands --------------------------------------------------
  .ua                                   = 0.0f,
  .ub                                   = 0.0f,
  .uc                                   = 0.0f,
  //--- FOC - Max current and voltage -----------------------------------------
  .vmax                                 = 0.0f,
//  .imax                                 = MOTOR12_IMAX,
  //--- FOC - Initialize duty cycle saturation --------------------------------
  .dtcMax                               = MOTOR2_DTC_MAX,
  .dtcMin                               = MOTOR2_DTC_MIN,
  //--- FOC - Duty cycles -----------------------------------------------------
  .dtc_u                                = 0.0f,
  .dtc_v                                = 0.0f,
  .dtc_w                                = 0.0f,
  //--- FOC - PI setpoint values ----------------------------------------------
  .idRef                                = 0.0f,
  .iqRef                                = 0.0f,
  //--- FOC - Initialize PI controller (Id) -----------------------------------
  .piId.p_fb                            = (float32_t *)(&motor_foc[MOTOR_2].id),
  .piId.p_set                           = (float32_t *)(&motor_foc[MOTOR_2].idRef),
  .piId.p_sat                           = (float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus),
  .piId.err                             = 0.0f,
  .piId.kp                              = (float32_t)MOTOR2_PI_ID_KP_COEF,
  .piId.ki                              = (float32_t)MOTOR2_PI_ID_KI_COEF,
  .piId.integral                        = 0.0f,
  .piId.ff                              = 0.0f,
  .piId.out                             = 0.0f,
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.p_fb                            = (float32_t *)(&motor_foc[MOTOR_2].iq),
  .piIq.p_set                           = (float32_t *)(&motor_foc[MOTOR_2].iqRef),
  .piIq.p_sat                           = (float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus),
  .piIq.err                             = 0.0f,
  .piIq.kp                              = (float32_t)MOTOR2_PI_IQ_KP_COEF,
  .piIq.ki                              = (float32_t)MOTOR2_PI_IQ_KI_COEF,
  .piIq.integral                        = 0.0f,
  .piIq.ff                              = 0.0f,
  .piIq.out                             = 0.0f,
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPosVel.p_fbTheta                   = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.thetaAbsolute),
  .pdPosVel.p_setTheta                  = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.posRef),
  .pdPosVel.errTheta                    = 0.0f,
  .pdPosVel.p_fbSpeed                   = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.speed.speedMech),
  .pdPosVel.p_setSpeed                  = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.velRef),
  .pdPosVel.errSpeed                    = 0.0f,
  .pdPosVel.p_kp                        = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.kpCoeff),
  .pdPosVel.p_kd                        = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.kdCoeff),
  .pdPosVel.derivative                  = 0.0f,
  .pdPosVel.p_ff                        = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.iqff),
  .pdPosVel.p_sat                       = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.iSat),
  .pdPosVel.out                         = 0.0f,
//  .pdPosVel.p_fbTheta                   = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.thetaAbsolute),
//  .pdPosVel.setTheta                    = 0.0f,
//  .pdPosVel.errTheta                    = 0.0f,
//  .pdPosVel.p_fbSpeed                   = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.speed.speedMech),
//  .pdPosVel.setSpeed                    = 0.0f,
//  .pdPosVel.errSpeed                    = 0.0f,
//  .pdPosVel.kp                          = 0.0f,
//  .pdPosVel.kd                          = 0.0f,
//  .pdPosVel.derivative                  = 0.0f,
//  .pdPosVel.ff                          = 0.0f,
//  .pdPosVel.sat                         = 0.0f,
//  .pdPosVel.out                            = 0.0f,
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resEst                               = 0.0f,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstFlt.a                          = MOTOR2_STATOR_RESISTOR_LPF_ALPHA,
  .resEstFlt.one_minus_a                = MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
  //--- Which command set is currently active ? -------------------------------
//  .cmd_cr_active                        = ODD,
//  .cmd_nx_active                        = ODD,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
 },
};

/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
#if (CM_CORE_ENABLE)
#pragma DATA_SECTION(motor, "MSGRAM_CPU_TO_CM");
#else
#pragma DATA_SECTION(motor, "ramgs1");
#endif
static motor_t motor[2] =
{
#if (CLA_CORE_ENABLE)
 {
  //--- MOTOR_1 ---------------------------------------------------------------
  .motor_id         = MOTOR_1,
  .motorHalCfg_u.ptr= &hal_motor_cfg[MOTOR_1],
  .motorDRV_u.ptr   = &drv_cfg[MOTOR_1],
  .motorFOC_u.ptr   = &motor_foc[MOTOR_1],
  //--- FSM motor initial state -----------------------------------------------
  .motor_state      = MOTOR_STATE_INIT,
  //--- Error message ---------------------------------------------------------
  .motor_error.all  = MOTOR_ERROR_NO_ERROR,
  .clCycleNb        = 0,
  .itCnt            = 0,
  .itDone           = false,
  .motorChAReg_u.ptr= (volatile uint16_t *)(MOTOR1_PWM1_CMD_ADDR),
  .motorChBReg_u.ptr= (volatile uint16_t *)(MOTOR1_PWM2_CMD_ADDR),
  .motorChCReg_u.ptr= (volatile uint16_t *)(MOTOR1_PWM3_CMD_ADDR),
 },
 {
  //--- MOTOR_1 ---------------------------------------------------------------
  .motor_id         = MOTOR_2,
  .motorHalCfg_u    = {.ptr = &hal_motor_cfg[MOTOR_2]},
  .motorDRV_u.ptr   = &drv_cfg[MOTOR_2],
  .motorFOC_u.ptr   = &motor_foc[MOTOR_2],
  //--- FSM motor initial state -----------------------------------------------
  .motor_state      = MOTOR_STATE_INIT,
  //--- Error message ---------------------------------------------------------
  .motor_error.all  = MOTOR_ERROR_NO_ERROR,
  .clCycleNb        = 0,
  .itCnt            = 0,
  .itDone           = false,
  .motorChAReg_u.ptr= (volatile uint16_t *)(MOTOR2_PWM1_CMD_ADDR),
  .motorChBReg_u.ptr= (volatile uint16_t *)(MOTOR2_PWM2_CMD_ADDR),
  .motorChCReg_u.ptr= (volatile uint16_t *)(MOTOR2_PWM3_CMD_ADDR),
 },
#else
 {
  //--- MOTOR_1 ---------------------------------------------------------------
  .motor_id         = MOTOR_1,
  .p_motorHalCfg    = &hal_motor_cfg[MOTOR_1],
  .p_motorDRV       = &drv_cfg[MOTOR_1],
  .p_motorFOC       = &motor_foc[MOTOR_1],
  //--- FSM motor initial state -----------------------------------------------
  .motor_state      = MOTOR_STATE_INIT,
  //--- Error message ---------------------------------------------------------
  .motor_error.all  = MOTOR_ERROR_NO_ERROR,
  .clCycleNb        = 0,
  .itCnt            = 0,
  .itDone           = false,
  .p_motorChAReg    = (volatile uint16_t *)(MOTOR1_PWM1_CMD_ADDR),
  .p_motorChBReg    = (volatile uint16_t *)(MOTOR1_PWM2_CMD_ADDR),
  .p_motorChCReg    = (volatile uint16_t *)(MOTOR1_PWM3_CMD_ADDR),
 },
 {
  //--- MOTOR_1 ---------------------------------------------------------------
  .motor_id         = MOTOR_2,
  .p_motorHalCfg    = &hal_motor_cfg[MOTOR_2],
  .p_motorDRV       = &drv_cfg[MOTOR_2],
  .p_motorFOC       = &motor_foc[MOTOR_2],
  //--- FSM motor initial state -----------------------------------------------
  .motor_state      = MOTOR_STATE_INIT,
  //--- Error message ---------------------------------------------------------
  .motor_error.all  = MOTOR_ERROR_NO_ERROR,
  .clCycleNb        = 0,
  .itCnt            = 0,
  .itDone           = false,
  .p_motorChAReg    = (volatile uint16_t *)(MOTOR2_PWM1_CMD_ADDR),
  .p_motorChBReg    = (volatile uint16_t *)(MOTOR2_PWM2_CMD_ADDR),
  .p_motorChCReg    = (volatile uint16_t *)(MOTOR2_PWM3_CMD_ADDR),
 },
#endif
};

static const dma_cfg_t  dmaMem2MemCfgList[] =
{
 // DMA base register initialization
 // DMA to duplicate message from uOmodri command struct to foc struct.
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
 // END OF ARRAY
 {.dmaChBase        = UINT32_MAX},
};

//#pragma DATA_SECTION(cpu1_to_cm_addr, "MSGRAM_CPU_TO_CM");
//static com_cpu1_to_cm_t cpu1_to_cm_addr[] =
//{
// //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
//// {.data_addr = IPC_CPU1_TO_CM_ADDR_TRANSLATE((uint32_t)&motor_foc[MOTOR_1].motor_cfg.Rs),         .data_size = 4, .data_pos = 0x1001},
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cfg.Rs,              .data_size = 4, .data_pos = 0x1001}, //.index = 000,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cfg.Ls,              .data_size = 4, .data_pos = 0x1002}, //.index = 001,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cfg.kv,              .data_size = 4, .data_pos = 0x1003}, //.index = 002,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cfg.ke,              .data_size = 4, .data_pos = 0x1004}, //.index = 003,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cfg.ki,              .data_size = 4, .data_pos = 0x1005}, //.index = 004,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cfg.polePairs,       .data_size = 4, .data_pos = 0x1006}, //.index = 005,
// //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.va,              .data_size = 4, .data_pos = 0x1011}, //.index = 006,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.vb,              .data_size = 4, .data_pos = 0x1012}, //.index = 007,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.vc,              .data_size = 4, .data_pos = 0x1013}, //.index = 008,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.vbus,            .data_size = 4, .data_pos = 0x1014}, //.index = 009,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.vExt,            .data_size = 4, .data_pos = 0x1015}, //.index = 010,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.ia,              .data_size = 4, .data_pos = 0x1016}, //.index = 011,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.ib,              .data_size = 4, .data_pos = 0x1017}, //.index = 012,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_acq.ic,              .data_size = 4, .data_pos = 0x1018}, //.index = 013,
// //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.thetaElec,       .data_size = 4, .data_pos = 0x1021}, //.index = 014,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.thetaMech[NEW],  .data_size = 4, .data_pos = 0x1022}, //.index = 015,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.thetaMech[OLD],  .data_size = 4, .data_pos = 0x1023}, //.index = 016,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.thetaIndex,      .data_size = 4, .data_pos = 0x1024}, //.index = 017,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.thetaMechIndexed,.data_size = 4, .data_pos = 0x1025}, //.index = 018,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.thetaAbsolute,   .data_size = 4, .data_pos = 0x1026}, //.index = 019,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.turnNb,          .data_size = 2, .data_pos = 0x1027}, //.index = 020,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.thetaDir,        .data_size = 2, .data_pos = 0x1028}, //.index = 021,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.indexDetected,   .data_size = 2, .data_pos = 0x1029}, //.index = 022,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.indexCompensated,.data_size = 2, .data_pos = 0x102A}, //.index = 023,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.indexToggle,     .data_size = 2, .data_pos = 0x102B}, //.index = 024,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.rollOverFlag,    .data_size = 2, .data_pos = 0x102C}, //.index = 025,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.speed.speedHigh, .data_size = 4, .data_pos = 0x1031}, //.index = 026,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.speed.speedLow,  .data_size = 4, .data_pos = 0x1032}, //.index = 027,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.speed.speedMech, .data_size = 4, .data_pos = 0x1033}, //.index = 028,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.speed.speedElec, .data_size = 4, .data_pos = 0x1034}, //.index = 029,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.speed.theta[NEW],.data_size = 4, .data_pos = 0x1035}, //.index = 030,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_enc.speed.theta[OLD],.data_size = 4, .data_pos = 0x1036}, //.index = 031,
// //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.posRef,          .data_size = 4, .data_pos = 0x1041}, //.index = 032,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.velRef,          .data_size = 4, .data_pos = 0x1042}, //.index = 033,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.iqRef,           .data_size = 4, .data_pos = 0x1043}, //.index = 034,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.kpCoeff,         .data_size = 4, .data_pos = 0x1044}, //.index = 035,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.kdCoeff,         .data_size = 4, .data_pos = 0x1045}, //.index = 036,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.iSat,            .data_size = 4, .data_pos = 0x1046}, //.index = 037,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.timeoutRef,      .data_size = 2, .data_pos = 0x1047}, //.index = 038,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.cptTimeout,      .data_size = 2, .data_pos = 0x1048}, //.index = 039,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].motor_cmd.enableReg.all,   .data_size = 2, .data_pos = 0x1049}, //.index = 040,
// //--- FSM motor initial state -----------------------------------------------
// {.data_addr = (uint32_t)&motor[MOTOR_1].motor_state,                   .data_size = 2, .data_pos = 0x1051}, //.index = 041,
// //--- Error message ---------------------------------------------------------
// {.data_addr = (uint32_t)&motor[MOTOR_1].motor_error,                   .data_size = 2, .data_pos = 0x1061}, //.index = 042,
// //--- FOC - Sine / Cosine computation of angular position -------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].sinTheta,                  .data_size = 4, .data_pos = 0x1071}, //.index = 043,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].cosTheta,                  .data_size = 4, .data_pos = 0x1072}, //.index = 044,
////    //--- FOC - Clarke transform results ----------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].ialpha,                    .data_size = 4, .data_pos = 0x1081}, //.index = 045,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].ibeta,                     .data_size = 4, .data_pos = 0x1082}, //.index = 046,
// //--- FOC - Park transform results ------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].id,                        .data_size = 4, .data_pos = 0x1091}, //.index = 047,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].iq,                        .data_size = 4, .data_pos = 0x1092}, //.index = 048,
// //--- FOC - Inverse Park transform results ----------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].ud,                        .data_size = 4, .data_pos = 0x10A1}, //.index = 049,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].uq,                        .data_size = 4, .data_pos = 0x10A2}, //.index = 050,
// //--- FOC - Inverse Clarke transform results --------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].ualpha,                    .data_size = 4, .data_pos = 0x10B1}, //.index = 051,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].ubeta,                     .data_size = 4, .data_pos = 0x10B2}, //.index = 052,
// //--- FOC - Phase commands --------------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].ua,                        .data_size = 4, .data_pos = 0x10C1}, //.index = 053,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].ub,                        .data_size = 4, .data_pos = 0x10C2}, //.index = 054,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].uc,                        .data_size = 4, .data_pos = 0x10C3}, //.index = 055,
// //--- FOC - Max current and voltage -----------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].vmax,                      .data_size = 4, .data_pos = 0x10D1}, //.index = 056,
//// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].imax,                      .data_size = 4, .data_pos = 0x10D2},
// //--- FOC - Initialize duty cycle saturation --------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].dtcMax,                    .data_size = 4, .data_pos = 0x10E1}, //.index = 057,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].dtcMin,                    .data_size = 4, .data_pos = 0x10E2}, //.index = 058,
// //--- FOC - Duty cycles -----------------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].dtc_u,                     .data_size = 4, .data_pos = 0x10F1}, //.index = 059,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].dtc_v,                     .data_size = 4, .data_pos = 0x10F2}, //.index = 060,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].dtc_w,                     .data_size = 4, .data_pos = 0x10F3}, //.index = 061,
// //--- FOC - PI setpoint values ----------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].idRef,                     .data_size = 4, .data_pos = 0x1101}, //.index = 062,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].iqRef,                     .data_size = 4, .data_pos = 0x1102}, //.index = 063,
// //--- FOC - Initialize PI controller (Id) -----------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piId.err,                  .data_size = 4, .data_pos = 0x1111}, //.index = 064,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piId.kp,                   .data_size = 4, .data_pos = 0x1112}, //.index = 065,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piId.ki,                   .data_size = 4, .data_pos = 0x1113}, //.index = 066,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piId.integral,             .data_size = 4, .data_pos = 0x1114}, //.index = 067,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piId.ff,                   .data_size = 4, .data_pos = 0x1115}, //.index = 068,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piId.out,                  .data_size = 4, .data_pos = 0x1116}, //.index = 069,
// //--- FOC - Initialize PI controller (Iq) -----------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piIq.err,                  .data_size = 4, .data_pos = 0x1121}, //.index = 070,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piIq.kp,                   .data_size = 4, .data_pos = 0x1122}, //.index = 071,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piIq.ki,                   .data_size = 4, .data_pos = 0x1123}, //.index = 072,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piIq.integral,             .data_size = 4, .data_pos = 0x1124}, //.index = 073,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piIq.ff,                   .data_size = 4, .data_pos = 0x1125}, //.index = 074,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].piIq.out,                  .data_size = 4, .data_pos = 0x1126}, //.index = 075,
// //--- FOC - Initialize PD controller (Speed & position) ---------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].pdPos.errTheta,            .data_size = 4, .data_pos = 0x1131}, //.index = 076,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].pdPos.errSpeed,            .data_size = 4, .data_pos = 0x1132}, //.index = 077,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].pdPos.derivative,          .data_size = 4, .data_pos = 0x1133}, //.index = 078,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_1].pdPos.out,                 .data_size = 4, .data_pos = 0x1134}, //.index = 079,
//
// //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cfg.Rs,              .data_size = 4, .data_pos = 0x2001}, //.index = 080,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cfg.Ls,              .data_size = 4, .data_pos = 0x2002}, //.index = 081,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cfg.kv,              .data_size = 4, .data_pos = 0x2003}, //.index = 082,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cfg.ke,              .data_size = 4, .data_pos = 0x2004}, //.index = 083,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cfg.ki,              .data_size = 4, .data_pos = 0x2005}, //.index = 084,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cfg.polePairs,       .data_size = 4, .data_pos = 0x2006}, //.index = 085,
// //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.va,              .data_size = 4, .data_pos = 0x2011}, //.index = 086,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.vb,              .data_size = 4, .data_pos = 0x2012}, //.index = 087,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.vc,              .data_size = 4, .data_pos = 0x2013}, //.index = 088,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.vbus,            .data_size = 4, .data_pos = 0x2014}, //.index = 089,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.vExt,            .data_size = 4, .data_pos = 0x2015}, //.index = 090,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.ia,              .data_size = 4, .data_pos = 0x2016}, //.index = 091,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.ib,              .data_size = 4, .data_pos = 0x2017}, //.index = 092,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_acq.ic,              .data_size = 4, .data_pos = 0x2018}, //.index = 093,
// //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.thetaElec,       .data_size = 4, .data_pos = 0x2021}, //.index = 094,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.thetaMech[NEW],  .data_size = 4, .data_pos = 0x2022}, //.index = 095,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.thetaMech[OLD],  .data_size = 4, .data_pos = 0x2023}, //.index = 096,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.thetaIndex,      .data_size = 4, .data_pos = 0x2024}, //.index = 097,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.thetaMechIndexed,.data_size = 4, .data_pos = 0x2025}, //.index = 098,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.thetaAbsolute,   .data_size = 4, .data_pos = 0x2026}, //.index = 099,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.turnNb,          .data_size = 2, .data_pos = 0x2027}, //.index = 100,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.thetaDir,        .data_size = 2, .data_pos = 0x2028}, //.index = 101,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.indexDetected,   .data_size = 2, .data_pos = 0x2029}, //.index = 102,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.indexCompensated,.data_size = 2, .data_pos = 0x202A}, //.index = 103,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.indexToggle,     .data_size = 2, .data_pos = 0x202B}, //.index = 104,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.rollOverFlag,    .data_size = 2, .data_pos = 0x202C}, //.index = 105,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.speed.speedHigh, .data_size = 4, .data_pos = 0x2031}, //.index = 106,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.speed.speedLow,  .data_size = 4, .data_pos = 0x2032}, //.index = 107,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.speed.speedMech, .data_size = 4, .data_pos = 0x2033}, //.index = 108,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.speed.speedElec, .data_size = 4, .data_pos = 0x2034}, //.index = 109,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.speed.theta[NEW],.data_size = 4, .data_pos = 0x2035}, //.index = 110,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_enc.speed.theta[OLD],.data_size = 4, .data_pos = 0x2036}, //.index = 111,
// //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.posRef,          .data_size = 4, .data_pos = 0x2041}, //.index = 112,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.velRef,          .data_size = 4, .data_pos = 0x2042}, //.index = 113,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.iqRef,           .data_size = 4, .data_pos = 0x2043}, //.index = 114,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.kpCoeff,         .data_size = 4, .data_pos = 0x2044}, //.index = 115,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.kdCoeff,         .data_size = 4, .data_pos = 0x2045}, //.index = 116,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.iSat,            .data_size = 4, .data_pos = 0x2046}, //.index = 117,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.timeoutRef,      .data_size = 2, .data_pos = 0x2047}, //.index = 118,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.cptTimeout,      .data_size = 2, .data_pos = 0x2048}, //.index = 119,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].motor_cmd.enableReg.all,   .data_size = 2, .data_pos = 0x2049}, //.index = 120,
// //--- FSM motor initial state -----------------------------------------------
// {.data_addr = (uint32_t)&motor[MOTOR_2].motor_state,                   .data_size = 2, .data_pos = 0x2051}, //.index = 121,
// //--- Error message ---------------------------------------------------------
// {.data_addr = (uint32_t)&motor[MOTOR_2].motor_error,                   .data_size = 2, .data_pos = 0x2061}, //.index = 122,
// //--- FOC - Sine / Cosine computation of angular position -------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].sinTheta,                  .data_size = 4, .data_pos = 0x2071}, //.index = 123,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].cosTheta,                  .data_size = 4, .data_pos = 0x2022}, //.index = 124,
// //--- FOC - Clarke transform results ----------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].ialpha,                    .data_size = 4, .data_pos = 0x2081}, //.index = 125,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].ibeta,                     .data_size = 4, .data_pos = 0x2082}, //.index = 126,
// //--- FOC - Park transform results ------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].id,                        .data_size = 4, .data_pos = 0x2091}, //.index = 127,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].iq,                        .data_size = 4, .data_pos = 0x2092}, //.index = 128,
// //--- FOC - Inverse Park transform results ----------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].ud,                        .data_size = 4, .data_pos = 0x20A1}, //.index = 129,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].uq,                        .data_size = 4, .data_pos = 0x20A2}, //.index = 130,
// //--- FOC - Inverse Clarke transform results --------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].ualpha,                    .data_size = 4, .data_pos = 0x20B1}, //.index = 131,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].ubeta,                     .data_size = 4, .data_pos = 0x20B2}, //.index = 132,
// //--- FOC - Phase commands --------------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].ua,                        .data_size = 4, .data_pos = 0x20C1}, //.index = 133,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].ub,                        .data_size = 4, .data_pos = 0x20C2}, //.index = 134,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].uc,                        .data_size = 4, .data_pos = 0x20C3}, //.index = 135,
// //--- FOC - Max current and voltage -----------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].vmax,                      .data_size = 4, .data_pos = 0x20D1}, //.index = 136,
//// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].imax,                      .data_size = 4, .data_pos = 0x20D2},
// //--- FOC - Initialize duty cycle saturation --------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].dtcMax,                    .data_size = 4, .data_pos = 0x20E1}, //.index = 137,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].dtcMin,                    .data_size = 4, .data_pos = 0x20E2}, //.index = 138,
// //--- FOC - Duty cycles -----------------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].dtc_u,                     .data_size = 4, .data_pos = 0x20F1}, //.index = 139,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].dtc_v,                     .data_size = 4, .data_pos = 0x20F2}, //.index = 140,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].dtc_w,                     .data_size = 4, .data_pos = 0x20F3}, //.index = 141,
// //--- FOC - PI setpoint values ----------------------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].idRef,                     .data_size = 4, .data_pos = 0x2101}, //.index = 142,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].iqRef,                     .data_size = 4, .data_pos = 0x2102}, //.index = 143,
// //--- FOC - Initialize PI controller (Id) -----------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piId.err,                  .data_size = 4, .data_pos = 0x2111}, //.index = 144,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piId.kp,                   .data_size = 4, .data_pos = 0x2112}, //.index = 145,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piId.ki,                   .data_size = 4, .data_pos = 0x2113}, //.index = 146,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piId.integral,             .data_size = 4, .data_pos = 0x2114}, //.index = 147,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piId.ff,                   .data_size = 4, .data_pos = 0x2115}, //.index = 148,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piId.out,                  .data_size = 4, .data_pos = 0x2116}, //.index = 149,
// //--- FOC - Initialize PI controller (Iq) -----------------------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piIq.err,                  .data_size = 4, .data_pos = 0x2121}, //.index = 150,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piIq.kp,                   .data_size = 4, .data_pos = 0x2122}, //.index = 151,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piIq.ki,                   .data_size = 4, .data_pos = 0x2123}, //.index = 152,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piIq.integral,             .data_size = 4, .data_pos = 0x2124}, //.index = 153,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piIq.ff,                   .data_size = 4, .data_pos = 0x2125}, //.index = 154,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].piIq.out,                  .data_size = 4, .data_pos = 0x2126}, //.index = 155,
// //--- FOC - Initialize PD controller (Speed & position) ---------------------
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].pdPos.errTheta,            .data_size = 4, .data_pos = 0x2131}, //.index = 156,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].pdPos.errSpeed,            .data_size = 4, .data_pos = 0x2132}, //.index = 157,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].pdPos.derivative,          .data_size = 4, .data_pos = 0x2133}, //.index = 158,
// {.data_addr = (uint32_t)&motor_foc[MOTOR_2].pdPos.out,                 .data_size = 4, .data_pos = 0x2134}, //.index = 159,
//
//// {.index = 180, .data_addr = (uint32_t)UINT32_MAX,                                      .data_size = 0, .data_pos = 0x0000{
//};

#endif  /*__UOMODRI_CONFIG_HANDLERS_H__*/
