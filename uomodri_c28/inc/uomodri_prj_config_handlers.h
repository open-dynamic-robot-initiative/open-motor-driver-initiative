#ifndef __UOMODRI_PRJ_CONFIG_HANDLERS_H__
#define __UOMODRI_PRJ_CONFIG_HANDLERS_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "f2838x_device.h"
#include "f2838x_examples.h"
#include "driverlib.h"
#include "interrupt.h"
#include "device.h"

#include "hal.h"
#include "drv8353.h"
#include "foc.h"
#include "motor.h"
#include "communication.h"
#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"


__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);
__attribute__((interrupt)) void gpio_isrFaultMotor1(void);
__attribute__((interrupt)) void gpio_isrFaultMotor2(void);

/** @var    int_cfg_t   InterruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t InterruptList[] =
{
 {
  .intNum           = MOTOR1_IA_INT_CH,
  .intAckGroup      = MOTOR1_IA_INT_ACK_GROUP,
  .p_intHandler     = &adc_isrMotor1,
 }, //
 {
  .intNum           = MOTOR2_IA_INT_CH,
  .intAckGroup      = MOTOR2_IA_INT_ACK_GROUP,
  .p_intHandler     = &adc_isrMotor2,
 },
 {
  .intNum           = MOTOR1_DRV_GPIO_NFAULT_INT,
  .intAckGroup      = MOTOR1_DRV_GPIO_NFAULT_INT_ACK_GROUP,
  .p_intHandler     = &gpio_isrFaultMotor1,
 },
 {
  .intNum           = MOTOR2_DRV_GPIO_NFAULT_INT,
  .intAckGroup      = MOTOR2_DRV_GPIO_NFAULT_INT_ACK_GROUP,
  .p_intHandler     = &gpio_isrFaultMotor2,
 },
// {
//  .intNum           = CPU_TIMER_0_INT,
//  .intAckGroup      = CPU_TIMER_0_INT_ACK_GROUP,
//  .p_intHandler     = &cpu_isrTimer0,
// },
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
 {
  .intNum           = INT_CLA1_1,
  .intAckGroup      = INTERRUPT_ACK_GROUP11,
  .handler          = &cla1_isr1
 },
#endif
 // END OF ARRAY
 {.intNum           = 0xFFFFFFFF},
};

/** @var    DRV_Cfg     DrvCfgList[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
static const drv_cfg_t  DrvCfgList[] =
{
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = MOTOR1_DRV_SPI_CS, .gpioNumber_EN = MOTOR1_DRV_GPIO_EN},
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = MOTOR2_DRV_SPI_CS, .gpioNumber_EN = MOTOR2_DRV_GPIO_EN},
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
  .ctrl_reg_02.bit.OTW_REP      = true,                 // Over Temperature Warning will raise nFault
  .ctrl_reg_02.bit.DIS_GDF      = false,
  .ctrl_reg_02.bit.DIS_GDUV     = false,
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (UOMODRI_USE_REV_1_0_CONFIG)
  .ctrl_reg_02.bit.OCP_ACT      = true,
#endif
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.bit.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.bit.IDRIVEP_HS   = DRV_ISOUR_HS_0P300_A,
  .ctrl_reg_03.bit.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.bit.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.bit.IDRIVEP_LS   = DRV_ISOUR_LS_0P300_A,
  .ctrl_reg_04.bit.TDRIVE       = DRV_TSOUR_4000_NS,
  .ctrl_reg_04.bit.CBC          = false,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.bit.VDS_LVL      = DRV_VDS_LVL_0P700_V,
  .ctrl_reg_05.bit.OCP_DEG      = DRV_OCPDEG_8_US,
  .ctrl_reg_05.bit.OCP_MODE     = DRV_AUTOMATIC_RETRY,
  .ctrl_reg_05.bit.DEAD_TIME    = DRV_DEADTIME_400_NS,
  .ctrl_reg_05.bit.TRETRY       = false,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.bit.SEN_LVL      = DRV_SENLVL_1P00_V,
  .ctrl_reg_06.bit.CSA_CAL_C    = false,
  .ctrl_reg_06.bit.CSA_CAL_B    = false,
  .ctrl_reg_06.bit.CSA_CAL_A    = false,
  .ctrl_reg_06.bit.DIS_SEN      = false,
  .ctrl_reg_06.bit.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.bit.LS_REF       = false,
  .ctrl_reg_06.bit.VREF_DIV     = true,
  .ctrl_reg_06.bit.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (UOMODRI_USE_REV_1_0_CONFIG)
  .ctrl_reg_07.bit.CAL_MODE     = false,
#endif
 },
 {
  .ctrl_reg_02.bit.CLR_FLT      = false,
  .ctrl_reg_02.bit.BRAKE        = false,
  .ctrl_reg_02.bit.COAST        = false,
  .ctrl_reg_02.bit.PWM1_DIR     = false,
  .ctrl_reg_02.bit.PWM1_COM     = false,
  .ctrl_reg_02.bit.PWM_MODE     = DRV_PWMMODE_6,
  .ctrl_reg_02.bit.OTW_REP      = true,                 // Over Temperature Warning will raise nFault
  .ctrl_reg_02.bit.DIS_GDF      = false,
  .ctrl_reg_02.bit.DIS_GDUV     = false,
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (UOMODRI_USE_REV_1_0_CONFIG)
  .ctrl_reg_02.bit.OCP_ACT      = true,
#endif
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.bit.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.bit.IDRIVEP_HS   = DRV_ISOUR_HS_0P300_A,
  .ctrl_reg_03.bit.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.bit.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.bit.IDRIVEP_LS   = DRV_ISOUR_LS_0P300_A,
  .ctrl_reg_04.bit.TDRIVE       = DRV_TSOUR_4000_NS,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.bit.VDS_LVL      = DRV_VDS_LVL_0P700_V,
  .ctrl_reg_05.bit.OCP_DEG      = DRV_OCPDEG_8_US,
  .ctrl_reg_05.bit.OCP_MODE     = DRV_AUTOMATIC_RETRY,
  .ctrl_reg_05.bit.DEAD_TIME    = DRV_DEADTIME_400_NS,
  .ctrl_reg_05.bit.TRETRY       = false,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.bit.SEN_LVL      = DRV_SENLVL_1P00_V,
  .ctrl_reg_06.bit.CSA_CAL_C    = false,
  .ctrl_reg_06.bit.CSA_CAL_B    = false,
  .ctrl_reg_06.bit.CSA_CAL_A    = false,
  .ctrl_reg_06.bit.DIS_SEN      = false,
  .ctrl_reg_06.bit.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.bit.LS_REF       = false,
  .ctrl_reg_06.bit.VREF_DIV     = true,
  .ctrl_reg_06.bit.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (UOMODRI_USE_REV_1_0_CONFIG)
  .ctrl_reg_07.bit.CAL_MODE     = false,
#endif
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
//  .enableTimeOut    = true,
//  .rxTimeOut        = true,
 },
 {
  .p_drvCfgHandler  = &DrvCfgList[MOTOR_2],
  .p_drvRegHandler  = &DrvRegList[MOTOR_2],
//  .enableTimeOut    = true,
//  .rxTimeOut        = true,
 },
};

static const hal_motor_cfg_t hal_motor_cfg[] =
{
 {
  .p_pwmCntCmp[0]   = &PwmCounterCompareCfgList[0],
  .p_pwmCntCmp[1]   = &PwmCounterCompareCfgList[1],
  .p_pwmCntCmp[2]   = &PwmCounterCompareCfgList[2],
  .p_vAcq[0]        = &AdcAcqList[0],
  .p_vAcq[1]        = &AdcAcqList[4],
  .p_vAcq[2]        = &AdcAcqList[8],
  .p_iAcq[0]        = &AdcAcqList[2],
  .p_iAcq[1]        = &AdcAcqList[6],
  .p_iAcq[2]        = &AdcAcqList[10],
  //.p_vbusAcq    = &AdcAcqList[8],
  .p_intAcq         = &AdcIntList[0],
 },
 {
  .p_pwmCntCmp[0]   = &PwmCounterCompareCfgList[3],
  .p_pwmCntCmp[1]   = &PwmCounterCompareCfgList[4],
  .p_pwmCntCmp[2]   = &PwmCounterCompareCfgList[5],
  .p_vAcq[0]        = &AdcAcqList[1],
  .p_vAcq[1]        = &AdcAcqList[5],
  .p_vAcq[2]        = &AdcAcqList[9],
  .p_iAcq[0]        = &AdcAcqList[3],
  .p_iAcq[1]        = &AdcAcqList[7],
  .p_iAcq[2]        = &AdcAcqList[11],
  //.p_vbusAcq    = &AdcAcqList[8],
  .p_intAcq         = &AdcIntList[1],
 },
};

#pragma DATA_SECTION(cmd_uOmodri, "ramgs0");
static cmd_uomodri_t cmd_uOmodri =
{
 .mode.bit.TIMEOUT  = 0,
 .mode.bit.EI2OC    = true,
 .mode.bit.EI1OC    = true,
 .mode.bit.EPRE     = true,
 .mode.bit.EM2      = false,
 .mode.bit.EM1      = false,
 .mode.bit.ES       = false,
 .position[MOTOR_1] = 0.0f,
 .position[MOTOR_2] = 0.0f,
 .velocity[MOTOR_1] = 0.0f,
 .velocity[MOTOR_2] = 0.0f,
 .current[MOTOR_1]  = 0.0f,
 .current[MOTOR_2]  = 0.0f,
 .kpCoeff[MOTOR_1]  = 0.0f,
 .kpCoeff[MOTOR_2]  = 0.0f,
 .kdCoeff[MOTOR_1]  = 0.0f,
 .kdCoeff[MOTOR_2]  = 0.0f,
 .iSat[MOTOR_1]     = 1.0f,
 .iSat[MOTOR_2]     = 1.0f,
 .index             = 0
};

#pragma DATA_SECTION(motor_foc, "MSGRAM_CPU_TO_CM");
static foc_t motor_foc[] =
{
 {
  //--- MOTOR_1 ---------------------------------------------------------------
  .motor_id                             = MOTOR_1,
  //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
  .motor_cfg.Rs                         = (float32_t)MOTOR1_RS,
  .motor_cfg.Ld                         = (float32_t)MOTOR1_LD,
  .motor_cfg.Lq                         = (float32_t)MOTOR1_LQ,
  .motor_cfg.kv                         = (float32_t)MOTOR1_KV,
  .motor_cfg.ke                         = (float32_t)MOTOR1_KE,
  .motor_cfg.ki                         = (float32_t)MOTOR1_KI,
  .motor_cfg.polePairs                  = (float32_t)MOTOR1_POLES_PAIRS,
  //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
  .motor_acq.p_adcRegs[0]               = MOTOR12_IVA_ADC_REGS,
  .motor_acq.p_adcRegs[1]               = MOTOR12_IVB_ADC_REGS,
  .motor_acq.p_adcRegs[2]               = MOTOR12_IVC_ADC_REGS,
  .motor_acq.p_adcRegs[3]               = VBUS_ADC_REGS,
  .motor_acq.p_adcResultRegs[0]         = MOTOR12_IVA_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[1]         = MOTOR12_IVB_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[2]         = MOTOR12_IVC_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[3]         = VBUS_ADC_RESULT_REGS,
  .motor_acq.p_vaMeasReg                = (volatile uint16_t *)(MOTOR1_VA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vbMeasReg                = (volatile uint16_t *)(MOTOR1_VB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vcMeasReg                = (volatile uint16_t *)(MOTOR1_VC_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vExtMeasReg              = (volatile uint16_t *)(EXTERN_V1_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg              = (volatile uint16_t *)(VBUS_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iaMeasReg                = (volatile int16_t *)(MOTOR1_IA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_ibMeasReg                = (volatile int16_t *)(MOTOR1_IB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_icMeasReg                = (volatile int16_t *)(MOTOR1_IC_ADC_PPB_RESULT_ADDR),
  .motor_acq.va                         = 0.0,
  .motor_acq.vb                         = 0.0,
  .motor_acq.vc                         = 0.0,
  .motor_acq.vbus                       = 0.0,
  .motor_acq.vExt                       = 0.0,
  .motor_acq.ia                         = 0.0,
  .motor_acq.ib                         = 0.0,
  .motor_acq.ic                         = 0.0,
//  .motor_acq.measNew                    = false,
  .motor_acq.vBusFlt.a                  = VBUS_LPF_ALPHA,
  .motor_acq.vBusFlt.one_minus_a        = VBUS_LPF_ONE_M_ALPHA,
  .motor_acq.vExtFlt.a                  = VEXT_LPF_ALPHA,
  .motor_acq.vExtFlt.one_minus_a        = VEXT_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.p_qepHandle                = (&QepCfgList[MOTOR_1]),
  .motor_enc.p_qepRegs                  = MOTOR1_QEP_REGS,
  .motor_enc.polePairs                  = MOTOR1_POLES_PAIRS,
  .motor_enc.thetaElec                  = 0.0,
  .motor_enc.thetaMech[NEW]             = 0.0,
  .motor_enc.thetaMech[OLD]             = 0.0,
  .motor_enc.thetaMechScaler            = (float32_t)MOTOR1_ENC_RESOLUTION_SCALE,
  .motor_enc.thetaIndex                 = 0.0,
  .motor_enc.thetaMechIndexed           = 0.0,
  .motor_enc.thetaAbsolute              = 0.0,
  .motor_enc.turnNb                     = 0,
  .motor_enc.thetaDir                   = 0,
  .motor_enc.indexDetected              = false,
  .motor_enc.indexCompensated           = false,
  .motor_enc.indexToggle                = false,
  .motor_enc.rollOverFlag               = false,
//  .motor_enc.rollOverFlagEn             = false,
  .motor_enc.speed.speedHighScaler      = (float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speed.speedHigh            = 0.0,
  .motor_enc.speed.speedLowScaler       = (float32_t)MOTOR1_ENC_SPEED_LOW_SCALE,
  .motor_enc.speed.speedLow             = 0.0,
//  .motor_enc.speed.speedRef             = 0.0,
  .motor_enc.speed.speedMech            = 0.0,
  .motor_enc.speed.speedElec            = 0.0,
  .motor_enc.speed.speedFlt.a           = (float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speed.speedFlt.one_minus_a = (float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA,
  //---------------------------------------------------------------------------
  .motor_align.state                    = ALIGN_COUNT_UP,
  .motor_align.cnt                      = 0,
  .motor_align.cntMax                   = PWM_FREQ,
  .motor_align.idAlignRef               = 0.5,// 500mA alignment current
  .motor_align.idAlign                  = 0.0,
  //---------------------------------------------------------------------------
  .motor_state[NEW]                     = MOTOR_STOP,
  .motor_state[OLD]                     = MOTOR_STOP,
  //---------------------------------------------------------------------------
  .iParkFlt.a                           = MOTOR1_CURRENT_LPF_ALPHA,
  .iParkFlt.one_minus_a                 = MOTOR1_CURRENT_LPF_ONE_M_ALPHA,
  //--- FOC - Sine / Cosine computation of angular position -------------------
  .sinTheta                             = 0.0,
  .cosTheta                             = 0.0,
  //--- FOC - Clarke transform results ----------------------------------------
  .ialpha                               = 0.0,
  .ibeta                                = 0.0,
  //--- FOC - Park transform results ------------------------------------------
  .id                                   = 0.0,
  .iq                                   = 0.0,
  //--- FOC - Inverse Park transform results ----------------------------------
  .ud                                   = 0.0,
  .uq                                   = 0.0,
  //--- FOC - Inverse Clarke transform results --------------------------------
  .ualpha                               = 0.0,
  .ubeta                                = 0.0,
  //--- FOC - Phase commands --------------------------------------------------
  .ua                                   = 0.0,
  .ub                                   = 0.0,
  .uc                                   = 0.0,
  //--- FOC - Max current and voltage -----------------------------------------
  .vmax                                 = 0.0,
  .imax                                 = MOTOR12_IMAX,
  //--- FOC - Initialize duty cycle saturation --------------------------------
  .dtcMax                               = MOTOR1_DTC_MAX,
  .dtcMin                               = MOTOR1_DTC_MIN,
  //--- FOC - Duty cycles -----------------------------------------------------
  .dtc_u                                = 0.0,
  .dtc_v                                = 0.0,
  .dtc_w                                = 0.0,
  //--- FOC - PI setpoint values ----------------------------------------------
  .idRef                                = 0.0,
  .iqRef                                = 0.0,
  //--- FOC - Initialize PI controller (Id) -----------------------------------
  .piId.p_fb                            = (float32_t *)(&motor_foc[MOTOR_1].id),
  .piId.p_set                           = (float32_t *)(&motor_foc[MOTOR_1].idRef),
  .piId.p_sat                           = (float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus),
  .piId.err                             = 0.0,
  .piId.kp                              = (float32_t)MOTOR1_ID_REGULATOR_KP_COEF,
  .piId.ki                              = (float32_t)MOTOR1_ID_REGULATOR_KI_COEF,
  .piId.integral                        = 0.0,
  .piId.ff                              = 0.0,
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.p_fb                            = (float32_t *)(&motor_foc[MOTOR_1].iq),
  .piIq.p_set                           = (float32_t *)(&motor_foc[MOTOR_1].iqRef),
  .piIq.p_sat                           = (float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus),
  .piIq.err                             = 0.0,
  .piIq.kp                              = (float32_t)MOTOR1_IQ_REGULATOR_KP_COEF,
  .piIq.ki                              = (float32_t)MOTOR1_IQ_REGULATOR_KI_COEF,
  .piIq.integral                        = 0.0,
  .piIq.ff                              = 0.0,
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPos.p_fbTheta                      = (float32_t *)(&motor_foc[MOTOR_1].motor_enc.thetaAbsolute),
  .pdPos.p_setTheta                     = (float32_t *)(&cmd_uOmodri.position[MOTOR_1]),
  .pdPos.errTheta                       = 0.0,
  .pdPos.p_fbSpeed                      = (float32_t *)(&motor_foc[MOTOR_1].motor_enc.speed.speedMech),
  .pdPos.p_setSpeed                     = (float32_t *)(&cmd_uOmodri.velocity[MOTOR_1]),
  .pdPos.errSpeed                       = 0.0,
  .pdPos.p_kp                           = (float32_t *)(&cmd_uOmodri.kpCoeff[MOTOR_1]),//(float32_t)MOTOR1_SPEED_REGULATOR_KP_COEF,
  .pdPos.p_kd                           = (float32_t *)(&cmd_uOmodri.kdCoeff[MOTOR_1]),//(float32_t)MOTOR1_SPEED_REGULATOR_KD_COEF,

  .pdPos.derivative                     = 0.0,
  .pdPos.p_ff                           = (float32_t *)(&cmd_uOmodri.current[MOTOR_1]),
  .pdPos.p_sat                          = (float32_t *)(&cmd_uOmodri.iSat[MOTOR_1]),//MOTOR1_SPEED_REGULATOR_SATURATION,//((float32_t *)(&pdSat)),
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resEst                               = 1.0,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstFlt.a                          = 0.0,
  .resEstFlt.one_minus_a                = 0.0,
  //--- Error message ---------------------------------------------------------
  .errorMsg                             = FOC_ERROR_NO_ERROR,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
 },
 //---------------------------------------------------------------------------
 //---------------------------------------------------------------------------
 {
  //--- MOTOR_2 ---------------------------------------------------------------
  .motor_id                             = MOTOR_2,
  //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
  .motor_cfg.Rs                         = (float32_t)MOTOR2_RS,
  .motor_cfg.Ld                         = (float32_t)MOTOR2_LD,
  .motor_cfg.Lq                         = (float32_t)MOTOR2_LQ,
  .motor_cfg.kv                         = (float32_t)MOTOR2_KV,
  .motor_cfg.ke                         = (float32_t)MOTOR2_KE,
  .motor_cfg.ki                         = (float32_t)MOTOR2_KI,
  .motor_cfg.polePairs                  = (float32_t)MOTOR2_POLES_PAIRS,
  //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
  .motor_acq.p_adcRegs[0]               = MOTOR12_IVA_ADC_REGS,
  .motor_acq.p_adcRegs[1]               = MOTOR12_IVB_ADC_REGS,
  .motor_acq.p_adcRegs[2]               = MOTOR12_IVC_ADC_REGS,
  .motor_acq.p_adcRegs[3]               = VBUS_ADC_REGS,
  .motor_acq.p_adcResultRegs[0]         = MOTOR12_IVA_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[1]         = MOTOR12_IVB_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[2]         = MOTOR12_IVC_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[3]         = VBUS_ADC_RESULT_REGS,
  .motor_acq.p_vaMeasReg                = (volatile uint16_t *)(MOTOR2_VA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vbMeasReg                = (volatile uint16_t *)(MOTOR2_VB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vcMeasReg                = (volatile uint16_t *)(MOTOR2_VC_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vExtMeasReg              = (volatile uint16_t *)(EXTERN_V2_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg              = (volatile uint16_t *)(VBUS_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iaMeasReg                = (volatile int16_t *)(MOTOR2_IA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_ibMeasReg                = (volatile int16_t *)(MOTOR2_IB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_icMeasReg                = (volatile int16_t *)(MOTOR2_IC_ADC_PPB_RESULT_ADDR),
  .motor_acq.va                         = 0.0,
  .motor_acq.vb                         = 0.0,
  .motor_acq.vc                         = 0.0,
  .motor_acq.vbus                       = 0.0,
  .motor_acq.ia                         = 0.0,
  .motor_acq.ib                         = 0.0,
  .motor_acq.ic                         = 0.0,
//  .motor_acq.measNew                    = false,
  .motor_acq.vBusFlt.a                  = VBUS_LPF_ALPHA,
  .motor_acq.vBusFlt.one_minus_a        = VBUS_LPF_ONE_M_ALPHA,
  .motor_acq.vExtFlt.a                  = VEXT_LPF_ALPHA,
  .motor_acq.vExtFlt.one_minus_a        = VEXT_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.p_qepHandle                = (&QepCfgList[MOTOR_2]),//(const eQEP_TypeDef*)&QepCfgList[MOTOR_2],
  .motor_enc.p_qepRegs                  = MOTOR2_QEP_REGS,
  .motor_enc.polePairs                  = MOTOR2_POLES_PAIRS,
  .motor_enc.thetaElec                  = 0.0,
  .motor_enc.thetaMech[NEW]             = 0.0,
  .motor_enc.thetaMech[OLD]             = 0.0,
  .motor_enc.thetaMechScaler            = (float32_t)MOTOR2_ENC_RESOLUTION_SCALE,
  .motor_enc.thetaIndex                 = 0.0,
  .motor_enc.thetaMechIndexed           = 0.0,
  .motor_enc.thetaAbsolute              = 0.0,
  .motor_enc.turnNb                     = 0,
  .motor_enc.thetaDir                   = 0,
  .motor_enc.indexDetected              = false,
  .motor_enc.indexCompensated           = false,
  .motor_enc.indexToggle                = false,
  .motor_enc.rollOverFlag               = false,
//  .motor_enc.rollOverFlagEn             = false,
  .motor_enc.speed.speedHighScaler      = (float32_t)MOTOR2_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speed.speedHigh            = 0.0,
  .motor_enc.speed.speedLowScaler       = (float32_t)MOTOR2_ENC_SPEED_LOW_SCALE,
  .motor_enc.speed.speedLow             = 0.0,
//  .motor_enc.speed.speedRef             = 0.0,
  .motor_enc.speed.speedMech            = 0.0,
  .motor_enc.speed.speedElec            = 0.0,
  .motor_enc.speed.speedFlt.a           = (float32_t)MOTOR2_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speed.speedFlt.one_minus_a = (float32_t)MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA,
  //---------------------------------------------------------------------------
  .motor_align.state                    = ALIGN_COUNT_UP,
  .motor_align.cnt                      = 0,
  .motor_align.cntMax                   = PWM_FREQ,
  .motor_align.idAlignRef               = 0.5,// 500mA alignment current
  .motor_align.idAlign                  = 0.0,
  //---------------------------------------------------------------------------
  .motor_state[NEW]                     = MOTOR_STOP,
  .motor_state[OLD]                     = MOTOR_STOP,
  //---------------------------------------------------------------------------
  .iParkFlt.a                           = MOTOR2_CURRENT_LPF_ALPHA,
  .iParkFlt.one_minus_a                 = MOTOR2_CURRENT_LPF_ONE_M_ALPHA,
  //--- FOC - Sine / Cosine computation of angular position -------------------
  .sinTheta                             = 0.0,
  .cosTheta                             = 0.0,
  //--- FOC - Clarke transform results ----------------------------------------
  .ialpha                               = 0.0,
  .ibeta                                = 0.0,
  //--- FOC - Park transform results ------------------------------------------
  .id                                   = 0.0,
  .iq                                   = 0.0,
  //--- FOC - Inverse Park transform results ----------------------------------
  .ud                                   = 0.0,
  .uq                                   = 0.0,
  //--- FOC - Inverse Clarke transform results --------------------------------
  .ualpha                               = 0.0,
  .ubeta                                = 0.0,
  //--- FOC - Phase commands --------------------------------------------------
  .ua                                   = 0.0,
  .ub                                   = 0.0,
  .uc                                   = 0.0,
  //--- FOC - Max current and voltage -----------------------------------------
  .vmax                                 = 0.0,
  .imax                                 = MOTOR12_IMAX,
  //--- FOC - Initialize duty cycle saturation --------------------------------
  .dtcMax                               = MOTOR2_DTC_MAX,
  .dtcMin                               = MOTOR2_DTC_MIN,
  //--- FOC - Duty cycles -----------------------------------------------------
  .dtc_u                                = 0.0,
  .dtc_v                                = 0.0,
  .dtc_w                                = 0.0,
  //--- FOC - PI setpoint values ----------------------------------------------
  .idRef                                = 0.0,
  .iqRef                                = 0.0,
  //--- FOC - Initialize PI controller (Id) -----------------------------------
  .piId.p_fb                            = (float32_t *)(&motor_foc[MOTOR_2].id),
  .piId.p_set                           = (float32_t *)(&motor_foc[MOTOR_2].idRef),
  .piId.p_sat                           = (float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus),
  .piId.err                             = 0.0,
  .piId.kp                              = (float32_t)MOTOR2_ID_REGULATOR_KP_COEF,
  .piId.ki                              = (float32_t)MOTOR2_ID_REGULATOR_KI_COEF,
  .piId.integral                        = 0.0,
  .piId.ff                              = 0.0,
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.p_fb                            = (float32_t *)(&motor_foc[MOTOR_2].iq),
  .piIq.p_set                           = (float32_t *)(&motor_foc[MOTOR_2].iqRef),
  .piIq.p_sat                           = (float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus),
  .piIq.err                             = 0.0,
  .piIq.kp                              = (float32_t)MOTOR2_IQ_REGULATOR_KP_COEF,
  .piIq.ki                              = (float32_t)MOTOR2_IQ_REGULATOR_KI_COEF,
  .piIq.integral                        = 0.0,
  .piIq.ff                              = 0.0,
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPos.p_fbTheta                      = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.thetaAbsolute),
  .pdPos.p_setTheta                     = (float32_t *)(&cmd_uOmodri.position[MOTOR_2]),
  .pdPos.errTheta                       = 0.0,
  .pdPos.p_fbSpeed                      = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.speed.speedMech),
  .pdPos.p_setSpeed                     = (float32_t *)(&cmd_uOmodri.velocity[MOTOR_2]),
  .pdPos.errSpeed                       = 0.0,
  .pdPos.p_kp                           = (float32_t *)(&cmd_uOmodri.kpCoeff[MOTOR_2]),//(float32_t)MOTOR2_SPEED_REGULATOR_KP_COEF,
  .pdPos.p_kd                           = (float32_t *)(&cmd_uOmodri.kdCoeff[MOTOR_2]),//(float32_t)MOTOR2_SPEED_REGULATOR_KD_COEF,
  .pdPos.derivative                     = 0.0,
  .pdPos.p_ff                           = (float32_t *)(&cmd_uOmodri.current[MOTOR_2]),
  .pdPos.p_sat                          = (float32_t *)(&cmd_uOmodri.iSat[MOTOR_2]),//(float32_t)MOTOR2_SPEED_REGULATOR_SATURATION,
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resEst                               = 0.0,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstFlt.a                          = 0.0,
  .resEstFlt.one_minus_a                = 0.0,
  //--- Error message ---------------------------------------------------------
  .errorMsg                             = FOC_ERROR_NO_ERROR,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
 },
};

/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
static motor_t motor[] =
{
 {
//  .motorID          = MOTOR_1,
  .p_motorHalCfg                = &hal_motor_cfg[MOTOR_1],
  .p_motorDRV                   = &drv_cfg[MOTOR_1],
  .p_motorFOC                   = &motor_foc[MOTOR_1],
  .clCycleNb                    = 0,
  .itCnt                        = 0,
  .statorResEst                 = 1.0,
  .itDone                       = false,
  .statorResEstFlt.a            = MOTOR1_STATOR_RESISTOR_LPF_ALPHA,
  .statorResEstFlt.one_minus_a  = MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
//  .enabled          = false,
//  .currentScaler    = ADC_CURRENT_SCALE,
//  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
//  .voltageScaler    = ADC_VOLTAGE_SCALE,
//  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
//  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
//  .voltageScaler    = ADC_VOLTAGE_SCALE,
//  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
  .p_motorChAReg                = (volatile uint16_t *)(MOTOR1_PWM1_CMD_ADDR),
  .p_motorChBReg                = (volatile uint16_t *)(MOTOR1_PWM2_CMD_ADDR),
  .p_motorChCReg                = (volatile uint16_t *)(MOTOR1_PWM3_CMD_ADDR),
 },
 {
//  .motorID          = MOTOR_2,
  .p_motorHalCfg                = &hal_motor_cfg[MOTOR_2],
  .p_motorDRV                   = &drv_cfg[MOTOR_2],
  .p_motorFOC                   = &motor_foc[MOTOR_2],
  .clCycleNb                    = 0,
  .itCnt                        = 0,
  .statorResEst                 = 1.0,
  .itDone                       = false,
  .statorResEstFlt.a            = MOTOR2_STATOR_RESISTOR_LPF_ALPHA,
  .statorResEstFlt.one_minus_a  = MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
//  .enabled          = false,
//  .currentScaler    = ADC_CURRENT_SCALE,
//  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
//  .voltageScaler    = ADC_VOLTAGE_SCALE,
//  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
  .p_motorChAReg                = (volatile uint16_t *)(MOTOR2_PWM1_CMD_ADDR),
  .p_motorChBReg                = (volatile uint16_t *)(MOTOR2_PWM2_CMD_ADDR),
  .p_motorChCReg                = (volatile uint16_t *)(MOTOR2_PWM3_CMD_ADDR),
 },
};

#endif  /*__UOMODRI_CONFIG_HANDLERS_H__*/
