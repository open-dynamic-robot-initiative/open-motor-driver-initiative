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

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   InterruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t InterruptList[] =
{
// {
//  .intNum           = MOTOR1_IA_INT_CH,
//  .intAckGroup      = MOTOR1_IA_INT_ACK_GROUP,
//  .p_intHandler     = &adc_isrMotor1
// },
// {
//  .intNum           = MOTOR2_IA_INT_CH,
//  .intAckGroup      = MOTOR2_IA_INT_ACK_GROUP,
//  .p_intHandler     = &adc_isrMotor2
// },
 {
  .intNum           = INT_CLA1_3,
  .intAckGroup      = INTERRUPT_ACK_GROUP11,
  .p_intHandler     = &cla_isrMotor1
 },
 {
  .intNum           = INT_CLA1_4,
  .intAckGroup      = INTERRUPT_ACK_GROUP11,
  .p_intHandler     = &cla_isrMotor2
 },
 {
  .intNum           = INT_TIMER0,
  .intAckGroup      = INTERRUPT_ACK_GROUP1,
  .p_intHandler     = &tmr_isrLed
 },
// {
//  .intNum           = INT_EPWM1,
//  .intAckGroup      = INTERRUPT_ACK_GROUP3,
//  .p_intHandler     = NULL,
// },
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

#if (CLA_CORE_ENABLE)
//#pragma DATA_ALIGN(motor_foc_m1, 4)
//#pragma DATA_SECTION(motor_foc_m1, "ClaDataRam0");
//foc_t motor_foc_m1;
//#pragma DATA_ALIGN(motor_foc_m2, 4)
//#pragma DATA_SECTION(motor_foc_m2, "ClaDataRam1");
//foc_t motor_foc_m2;
#else
#pragma DATA_ALIGN(motor_foc, 4)
#pragma DATA_SECTION(motor_foc, "ramgs0");
static foc_t motor_foc[] =
{
 {
  //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
  .motor_cfg.Rs                             = MOTOR1_RS,
  .motor_cfg.Ls                             = MOTOR1_LS,
  .motor_cfg.kv                             = MOTOR1_KV,
  .motor_cfg.ke                             = MOTOR1_KE,
  .motor_cfg.ki                             = MOTOR1_KI,
  .motor_cfg.polePairs                      = MOTOR1_POLES_PAIRS,
  //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
  .motor_acq.p_vExtMeasReg                  = (volatile uint16_t *)(VEXT1_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg                  = (volatile uint16_t *)(VBUS_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iMeasReg[CHANNEL_1]          = (volatile int16_t *)(MOTOR1_IA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iMeasReg[CHANNEL_2]          = (volatile int16_t *)(MOTOR1_IB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iMeasReg[CHANNEL_3]          = (volatile int16_t *)(MOTOR1_IC_ADC_PPB_RESULT_ADDR),
  .motor_acq.vbus                           = 0.0f,
  .motor_acq.vExt                           = 0.0f,
  .motor_acq.ia                             = 0.0f,
  .motor_acq.ib                             = 0.0f,
  .motor_acq.ic                             = 0.0f,
  .motor_acq.vBusFlt.a                      = VBUS_LPF_ALPHA,
  .motor_acq.vBusFlt.one_minus_a            = VBUS_LPF_ONE_M_ALPHA,
  .motor_acq.vExtFlt.a                      = VEXT_LPF_ALPHA,
  .motor_acq.vExtFlt.one_minus_a            = VEXT_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.eqepBaseAddr                   = MOTOR1_QEP_BASE,
  .motor_enc.polePairs                      = MOTOR1_POLES_PAIRS,
  .motor_enc.thetaElec                      = 0.0f,
  .motor_enc.thetaMech[NEW]                 = 0.0f,
  .motor_enc.thetaMech[OLD]                 = 0.0f,
  .motor_enc.thetaMechScaler                = (float32_t)MOTOR1_ENC_RESOLUTION_SCALE,
  .motor_enc.thetaIndexRelative             = 0.0f,
  .motor_enc.thetaIndexAbsolute             = 0.0f,
  .motor_enc.thetaAbsolute                  = 0.0f,
  .motor_enc.turnNb                         = 0,
  .motor_enc.thetaDir                       = 0,
  .motor_enc.flags.all                      = 0,
  .motor_enc.speed.speedHighScaler          = (float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speed.speedHigh                = 0.0f,
  .motor_enc.speed.speedLowScaler           = (float32_t)MOTOR1_ENC_SPEED_LOW_SCALE,
  .motor_enc.speed.speedLow                 = 0.0f,
  .motor_enc.speed.alpha                    = 0.0f,
  //  .motor_enc.speed.speedRef             = 0.0f,
  .motor_enc.speed.speedMech[0]             = 0.0f,
  .motor_enc.speed.speedMech[1]             = 0.0f,
  .motor_enc.speed.speedElec                = 0.0f,
  .motor_enc.speed.theta[NEW]               = 0.0f,
  .motor_enc.speed.theta[OLD]               = 0.0f,
  .motor_enc.speed.speedFlt[0].a            = (float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speed.speedFlt[0].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA,
  .motor_enc.speed.speedFlt[1].a            = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ALPHA,
  .motor_enc.speed.speedFlt[1].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA,
  //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
  .motor_cmd.posRef                         = 0.0f,
  .motor_cmd.posRefOffset                   = 0.0f,
  .motor_cmd.velRef                         = 0.0f,
  .motor_cmd.iqff                           = 0.0f,
  .motor_cmd.kpCoeff                        = 0.0f,
  .motor_cmd.kdCoeff                        = 0.0f,
  .motor_cmd.iSat                           = MOTOR1_CURRENT_CMD_SAT_MAX,
  .motor_cmd.timeoutRef                     = 0,
  .motor_cmd.cptTimeout                     = 0,
  .motor_cmd.index                          = 0,
  .motor_cmd.cmdArray.all                   = 0,
  //--- Initial position for alignment procedure ------------------------------
  .initPos                                  = 0.0f,
  .initPosEnc[FW]                           = 0.0f,
  .initPosEnc[REV]                          = 0.0f,
  //---------------------------------------------------------------------------
  .iParkFlt.a                               = MOTOR1_CURRENT_LPF_ALPHA,
  .iParkFlt.one_minus_a                     = MOTOR1_CURRENT_LPF_ONE_M_ALPHA,
  //--- FOC - Sine / Cosine computation of angular position -------------------
  .sinTheta                                 = 0.0f,
  .cosTheta                                 = 0.0f,
  //--- FOC - Clarke transform results ----------------------------------------
  .ialpha                                   = 0.0f,
  .ibeta                                    = 0.0f,
  //--- FOC - Park transform results ------------------------------------------
  .id                                       = 0.0f,
  .iq                                       = 0.0f,
  //--- FOC - Inverse Park transform results ----------------------------------
  .ud                                       = 0.0f,
  .uq                                       = 0.0f,
  //--- FOC - Inverse Clarke transform results --------------------------------
  .ualpha                                   = 0.0f,
  .ubeta                                    = 0.0f,
  //--- FOC - Phase commands --------------------------------------------------
  .ua                                       = 0.0f,
  .ub                                       = 0.0f,
  .uc                                       = 0.0f,
  //--- FOC - Max current and voltage -----------------------------------------
  .vmax                                     = 0.0f,
//  .imax                                 = MOTOR12_IMAX,
  //--- FOC - Initialize duty cycle saturation --------------------------------
  .dtcMax                                   = MOTOR1_DTC_MAX,
  .dtcMin                                   = MOTOR1_DTC_MIN,
  //--- FOC - Duty cycles -----------------------------------------------------
  .dtc_u                                    = 0.0f,
  .dtc_v                                    = 0.0f,
  .dtc_w                                    = 0.0f,
  //--- FOC - PI setpoint values ----------------------------------------------
  .idRef                                    = 0.0f,
  .iqRef                                    = 0.0f,
  //--- FOC - Initialize PI controller (Id) -----------------------------------
  .piId.p_fb                                = (float32_t *)(&motor_foc[MOTOR_1].id),
  .piId.p_set                               = (float32_t *)(&motor_foc[MOTOR_1].idRef),
  .piId.p_sat                               = (float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus),
  .piId.err                                 = 0.0f,
  .piId.kp                                  = (float32_t)MOTOR1_PI_ID_KP_COEF,
  .piId.ki                                  = (float32_t)MOTOR1_PI_ID_KI_COEF,
  .piId.integral                            = 0.0f,
  .piId.ff                                  = 0.0f,
  .piId.out                                 = 0.0f,
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.p_fb                                = (float32_t *)(&motor_foc[MOTOR_1].iq),
  .piIq.p_set                               = (float32_t *)(&motor_foc[MOTOR_1].iqRef),
  .piIq.p_sat                               = (float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus),
  .piIq.err                                 = 0.0f,
  .piIq.kp                                  = (float32_t)MOTOR1_PI_IQ_KP_COEF,
  .piIq.ki                                  = (float32_t)MOTOR1_PI_IQ_KI_COEF,
  .piIq.integral                            = 0.0f,
  .piIq.ff                                  = 0.0f,
  .piIq.out                                 = 0.0f,
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPosVel.p_fbTheta                       = (float32_t *)(&motor_foc[MOTOR_1].motor_enc.thetaAbsolute),
  .pdPosVel.p_setTheta                      = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.posRefOffset),
  .pdPosVel.errTheta                        = 0.0f,
  .pdPosVel.p_fbSpeed                       = (float32_t *)(&motor_foc[MOTOR_1].motor_enc.speed.speedMech[0]),
  .pdPosVel.p_setSpeed                      = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.velRef),
  .pdPosVel.errSpeed                        = 0.0f,
  .pdPosVel.p_kp                            = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.kpCoeff),
  .pdPosVel.p_kd                            = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.kdCoeff),
  .pdPosVel.derivative                      = 0.0f,
  .pdPosVel.p_ff                            = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.iqff),
  .pdPosVel.p_sat                           = (float32_t *)(&motor_foc[MOTOR_1].motor_cmd.iSat),
  .pdPosVel.out                             = 0.0f,
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resEst                                   = 1.0f,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstFlt.a                              = MOTOR1_STATOR_RESISTOR_LPF_ALPHA,
  .resEstFlt.one_minus_a                    = MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
 },
 //---------------------------------------------------------------------------
 //---------------------------------------------------------------------------
 {
  //--- MOTOR PARAMS STRUCTURE ------------------------------------------------
  .motor_cfg.Rs                             = MOTOR2_RS,
  .motor_cfg.Ls                             = MOTOR2_LS,
  .motor_cfg.kv                             = MOTOR2_KV,
  .motor_cfg.ke                             = MOTOR2_KE,
  .motor_cfg.ki                             = MOTOR2_KI,
  .motor_cfg.polePairs                      = MOTOR2_POLES_PAIRS,
  //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
  .motor_acq.p_vExtMeasReg                  = (volatile uint16_t *)(VEXT2_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg                  = (volatile uint16_t *)(VBUS_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iMeasReg[CHANNEL_1]          = (volatile int16_t *)(MOTOR2_IA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iMeasReg[CHANNEL_2]          = (volatile int16_t *)(MOTOR2_IB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iMeasReg[CHANNEL_3]          = (volatile int16_t *)(MOTOR2_IC_ADC_PPB_RESULT_ADDR),
  .motor_acq.vbus                           = 0.0f,
  .motor_acq.vExt                           = 0.0f,
  .motor_acq.ia                             = 0.0f,
  .motor_acq.ib                             = 0.0f,
  .motor_acq.ic                             = 0.0f,
  .motor_acq.vBusFlt.a                      = VBUS_LPF_ALPHA,
  .motor_acq.vBusFlt.one_minus_a            = VBUS_LPF_ONE_M_ALPHA,
  .motor_acq.vExtFlt.a                      = VEXT_LPF_ALPHA,
  .motor_acq.vExtFlt.one_minus_a            = VEXT_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.eqepBaseAddr                   = MOTOR2_QEP_BASE,
  .motor_enc.polePairs                      = MOTOR2_POLES_PAIRS,
  .motor_enc.thetaElec                      = 0.0f,
  .motor_enc.thetaMech[NEW]                 = 0.0f,
  .motor_enc.thetaMech[OLD]                 = 0.0f,
  .motor_enc.thetaMechScaler                = (float32_t)MOTOR2_ENC_RESOLUTION_SCALE,
  .motor_enc.thetaIndexRelative             = 0.0f,
  .motor_enc.thetaIndexAbsolute             = 0.0f,
  .motor_enc.thetaAbsolute                  = 0.0f,
  .motor_enc.turnNb                         = 0,
  .motor_enc.thetaDir                       = 0,
  .motor_enc.flags.all                      = 0,
  .motor_enc.speed.speedHighScaler          = (float32_t)MOTOR2_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speed.speedHigh                = 0.0f,
  .motor_enc.speed.speedLowScaler           = (float32_t)MOTOR2_ENC_SPEED_LOW_SCALE,
  .motor_enc.speed.speedLow                 = 0.0f,
  .motor_enc.speed.alpha                    = 0.0f,
  //  .motor_enc.speed.speedRef             = 0.0f,
  .motor_enc.speed.speedMech[0]             = 0.0f,
  .motor_enc.speed.speedMech[1]             = 0.0f,
  .motor_enc.speed.speedElec                = 0.0f,
  .motor_enc.speed.theta[NEW]               = 0.0f,
  .motor_enc.speed.theta[OLD]               = 0.0f,
  .motor_enc.speed.speedFlt[0].a            = (float32_t)MOTOR2_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speed.speedFlt[0].one_minus_a  = (float32_t)MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA,
  .motor_enc.speed.speedFlt[1].a            = (float32_t)MOTOR2_ENC_SPEED_LOCAL_LPF_ALPHA,
  .motor_enc.speed.speedFlt[1].one_minus_a  = (float32_t)MOTOR2_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA,
  //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
  .motor_cmd.posRef                         = 0.0f,
  .motor_cmd.posRefOffset                   = 0.0f,
  .motor_cmd.velRef                         = 0.0f,
  .motor_cmd.iqff                           = 0.0f,
  .motor_cmd.kpCoeff                        = 0.0f,
  .motor_cmd.kdCoeff                        = 0.0f,
  .motor_cmd.iSat                           = MOTOR2_CURRENT_CMD_SAT_MAX,
  .motor_cmd.timeoutRef                     = 0,
  .motor_cmd.cptTimeout                     = 0,
  .motor_cmd.index                          = 0,
  .motor_cmd.cmdArray.all                   = 0,
  //--- Initial position for alignment procedure ------------------------------
  .initPos                                  = 0.0f,
  .initPosEnc[FW]                           = 0.0f,
  .initPosEnc[REV]                          = 0.0f,
  //---------------------------------------------------------------------------
  .iParkFlt.a                               = MOTOR2_CURRENT_LPF_ALPHA,
  .iParkFlt.one_minus_a                     = MOTOR2_CURRENT_LPF_ONE_M_ALPHA,
  //--- FOC - Sine / Cosine computation of angular position -------------------
  .sinTheta                                 = 0.0f,
  .cosTheta                                 = 0.0f,
  //--- FOC - Clarke transform results ----------------------------------------
  .ialpha                                   = 0.0f,
  .ibeta                                    = 0.0f,
  //--- FOC - Park transform results ------------------------------------------
  .id                                       = 0.0f,
  .iq                                       = 0.0f,
  //--- FOC - Inverse Park transform results ----------------------------------
  .ud                                       = 0.0f,
  .uq                                       = 0.0f,
  //--- FOC - Inverse Clarke transform results --------------------------------
  .ualpha                                   = 0.0f,
  .ubeta                                    = 0.0f,
  //--- FOC - Phase commands --------------------------------------------------
  .ua                                       = 0.0f,
  .ub                                       = 0.0f,
  .uc                                       = 0.0f,
  //--- FOC - Max current and voltage -----------------------------------------
  .vmax                                     = 0.0f,
//  .imax                                 = MOTOR12_IMAX,
  //--- FOC - Initialize duty cycle saturation --------------------------------
  .dtcMax                                   = MOTOR2_DTC_MAX,
  .dtcMin                                   = MOTOR2_DTC_MIN,
  //--- FOC - Duty cycles -----------------------------------------------------
  .dtc_u                                    = 0.0f,
  .dtc_v                                    = 0.0f,
  .dtc_w                                    = 0.0f,
  //--- FOC - PI setpoint values ----------------------------------------------
  .idRef                                    = 0.0f,
  .iqRef                                    = 0.0f,
  //--- FOC - Initialize PI controller (Id) -----------------------------------
  .piId.p_fb                                = (float32_t *)(&motor_foc[MOTOR_2].id),
  .piId.p_set                               = (float32_t *)(&motor_foc[MOTOR_2].idRef),
  .piId.p_sat                               = (float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus),
  .piId.err                                 = 0.0f,
  .piId.kp                                  = (float32_t)MOTOR2_PI_ID_KP_COEF,
  .piId.ki                                  = (float32_t)MOTOR2_PI_ID_KI_COEF,
  .piId.integral                            = 0.0f,
  .piId.ff                                  = 0.0f,
  .piId.out                                 = 0.0f,
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.p_fb                                = (float32_t *)(&motor_foc[MOTOR_2].iq),
  .piIq.p_set                               = (float32_t *)(&motor_foc[MOTOR_2].iqRef),
  .piIq.p_sat                               = (float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus),
  .piIq.err                                 = 0.0f,
  .piIq.kp                                  = (float32_t)MOTOR2_PI_IQ_KP_COEF,
  .piIq.ki                                  = (float32_t)MOTOR2_PI_IQ_KI_COEF,
  .piIq.integral                            = 0.0f,
  .piIq.ff                                  = 0.0f,
  .piIq.out                                 = 0.0f,
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPosVel.p_fbTheta                       = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.thetaAbsolute),
  .pdPosVel.p_setTheta                      = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.posRefOffset),
  .pdPosVel.errTheta                        = 0.0f,
  .pdPosVel.p_fbSpeed                       = (float32_t *)(&motor_foc[MOTOR_2].motor_enc.speed.speedMech[0]),
  .pdPosVel.p_setSpeed                      = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.velRef),
  .pdPosVel.errSpeed                        = 0.0f,
  .pdPosVel.p_kp                            = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.kpCoeff),
  .pdPosVel.p_kd                            = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.kdCoeff),
  .pdPosVel.derivative                      = 0.0f,
  .pdPosVel.p_ff                            = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.iqff),
  .pdPosVel.p_sat                           = (float32_t *)(&motor_foc[MOTOR_2].motor_cmd.iSat),
  .pdPosVel.out                             = 0.0f,
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resEst                                   = 0.0f,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstFlt.a                              = MOTOR2_STATOR_RESISTOR_LPF_ALPHA,
  .resEstFlt.one_minus_a                    = MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
  //--- Which command set is currently active ? -------------------------------
//  .cmd_cr_active                        = ODD,
//  .cmd_nx_active                        = ODD,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
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
