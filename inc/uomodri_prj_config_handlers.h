#ifndef __UOMODRI_CONFIG_HANDLERS_H__
#define __UOMODRI_CONFIG_HANDLERS_H__

// drivers
#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"

#include "hal.h"
#include "drv8353.h"
#include "encoder.h"
#include "foc.h"
#include "motor.h"
#include "spi_communication.h"
#include "align_routine.h"

#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"

// Communication dual buffers for initialization
extern DualBufferW dualBufferW;
extern DualBufferR dualBufferR;

extern __interrupt void adc1ISR(void);
extern __interrupt void adc2ISR(void);
extern __interrupt void adc3ISR(void);
extern __interrupt void adc4ISR(void);
extern __interrupt void isrMotor1(void);
extern __interrupt void isrMotor2(void);

static const IT_TypeDef InterruptList[] =
{
 {.intNum = INT_ADCA1, .handler = &adc1ISR},
 {.intNum = INT_ADCA2, .handler = &adc2ISR},
 {.intNum = INT_ADCA3, .handler = &adc3ISR},
 {.intNum = INT_ADCA4, .handler = &adc4ISR},
 {.intNum = INT_EPWM1, .handler = &isrMotor1},
 {.intNum = INT_EPWM4, .handler = &isrMotor2},
 // END OF ARRAY
 {.intNum = 0xFFFFFFFF},
};

/** @var    DRV_Cfg     DrvCfgList[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
static const DRV_Cfg DrvCfgList[] =
{
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = DRV1_GPIO_CS, .gpioNumber_EN = DRV1_GPIO_EN},
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = DRV2_GPIO_CS, .gpioNumber_EN = DRV2_GPIO_EN},
 // END OF ARRAY
 {.spiHandle = 0xFFFFFFFF},
};

/** @var    DRV_Reg     DrvRegList[]
 *  @brief  List all configuration registers for each DRVs.
 */
static DRV_Reg DrvRegList[] =
{
 { .Ctrl_Reg_02.ctrl02.CLR_FLT      = false,
   .Ctrl_Reg_02.ctrl02.BRAKE        = false,
   .Ctrl_Reg_02.ctrl02.COAST        = false,
   .Ctrl_Reg_02.ctrl02.PWM1_DIR     = false,
   .Ctrl_Reg_02.ctrl02.PWM1_COM     = false,
   .Ctrl_Reg_02.ctrl02.PWM_MODE     = DRV_PWMMODE_6,
   .Ctrl_Reg_02.ctrl02.OTW_REP      = true,                 // Over Temperature Warning will raise nFault
   .Ctrl_Reg_02.ctrl02.DIS_GDF      = false,
   .Ctrl_Reg_02.ctrl02.DIS_GDUV     = false,
#ifdef UOMODRI
   .Ctrl_Reg_02.Ctrl02.OCP_ACT      = true,
#endif
   // Gate Drive HS register initialization (Address: 0x3)
   .Ctrl_Reg_03.ctrl03.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
   .Ctrl_Reg_03.ctrl03.IDRIVEP_HS   = DRV_ISOUR_HS_0P300_A,
   .Ctrl_Reg_03.ctrl03.LOCK         = DRV_REG_UNLOCK,
   // Gate Drive LS register initialization (Address: 0x4)
   .Ctrl_Reg_04.ctrl04.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
   .Ctrl_Reg_04.ctrl04.IDRIVEP_LS   = DRV_ISOUR_LS_0P300_A,
   .Ctrl_Reg_04.ctrl04.TDRIVE       = DRV_TSOUR_4000_NS,
   .Ctrl_Reg_04.ctrl04.CBC          = false,
   // Over Current Protection register initialization (Address: 0x5)
   .Ctrl_Reg_05.ctrl05.VDS_LVL      = DRV_VDS_LVL_0P700_V,
   .Ctrl_Reg_05.ctrl05.OCP_DEG      = DRV_OCPDEG_8_US,
   .Ctrl_Reg_05.ctrl05.OCP_MODE     = DRV_AUTOMATIC_RETRY,
   .Ctrl_Reg_05.ctrl05.DEAD_TIME    = DRV_DEADTIME_400_NS,
   .Ctrl_Reg_05.ctrl05.TRETRY       = false,
   // Current Sense Amplifier register initialization (Address: 0x6)
   .Ctrl_Reg_06.ctrl06.SEN_LVL      = DRV_SENLVL_1P00_V,
   .Ctrl_Reg_06.ctrl06.CSA_CAL_C    = false,
   .Ctrl_Reg_06.ctrl06.CSA_CAL_B    = false,
   .Ctrl_Reg_06.ctrl06.CSA_CAL_A    = false,
   .Ctrl_Reg_06.ctrl06.DIS_SEN      = false,
   .Ctrl_Reg_06.ctrl06.CSA_GAIN     = DRV_CSA_GAIN_REG,
   .Ctrl_Reg_06.ctrl06.LS_REF       = false,
   .Ctrl_Reg_06.ctrl06.VREF_DIV     = true,
   .Ctrl_Reg_06.ctrl06.CSA_FET      = false,
   // Amplifier calibration calibration routine (Address: 0x7)
   .Ctrl_Reg_07.ctrl07.CAL_MODE     = true
 },
 { .Ctrl_Reg_02.ctrl02.CLR_FLT      = false,
   .Ctrl_Reg_02.ctrl02.BRAKE        = false,
   .Ctrl_Reg_02.ctrl02.COAST        = false,
   .Ctrl_Reg_02.ctrl02.PWM1_DIR     = false,
   .Ctrl_Reg_02.ctrl02.PWM1_COM     = false,
   .Ctrl_Reg_02.ctrl02.PWM_MODE     = DRV_PWMMODE_6,
   .Ctrl_Reg_02.ctrl02.OTW_REP      = true,                 // Over Temperature Warning will raise nFault
   .Ctrl_Reg_02.ctrl02.DIS_GDF      = false,
   .Ctrl_Reg_02.ctrl02.DIS_GDUV     = false,
#ifdef UOMODRI
   .Ctrl_Reg_02.Ctrl02.OCP_ACT      = true,
#endif
   // Gate Drive HS register initialization (Address: 0x3)
   .Ctrl_Reg_03.ctrl03.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
   .Ctrl_Reg_03.ctrl03.IDRIVEP_HS   = DRV_ISOUR_HS_0P300_A,
   .Ctrl_Reg_03.ctrl03.LOCK         = DRV_REG_UNLOCK,
   // Gate Drive LS register initialization (Address: 0x4)
   .Ctrl_Reg_04.ctrl04.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
   .Ctrl_Reg_04.ctrl04.IDRIVEP_LS   = DRV_ISOUR_LS_0P300_A,
   .Ctrl_Reg_04.ctrl04.TDRIVE       = DRV_TSOUR_4000_NS,
   // Over Current Protection register initialization (Address: 0x5)
   .Ctrl_Reg_05.ctrl05.VDS_LVL      = DRV_VDS_LVL_0P600_V,
   .Ctrl_Reg_05.ctrl05.OCP_DEG      = DRV_OCPDEG_8_US,
   .Ctrl_Reg_05.ctrl05.OCP_MODE     = DRV_AUTOMATIC_RETRY,
   .Ctrl_Reg_05.ctrl05.DEAD_TIME    = DRV_DEADTIME_400_NS,
   .Ctrl_Reg_05.ctrl05.TRETRY       = false,
   // Current Sense Amplifier register initialization (Address: 0x6)
   .Ctrl_Reg_06.ctrl06.SEN_LVL      = DRV_SENLVL_1P00_V,
   .Ctrl_Reg_06.ctrl06.CSA_CAL_C    = false,
   .Ctrl_Reg_06.ctrl06.CSA_CAL_B    = false,
   .Ctrl_Reg_06.ctrl06.CSA_CAL_A    = false,
   .Ctrl_Reg_06.ctrl06.DIS_SEN      = false,
   .Ctrl_Reg_06.ctrl06.CSA_GAIN     = DRV_CSA_GAIN_REG,
   .Ctrl_Reg_06.ctrl06.LS_REF       = false,
   .Ctrl_Reg_06.ctrl06.VREF_DIV     = true,
   .Ctrl_Reg_06.ctrl06.CSA_FET      = false,
   // Amplifier calibration calibration routine (Address: 0x7)
   .Ctrl_Reg_07.ctrl07.CAL_MODE     = true
 },
};

/** @var    DRV8353     drv
 *  @brief  General DRV8353 configuration structure.
 */
static DRV8353 drv[] =
{
 {.drvCfgHandler = &DrvCfgList[MOTOR_1], .drvRegHandler = &DrvRegList[MOTOR_1], .enableTimeOut = true, .rxTimeOut = true},
 {.drvCfgHandler = &DrvCfgList[MOTOR_2], .drvRegHandler = &DrvRegList[MOTOR_2], .enableTimeOut = true, .rxTimeOut = true},
 {.drvCfgHandler = NULL},
};

static const MOTOR_HAL_CFG motor_hal_cfg[] =
{
 {.motorID      = MOTOR_1,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[0], &PwmCounterCompareCfgList[1], &PwmCounterCompareCfgList[2]},
  .p_vConv      = {&AdcAcqList[0], &AdcAcqList[4], &AdcAcqList[9]},
  .p_iConv      = {&AdcAcqList[2], &AdcAcqList[6], &AdcAcqList[11]},
  .p_vbusConv   = &AdcAcqList[8],
  .p_ITConv     = {&AdcIntList[0], &AdcIntList[2]},
//  .p_ia         = ((volatile int16_t *)(.p_iConv[0]->adcPPBResultBase)),
//  .p_ib         = (volatile int16_t *)(.p_iConv[1]->adcPPBResultBase),
//  .p_ic         = (volatile int16_t *)(.p_iConv[2]->adcPPBResultBase),
//  .p_iMesured   = {(*((volatile uint16_t *)(.p_iConv[0]->adcResultBase + ADC_O_PPB1RESULT))), (&((.p_iConv[0]->adcResultBase + ADC_O_PPB1RESULT))), (&((.p_iConv[0]->adcResultBase + ADC_O_PPB1RESULT)))},
//  .p_iMesured   = {(*((volatile uint16_t *)(AdcPPBsList[0].adcResultBase)))},
//  .p_iMesured   = {&(.p_iConv[0]->adcResultBase), (AdcPPBsList[4].adcResultBase), (AdcPPBsList[10].adcResultBase)},
//  .p_iMesured   = {((volatile int16_t *)(AdcPPBsList[0].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[4].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[10].adcResultBase))},
//  .p_vMesured   = {((volatile int16_t *)(AdcPPBsList[2].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[6].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[12].adcResultBase))},
//  .p_vBusMesured= (volatile int16_t *)(AdcPPBsList[8].adcResultBase),
//  .dataValid    = {false, false},
  //  .offsetCal      = true
 },
 {.motorID      = MOTOR_2,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[3], &PwmCounterCompareCfgList[4], &PwmCounterCompareCfgList[5]},
  .p_vConv      = {&AdcAcqList[1], &AdcAcqList[5], &AdcAcqList[10]},
  .p_iConv      = {&AdcAcqList[3], &AdcAcqList[7], &AdcAcqList[12]},
  .p_vbusConv   = &AdcAcqList[8],
  .p_ITConv     = {&AdcIntList[1], &AdcIntList[3]},
//  .p_ia         = (((volatile int16_t *)((uintptr_t)(M2_IA_ADC_PPB_RESULT_ADDR)))),
//  .p_ia         = (&((volatile int16_t *)((AdcPPBsList[1].adcPPBResultBase)))),
//  .p_ia         = (((volatile int16_t *)((uintptr_t)&(AdcPPBsList[1]->adcPPBResultBase)))),
//  .p_ia         = (&((volatile uint16_t *)((uintptr_t)(motors_acq[MOTOR_2].p_iConv[0]->adcPPBResultBase)))),
//  .p_ia         = ((int32_t *)(AdcPPBsList[1].adcPPBResultBase)),
//  .p_iMesured   = {((volatile int32_t *)(AdcPPBsList[1].adcPPBResultBase)), ((volatile int32_t *)(AdcPPBsList[5].adcPPBResultBase)), ((volatile int32_t *)(AdcPPBsList[11].adcPPBResultBase))},
//  .p_iMesured   = {(*((volatile uint16_t *)(AdcPPBsList[1].adcResultBase)))},
//
//  {((volatile int16_t *)(AdcPPBsList[1].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[5].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[11].adcResultBase))},
//  .p_vMesured   = {((volatile int16_t *)(AdcPPBsList[3].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[7].adcResultBase)),
//                   ((volatile int16_t *)(AdcPPBsList[13].adcResultBase))},
//  .p_vBusMesured= (volatile int16_t *)(AdcPPBsList[9].adcResultBase),
//  .dataValid    = {false, false},
  //  .offsetCal      = true
 },
// {.motorID      = NO_MOT},
};

static const MOTOR_PARAMS motor_params[] =
{
 {.Rs = MOTOR1_RS, .Ld = MOTOR1_LD, .Lq = MOTOR1_LQ, .kv = MOTOR1_KV, .ke = MOTOR1_KE, .ki = MOTOR1_KI, .polePairs = MOTOR1_POLES_PAIRS},
 {.Rs = MOTOR2_RS, .Ld = MOTOR2_LD, .Lq = MOTOR2_LQ, .kv = MOTOR2_KV, .ke = MOTOR2_KE, .ki = MOTOR2_KI, .polePairs = MOTOR2_POLES_PAIRS},
};

static ENCODER encoder[] =
{
 {
  .motorID          = MOTOR_1,
  .p_qepHandle      = &QepCfgList[0],
  .p_qepRegs        = (struct EQEP_REGS *)(MOTOR1_QEP_BASE),
  .p_qepCntPosReg   = (volatile int32_t *)(MOTOR1_QEP_COUNTER_ADDR),
  .p_polePairs      = (uint8_t *)(&motor_params[MOTOR_1].polePairs),
  .mechPos          = {0.0, 0.0},
  .mechPosNorm      = (float32_t)(MOTOR1_ENC_RESOLUTION_SCALE),

  .speedEst.highSpeedNorm   = MOTOR1_ENC_HIGH_SPEED_SCALE,
  .speedEst.lowSpeedNorm    = MOTOR1_ENC_LOW_SPEED_SCALE,
  .speedEst.mechSpeed       = PWM_FREQUENCY,
  .speedEst.speedRef        = 200,
 },
 {
  .motorID          = MOTOR_2,
  .p_qepHandle      = &QepCfgList[1],
  .p_qepRegs        = (struct EQEP_REGS *)(MOTOR2_QEP_BASE),
  .p_qepCntPosReg   = (volatile int32_t *)(MOTOR2_QEP_COUNTER_ADDR),
  .p_polePairs      = (uint8_t *)(&motor_params[MOTOR_2].polePairs),
  .mechPos          = {0.0, 0.0},
  .mechPosNorm      = (float32_t)(MOTOR2_ENC_RESOLUTION_SCALE),
 },
};

static MOTOR_MEASURES motor_meas[] =
{
 {.motorID          = MOTOR_1,
  .ia = 0.0f, .ib = 0.0f, .ic = 0.0f,
  .va = 0.0f, .vb = 0.0f, .vc = 0.0f,
  .mechanicalAngle  = &encoder[MOTOR_1].mechPos[NEW],
  .vaMeasReg        = (volatile int32_t *)(MOTOR1_VA_ADC_PPB_RESULT_ADDR),
  .vbMeasReg        = (volatile int32_t *)(MOTOR1_VB_ADC_PPB_RESULT_ADDR),
  .vcMeasReg        = (volatile int32_t *)(MOTOR1_VC_ADC_PPB_RESULT_ADDR),
  .vBusMeasReg      = (volatile int32_t *)(MOTOR12_VBUS_ADC_PPB_RESULT_ADDR),
  .iaMeasReg        = (volatile int32_t *)(MOTOR1_IA_ADC_PPB_RESULT_ADDR),
  .ibMeasReg        = (volatile int32_t *)(MOTOR1_IB_ADC_PPB_RESULT_ADDR),
  .icMeasReg        = (volatile int32_t *)(MOTOR1_IC_ADC_PPB_RESULT_ADDR),
  .encoderPosReg    = (volatile int32_t *)(&encoder[MOTOR_1].p_qepCntPosReg),
 },
 {.motorID          = MOTOR_2,
  .ia = 0.0f, .ib = 0.0f, .ic = 0.0f,
  .va = 0.0f, .vb = 0.0f, .vc = 0.0f,
  .mechanicalAngle  = &encoder[MOTOR_2].mechPos[NEW],
  .vaMeasReg        = (volatile int32_t *)(MOTOR2_VA_ADC_PPB_RESULT_ADDR),
  .vbMeasReg        = (volatile int32_t *)(MOTOR2_VB_ADC_PPB_RESULT_ADDR),
  .vcMeasReg        = (volatile int32_t *)(MOTOR2_VC_ADC_PPB_RESULT_ADDR),
  .vBusMeasReg      = (volatile int32_t *)(MOTOR12_VBUS_ADC_PPB_RESULT_ADDR),
  .iaMeasReg        = (volatile int32_t *)(MOTOR2_IA_ADC_PPB_RESULT_ADDR),
  .ibMeasReg        = (volatile int32_t *)(MOTOR2_IB_ADC_PPB_RESULT_ADDR),
  .icMeasReg        = (volatile int32_t *)(MOTOR2_IC_ADC_PPB_RESULT_ADDR),
  .encoderPosReg    = (volatile int32_t *)(&(encoder[MOTOR_2].p_qepCntPosReg)),
 },
};

static FOC motor_foc[] =
{
 {.enabled = true,
  .piId.integral = 0.0f, .piId.err = 0.0f, .piId.ff = 0.0f, .piId.out = 0.0f,
  .piIq.integral = 0.0f, .piIq.err = 0.0f, .piIq.ff = 0.0f, .piIq.out = 0.0f,
  .posPd.err = 0.0f, .posPd.vErr = 0.0f, .posPd.out = 0.0f,
  .alignRoutine.state = ALIGN_COUNT_UP, .alignRoutine.counter = 0, .alignRoutine.idAlign = 0.0f,

  .encoder.mechPos = {0.0, 0.0}, .encoder.elecPos = 0.0f,
  .encoder.isIndexCompensated = false, .encoder.indexAngle = 0.0f, .encoder.indexedMechAngle = 0.0f,
  .encoder.lastMechAngle = 0.0f, .encoder.absoluteAngle = 0.0f,
  .encoder.speedEst.alpha = 0.0f,
  .encoder.speedEst.speedRef = 0.0f,
  .encoder.speedEst.pos = {0.0f, 0.0f},
  .encoder.speedEst.lowSpeed = 0.0f,
  .encoder.speedEst.highSpeed = 0.0f,
  .encoder.speedEst.mechSpeed = 0.0f,
  .encoder.speedEst.elecSpeed = 0.0f,
  .encoder.turnNb = 0,
  .encoder.indexDetected = false,
  .encoder.indexToggle = false,
  .encoder.rollOverFlag = false,

  .ia = 0.0f, .ib = 0.0f, .ic = 0.0f, .ialpha = 0.0f, .ibeta = 0.0f, .id = 0.0f, .iq = 0.0f,
  .va = 0.0f, .vb = 0.0f, .vc = 0.0f, .valpha = 0.0f, .vbeta = 0.0f, .vd = 0.0f, .vq = 0.0f,

  .mechAngle = (float32_t *)(&encoder[MOTOR_1].mechPos[NEW]),
  .elecAngle = (float32_t *)(&encoder[MOTOR_1].elecPos),
  .sinTheta = 0.0f, .cosTheta = 0.0f,

  .auxAdc = 0.0f, .phaseAVolt = 0.0f, .phaseBVolt = 0.0f, .phaseCVolt = 0.0f, .resistanceEst = 0.0f,
  .Rs = (float32_t *)(&motor_params[MOTOR_1].Rs),
  .Ld = (float32_t *)(&motor_params[MOTOR_1].Ld),
  .Lq = (float32_t *)(&motor_params[MOTOR_1].Lq),
  .ke = (float32_t *)(&motor_params[MOTOR_1].ke),
 },
 {.enabled = true,
  .piId.integral = 0.0f, .piId.err = 0.0f, .piId.ff = 0.0f, .piId.out = 0.0f,
  .piIq.integral = 0.0f, .piIq.err = 0.0f, .piIq.ff = 0.0f, .piIq.out = 0.0f,
  .posPd.err = 0.0f, .posPd.vErr = 0.0f, .posPd.out = 0.0f,
  .alignRoutine.state = ALIGN_COUNT_UP, .alignRoutine.counter = 0, .alignRoutine.idAlign = 0.0f,
  .ia = 0.0f, .ib = 0.0f, .ic = 0.0f, .ialpha = 0.0f, .ibeta = 0.0f, .id = 0.0f, .iq = 0.0f,
  .va = 0.0f, .vb = 0.0f, .vc = 0.0f, .valpha = 0.0f, .vbeta = 0.0f, .vd = 0.0f, .vq = 0.0f,

  .mechAngle = (float32_t *)(&encoder[MOTOR_2].mechPos[NEW]),
  .elecAngle = (float32_t *)(&encoder[MOTOR_2].elecPos),
  .sinTheta = 0.0f, .cosTheta = 0.0f,

  .auxAdc = 0.0f, .phaseAVolt = 0.0f, .phaseBVolt = 0.0f, .phaseCVolt = 0.0f, .resistanceEst = 0.0f,
  .Rs = (float32_t *)(&motor_params[MOTOR_2].Rs),
  .Ld = (float32_t *)(&motor_params[MOTOR_2].Ld),
  .Lq = (float32_t *)(&motor_params[MOTOR_2].Lq),
  .ke = (float32_t *)(&motor_params[MOTOR_2].ke),
 },
};
static MOTOR_STRUCT motor[] =
{
 {
  .motorID = MOTOR_1,
  .motorHalCfg = &motor_hal_cfg[MOTOR_1],
  .motorParams = &motor_params[MOTOR_1],
  .motorMeas = &motor_meas[MOTOR_1],
  .motorEncoder = &encoder[MOTOR_1],
  .motorDRV = &drv[MOTOR_1],
  .motorFOC = &motor_foc[MOTOR_1],
  .currentScaler = ADC_CURRENT_SCALE,
  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
  .voltageScaler = ADC_VOLTAGE_SCALE,
  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
 },
 {
  .motorID = MOTOR_2,
  .motorHalCfg = &motor_hal_cfg[MOTOR_2],
  .motorParams = &motor_params[MOTOR_2],
  .motorMeas = &motor_meas[MOTOR_1],
  .motorEncoder = &encoder[MOTOR_2],
  .motorDRV = &drv[MOTOR_2],
  .motorFOC = &motor_foc[MOTOR_2],
  .currentScaler = ADC_CURRENT_SCALE,
  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
  .voltageScaler = ADC_VOLTAGE_SCALE,
  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
 },
};

//static const ePWM_TypeDef PwmCfgList[] =
//{
// {MOTOR1_PWM1_BASE, SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT, true},
// {MOTOR1_PWM2_BASE, SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT, true},
// {MOTOR1_PWM3_BASE, SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT, true},
// {MOTOR2_PWM1_BASE, SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT, true},
// {MOTOR2_PWM2_BASE, SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT, true},
// {MOTOR2_PWM3_BASE, SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT, true},
// // END OF ARRAY
// {0xFFFFFFFF, SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT, true},
//};

#endif  /*__UOMODRI_CONFIG_HANDLERS_H__*/
