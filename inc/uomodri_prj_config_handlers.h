#ifndef __UOMODRI_CONFIG_HANDLERS_H__
#define __UOMODRI_CONFIG_HANDLERS_H__

// drivers
#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"

#include "hal.h"
#include "drv8353.h"
#include "foc.h"
#include "motor.h"
#include "communication_interface.h"
#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"

extern __interrupt void adc_isrMotor1(void);
extern __interrupt void adc_isrMotor2(void);
//extern __interrupt void pwm_isrMotor1(void);
//extern __interrupt void pwm_isrMotor2(void);
extern __interrupt void gpio_isrFault1(void);
extern __interrupt void gpio_isrFault2(void);

extern float32_t pdSat;

static ComInterface comInterface =
{
 .systemEnabled     = false,
 .lastCommandIndex  = 0,
 .time              = 0.0,
 .lastMessageMs     = 0,
 .timeoutThreshold  = 0xFFFF,
 .rolloverErrorFlag = false,
 .canError          = SPI_SENSOR_STATUS_ERROR_NO_ERROR,
 .positionRef       = {0.0, 0.0},
 .velocityRef       = {0.0, 0.0},
 .iqRef             = {0.0, 0.0},
 .positionLsb       = CI_POSITION_LSB,
 .velocityLsb       = CI_VELOCITY_LSB,
 .iqLsb             = CI_IQ_LSB,
 .currentSatLsb     = CI_CURRENT_SAT_LSB,
 .resistanceLsb     = CI_LSBISTANCE_LSB,
 .voltageLsb        = CI_VOLTAGE_LSB,
 .kpLsb             = CI_KP_LSB,
 .kdLsb             = CI_KD_LSB,
};

static const IT_TypeDef InterruptList[] =
{
 {.motorID = MOTOR_1,   .intNum = MOTOR1_IA_INT_CH, .intPeriphBase = MOTOR12_IVA_ADC_ADDR,  .intPeriphSrc = MOTOR1_IA_INT_NUM,  .intCpuGroup = INTERRUPT_CPU_INT10, .intAckGroup = INTERRUPT_ACK_GROUP10,   .handler = &adc_isrMotor1},
 {.motorID = MOTOR_2,   .intNum = MOTOR2_IA_INT_CH, .intPeriphBase = MOTOR12_IVA_ADC_ADDR,  .intPeriphSrc = MOTOR2_IA_INT_NUM,  .intCpuGroup = INTERRUPT_CPU_INT10, .intAckGroup = INTERRUPT_ACK_GROUP10,   .handler = &adc_isrMotor2},
// {.motorID = MOTOR_1,   .intNum = INT_EPWM1,        .intPeriphBase = MOTOR1_PWM1_BASE,                                          .intCpuGroup = INTERRUPT_CPU_INT3,  .intAckGroup = INTERRUPT_ACK_GROUP3,    .handler = &pwm_isrMotor1},
// {.motorID = MOTOR_2,   .intNum = INT_EPWM4,        .intPeriphBase = MOTOR2_PWM1_BASE,                                          .intCpuGroup = INTERRUPT_CPU_INT3,  .intAckGroup = INTERRUPT_ACK_GROUP3,    .handler = &pwm_isrMotor2},
 {.motorID = MOTOR_1,   .intNum = INT_XINT1,                                                                                    .intCpuGroup = INTERRUPT_CPU_INT1,  .intAckGroup = INTERRUPT_ACK_GROUP1,    .handler = &gpio_isrFault1},
 //{.motorID = MOTOR_2,   .intNum = INT_XINT2,                                                                                    .intCpuGroup = INTERRUPT_CPU_INT1,  .intAckGroup = INTERRUPT_ACK_GROUP1,    .handler = &gpio_isrFault2},
 // END OF ARRAY
 {.intNum = 0xFFFFFFFF},
};

/** @var    DRV_Cfg     DrvCfgList[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
static const DRV_Cfg DrvCfgList[] =
{
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = MOTOR1_DRV_GPIO_CS, .gpioNumber_EN = MOTOR1_DRV_GPIO_EN},
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = MOTOR2_DRV_GPIO_CS, .gpioNumber_EN = MOTOR2_DRV_GPIO_EN},
// // END OF ARRAY
// {.spiHandle = 0xFFFFFFFF},
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
#if (UOMODRI == 1)
   .Ctrl_Reg_02.ctrl02.OCP_ACT      = true,
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
#if (UOMODRI == 1)
   .Ctrl_Reg_02.ctrl02.OCP_ACT      = true,
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
};

/** @var    DRV8353     drv
 *  @brief  General DRV8353 configuration structure.
 */
static DRV8353 drv[] =
{
 {.drvCfgHandler = &DrvCfgList[MOTOR_1], .drvRegHandler = &DrvRegList[MOTOR_1], .enableTimeOut = true, .rxTimeOut = true},
 {.drvCfgHandler = &DrvCfgList[MOTOR_2], .drvRegHandler = &DrvRegList[MOTOR_2], .enableTimeOut = true, .rxTimeOut = true},
};

static const HAL_MOTOR_CFG hal_motor_cfg[] =
{
 {.motorID      = MOTOR_1,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[0], &PwmCounterCompareCfgList[1], &PwmCounterCompareCfgList[2]},
  .p_vConv      = {&AdcAcqList[0], &AdcAcqList[4], &AdcAcqList[9]},
  .p_iConv      = {&AdcAcqList[2], &AdcAcqList[6], &AdcAcqList[11]},
  .p_vbusConv   = &AdcAcqList[8],
  .p_ITConv     = &AdcIntList[0],
 },
 {.motorID      = MOTOR_2,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[3], &PwmCounterCompareCfgList[4], &PwmCounterCompareCfgList[5]},
  .p_vConv      = {&AdcAcqList[1], &AdcAcqList[5], &AdcAcqList[10]},
  .p_iConv      = {&AdcAcqList[3], &AdcAcqList[7], &AdcAcqList[12]},
  .p_vbusConv   = &AdcAcqList[8],
  .p_ITConv     = &AdcIntList[1],
 },
};

static FOC motor_foc[] =
{
 {
  .motorID      = MOTOR_1,
  .motor_cfg    = {.Rs              = MOTOR1_RS,
                   .Ld              = MOTOR1_LD,
                   .Lq              = MOTOR1_LQ,
                   .kv              = MOTOR1_KV,
                   .ke              = MOTOR1_KE,
                   .ki              = MOTOR1_KI,
                   .polePairs       = MOTOR1_POLES_PAIRS,
  },
  .motor_acq    = {.vaMeasReg       = (volatile uint16_t *)(MOTOR1_VA_ADC_PPB_RESULT_ADDR),
                   .vbMeasReg       = (volatile uint16_t *)(MOTOR1_VB_ADC_PPB_RESULT_ADDR),
                   .vcMeasReg       = (volatile uint16_t *)(MOTOR1_VC_ADC_PPB_RESULT_ADDR),
                   .vBusMeasReg     = (volatile uint16_t *)(MOTOR12_VBUS_ADC_PPB_RESULT_ADDR),
                   .iaMeasReg       = (volatile int16_t *)(MOTOR1_IA_ADC_PPB_RESULT_ADDR),
                   .ibMeasReg       = (volatile int16_t *)(MOTOR1_IB_ADC_PPB_RESULT_ADDR),
                   .icMeasReg       = (volatile int16_t *)(MOTOR1_IC_ADC_PPB_RESULT_ADDR),
                   .va = 0.0, .vb = 0.0, .vc = 0.0, .vbus = 0.0,
                   .ia = 0.0, .ib = 0.0, .ic = 0.0,
                   .measNew         = false,
                   .vBusLpf         = {.a               = MOTOR12_VBUS_LPF_ALPHA,
                                       .one_minus_a     = MOTOR12_VBUS_LPF_ONE_M_ALPHA},
  },
  .motor_enc    = {.p_qepHandle     = &QepCfgList[MOTOR_1],
                   .p_qepRegs       = (struct EQEP_REGS *)(MOTOR1_QEP_BASE),//.p_qepCntPosReg  = (volatile int32_t *)(MOTOR1_QEP_COUNTER_ADDR),
                   .polePairs       = MOTOR1_POLES_PAIRS,//.p_polePairs     = (uint8_t *)(&motor_foc[MOTOR_1].motorParams.polePairs),
                   .thetaMech       = {0.0, 0.0},
                   .thetaElec       = 0.0,
                   .thetaDir        = 0,
//                   .thetaNew        = false,
                   .thetaMechScaler = (float32_t)(MOTOR1_ENC_RESOLUTION_SCALE),
                   .indexCompensated= false,
                   .indexTheta      = 0.0,
                   .indexedThetaMech= 0.0,
                   .thetaAbsolute   = 0.0,
                   .turnNb          = 0,
                   .indexDetected   = false,
                   .indexToggle     = false,
                   .rollOverFlag    = false,
                   .speedEst        = {.speedHighScaler = MOTOR1_ENC_SPEED_HIGH_SCALE,
                                       .speedLowScaler  = MOTOR1_ENC_SPEED_LOW_SCALE,
                                       .speedMech       = PWM_FREQ,
                                       .speedRef        = MOTOR1_ENC_SPEED_CUTOFF_FREQ,
                                       .lpf             = {.a           = MOTOR1_ENC_SPEED_LPF_ALPHA,
                                                           .one_minus_a = MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA}},
  },
//  Initialize control loop low-pass filter
//  alpha = 1 / (1 + (1 / (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
//  Simplifying :
//  alpha = 1 - (1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
//  1 - alpha = 1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ))
  .iParkLpf     = {.a               = MOTOR1_CURRENT_LPF_ALPHA,
                   .one_minus_a     = MOTOR1_CURRENT_LPF_ONE_M_ALPHA,
  },
  //  Initialize PI & PD controllers
  .piId         = {.in              = ((float32_t *)(&motor_foc[MOTOR_1].id)),
                   .set             = ((float32_t *)(&motor_foc[MOTOR_1].idRef)),
                   .kp              = MOTOR1_ID_REGULATOR_KP_COEF,
                   .ki              = MOTOR1_ID_REGULATOR_KI_COEF,
                   .integral        = 0.0,
                   .ff              = 0.0,
                   .sat             = ((float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus)),
  },
  .piIq         = {.in              = ((float32_t *)(&motor_foc[MOTOR_1].iq)),
                   .set             = ((float32_t *)(&motor_foc[MOTOR_1].iqRef)),
                   .kp              = MOTOR1_IQ_REGULATOR_KP_COEF,
                   .ki              = MOTOR1_IQ_REGULATOR_KI_COEF,
                   .integral        = 0.0,
                   .ff              = 0.0,
                   .sat             = ((float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus)),
  },
  .pdPos        = {.inTheta         = ((float32_t *)(&motor_foc[MOTOR_1].motor_enc.thetaAbsolute)),
                   .setTheta        = ((float32_t *)(&comInterface.positionRef[MOTOR_1])),
                   .inSpeed         = ((float32_t *)(&motor_foc[MOTOR_1].motor_enc.speedEst.speedMech)),
                   .setSpeed        = ((float32_t *)(&comInterface.velocityRef[MOTOR_1])),
                   .kp              = MOTOR1_SPEED_REGULATOR_KP_COEF,
                   .kd              = MOTOR1_SPEED_REGULATOR_KD_COEF,
                   .derivative      = 0.0,
                   .ff              = 0.0,
                   .sat             = ((float32_t *)(&pdSat)),
  },
  .idRef        = 0.0,
  .iqRef        = 0.0,
  // FOC max current and voltage
  .vmax         = 0.0,
  .imax         = MOTOR12_IMAX,
  // Initialize duty cycle saturation
  .dtcMax       = MOTOR1_DTC_MAX,
  .dtcMin       = MOTOR1_DTC_MIN,
  // FOC controller enable to start?
  .enabled      = false,
  // FOC - Sine / Cosine results of angular position
  .sinTheta     = 0.0,
  .cosTheta     = 0.0,
  // FOC - Clarke transform results
  .ialpha       = 0.0,
  .ibeta        = 0.0,
  // FOC - Park transform results
  .id           = 0.0,
  .iq           = 0.0,
  // FOC - Inverse Park transform results
  .ud           = 0.0,
  .uq           = 0.0,
  // FOC - Inverse Clarke transform results
  .ualpha       = 0.0,
  .ubeta        = 0.0,

//  .auxAdc = 0.0,
//  .phaseAVolt = 0.0, .phaseBVolt = 0.0, .phaseCVolt = 0.0,
  .resistanceEst = 0.0,
//  .pwmCompA = (volatile uint16_t *)(MOTOR1_PWM1_CMD_ADDR),
//  .pwmCompB = (volatile uint16_t *)(MOTOR1_PWM2_CMD_ADDR),
//  .pwmCompC = (volatile uint16_t *)(MOTOR1_PWM3_CMD_ADDR),
//  .Rs = (float32_t *)(&motor_params[MOTOR_1].Rs),
//  .Ld = (float32_t *)(&motor_params[MOTOR_1].Ld),
//  .Lq = (float32_t *)(&motor_params[MOTOR_1].Lq),
//  .ke = (float32_t *)(&motor_params[MOTOR_1].ke),
 },
 {
  .motorID      = MOTOR_2,
  .motor_cfg    = {.Rs              = MOTOR2_RS,
                   .Ld              = MOTOR2_LD,
                   .Lq              = MOTOR2_LQ,
                   .kv              = MOTOR2_KV,
                   .ke              = MOTOR2_KE,
                   .ki              = MOTOR2_KI,
                   .polePairs       = MOTOR2_POLES_PAIRS,
  },
  .motor_acq    = {.vaMeasReg       = (volatile uint16_t *)(MOTOR2_VA_ADC_PPB_RESULT_ADDR),
                   .vbMeasReg       = (volatile uint16_t *)(MOTOR2_VB_ADC_PPB_RESULT_ADDR),
                   .vcMeasReg       = (volatile uint16_t *)(MOTOR2_VC_ADC_PPB_RESULT_ADDR),
                   .vBusMeasReg     = (volatile uint16_t *)(MOTOR12_VBUS_ADC_PPB_RESULT_ADDR),
                   .iaMeasReg       = (volatile int16_t *)(MOTOR2_IA_ADC_PPB_RESULT_ADDR),
                   .ibMeasReg       = (volatile int16_t *)(MOTOR2_IB_ADC_PPB_RESULT_ADDR),
                   .icMeasReg       = (volatile int16_t *)(MOTOR2_IC_ADC_PPB_RESULT_ADDR),
                   .va = 0.0, .vb = 0.0, .vc = 0.0, .vbus = 0.0,
                   .ia = 0.0, .ib = 0.0, .ic = 0.0,
                   .measNew         = false,
                   .vBusLpf         = {.a               = MOTOR12_VBUS_LPF_ALPHA,
                                       .one_minus_a     = MOTOR12_VBUS_LPF_ONE_M_ALPHA},
  },
  .motor_enc    = {.p_qepHandle     = &QepCfgList[MOTOR_2],
                   .p_qepRegs       = (struct EQEP_REGS *)(MOTOR2_QEP_BASE),//.p_qepCntPosReg  = (volatile int32_t *)(MOTOR2_QEP_COUNTER_ADDR),
                   .polePairs       = MOTOR2_POLES_PAIRS,//.p_polePairs     = (uint8_t *)(&motor_foc[MOTOR_2].motorParams.polePairs),
                   .thetaMech       = {0.0, 0.0},
                   .thetaElec       = 0.0,
                   .thetaDir        = 0,
//                   .thetaNew        = false,
                   .thetaMechScaler = (float32_t)(MOTOR2_ENC_RESOLUTION_SCALE),
                   .indexCompensated= false,
                   .indexTheta      = 0.0,
                   .indexedThetaMech= 0.0,
                   .thetaAbsolute   = 0.0,
                   .turnNb          = 0,
                   .indexDetected   = false,
                   .indexToggle     = false,
                   .rollOverFlag    = false,
                   .speedEst        = {.speedHighScaler = MOTOR2_ENC_SPEED_HIGH_SCALE,
                                       .speedLowScaler  = MOTOR2_ENC_SPEED_LOW_SCALE,
                                       .speedMech       = PWM_FREQ,
                                       .speedRef        = MOTOR2_ENC_SPEED_CUTOFF_FREQ,
                                       .lpf             = {.a           = MOTOR2_ENC_SPEED_LPF_ALPHA,
                                                           .one_minus_a = MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA}},
  },
//  Initialize control loop low-pass filter
//  alpha = 1 / (1 + (1 / (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
//  Simplifying :
//  alpha = 1 - (1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ)))
//  1 - alpha = 1 / (1 + (2 * PI * CUTTOFF_FREQ / PWM_FREQ))
  .iParkLpf     = {.a               = MOTOR2_CURRENT_LPF_ALPHA,
                   .one_minus_a     = MOTOR2_CURRENT_LPF_ONE_M_ALPHA,
  },
  //  Initialize PI & PD controllers
  .idRef = 0.0, .iqRef = 0.0,
  .piId         = {.in              = ((float32_t *)(&motor_foc[MOTOR_2].id)),
                   .set             = ((float32_t *)(&motor_foc[MOTOR_2].idRef)),
                   .kp              = MOTOR2_ID_REGULATOR_KP_COEF,
                   .ki              = MOTOR2_ID_REGULATOR_KI_COEF,
                   .integral        = 0.0,
                   .ff              = 0.0,
                   .sat             = ((float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus)),
  },
  .piIq         = {.in              = ((float32_t *)(&motor_foc[MOTOR_2].iq)),
                   .set             = ((float32_t *)(&motor_foc[MOTOR_2].iqRef)),
                   .kp              = MOTOR2_IQ_REGULATOR_KP_COEF,
                   .ki              = MOTOR2_IQ_REGULATOR_KI_COEF,
                   .integral        = 0.0,
                   .ff              = 0.0,
                   .sat             = ((float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus)),
  },
  .pdPos        = {.inTheta         = ((float32_t *)(&motor_foc[MOTOR_2].motor_enc.thetaAbsolute)),
                   .setTheta        = ((float32_t *)(&comInterface.positionRef[MOTOR_2])),
                   .inSpeed         = ((float32_t *)(&motor_foc[MOTOR_2].motor_enc.speedEst.speedMech)),
                   .setSpeed        = ((float32_t *)(&comInterface.velocityRef[MOTOR_2])),
                   .kp              = MOTOR2_SPEED_REGULATOR_KP_COEF,
                   .kd              = MOTOR2_SPEED_REGULATOR_KD_COEF,
                   .derivative      = 0.0,
                   .ff              = 0.0,
                   .sat             = ((float32_t *)(&pdSat)),
  },
  .idRef        = 0.0,
  .iqRef        = 0.0,
  // FOC max current and voltage
  .vmax         = 0.0,
  .imax         = MOTOR12_IMAX,
  // Initialize duty cycle saturation
  .dtcMax       = MOTOR2_DTC_MAX,
  .dtcMin       = MOTOR2_DTC_MIN,
  // FOC controller enable to start?
  .enabled      = false,
  // FOC - Sine / Cosine results of angular position
  .sinTheta     = 0.0,
  .cosTheta     = 0.0,
  // FOC - Clarke transform results
  .ialpha       = 0.0,
  .ibeta        = 0.0,
  // FOC - Park transform results
  .id           = 0.0,
  .iq           = 0.0,
  // FOC - Inverse Park transform results
  .ud           = 0.0,
  .uq           = 0.0,
  // FOC - Inverse Clarke transform results
  .ualpha       = 0.0,
  .ubeta        = 0.0,

//  .auxAdc = 0.0, .phaseAVolt = 0.0, .phaseBVolt = 0.0, .phaseCVolt = 0.0,
  .resistanceEst = 0.0,

//  .pwmCompA = (volatile uint16_t *)(MOTOR2_PWM1_CMD_ADDR),
//  .pwmCompB = (volatile uint16_t *)(MOTOR2_PWM2_CMD_ADDR),
//  .pwmCompC = (volatile uint16_t *)(MOTOR2_PWM3_CMD_ADDR),
//  .Rs = (float32_t *)(&motor_params[MOTOR_2].Rs),
//  .Ld = (float32_t *)(&motor_params[MOTOR_2].Ld),
//  .Lq = (float32_t *)(&motor_params[MOTOR_2].Lq),
//  .ke = (float32_t *)(&motor_params[MOTOR_2].ke),
 },
};

/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
static MOTOR_STRUCT motor[] =
{
 {
  .motorID          = MOTOR_1,
  .motorHalCfg      = &hal_motor_cfg[MOTOR_1],
  .motorDRV         = &drv[MOTOR_1],
  .motorFOC         = &motor_foc[MOTOR_1],
  .motorAlign       = {.state = ALIGN_COUNT_UP, .counter = 0, .max = PWM_FREQ, .idAlignRef = 0.5, .idAlign = 0.0},// 400mA alignment current
  .statorResEstLpf  = {.a = MOTOR1_STATOR_RESISTOR_LPF_ALPHA, .one_minus_a = MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA},
  .enabled          = false,
  //  .currentScaler    = ADC_CURRENT_SCALE,
  //  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
  //  .voltageScaler    = ADC_VOLTAGE_SCALE,
  //  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
  .pwmChAReg        = (volatile uint16_t *)(MOTOR1_PWM1_CMD_ADDR),
  .pwmChBReg        = (volatile uint16_t *)(MOTOR1_PWM2_CMD_ADDR),
  .pwmChCReg        = (volatile uint16_t *)(MOTOR1_PWM3_CMD_ADDR),
 },
 {
  .motorID          = MOTOR_2,
  .motorHalCfg      = &hal_motor_cfg[MOTOR_2],
  .motorDRV         = &drv[MOTOR_2],
  .motorFOC         = &motor_foc[MOTOR_2],
  .motorAlign       = {.state = ALIGN_COUNT_UP, .counter = 0, .max = PWM_FREQ, .idAlignRef = 0.5, .idAlign = 0.0},// 400mA alignment current
  .statorResEstLpf  = {.a = MOTOR2_STATOR_RESISTOR_LPF_ALPHA, .one_minus_a = MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA},
  .enabled          = false,
  //  .currentScaler    = ADC_CURRENT_SCALE,
  //  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
  //  .voltageScaler    = ADC_VOLTAGE_SCALE,
  //  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
  .pwmChAReg        = (volatile uint16_t *)(MOTOR2_PWM1_CMD_ADDR),
  .pwmChBReg        = (volatile uint16_t *)(MOTOR2_PWM2_CMD_ADDR),
  .pwmChCReg        = (volatile uint16_t *)(MOTOR2_PWM3_CMD_ADDR),
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
