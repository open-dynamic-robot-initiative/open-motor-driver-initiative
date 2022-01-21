#ifndef __UOMODRI_PRJ_CONFIG_HANDLERS_H__
#define __UOMODRI_PRJ_CONFIG_HANDLERS_H__

// drivers
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

//extern const GPIO_TypeDef GpioPinsCfgList[];
//extern const ADC_INIT_TypeDef AdcInitList[];
//extern const ADC_ACQ_TypeDef AdcAcqList[];
//extern const ADC_IT_TypeDef AdcIntList[];
//extern const ADC_TypeDef AdcCfgList[];
//extern const eQEP_TypeDef QepCfgList[];
//extern const SPI_TypeDef spiCfgList[];
//extern const DMA_TypeDef dmaCfgList[];
//extern const IPC_TypeDef ipcCfgList[];
//extern const ePWM_TB_TypeDef PwmTimeBaseCfgList[];
//extern const ePWM_CC_TypeDef PwmCounterCompareCfgList[];
//extern const ePWM_AQ_TypeDef PwmActionQualifierCfgList[];
//extern const ePWM_DB_TypeDef PwmDeadBandCfgList[];
//extern const ePWM_ET_TypeDef PwmAdcSocEventTriggerCfgList[];
//extern const ePWM_IT_TypeDef PwmInterruptsEventCfgList[];
//extern const ePWM_TypeDef PwmCfgList[];
//extern       HAL hal;

//extern uint8_t      polePairs;    // Number of pole pairs of the motor
//extern float32_t    thetaMechScaler; // [pu/register]
//extern float32_t    thetaMech[2]; // [pu]
//extern bool         thetaDir; //0 = CCW/reverse, 1 = CW/forward
//extern float32_t    thetaElec; // [pu]
//extern float32_t    thetaMechCal[2]; // [pu]
//extern float32_t    pdSat;
//__attribute__((interrupt)) void ipc_isrStatusMsg(void);
//__attribute__((interrupt)) void ipc_isrMsgCmd(void);
__attribute__((interrupt)) void dma_isrCmdMsg(void);
//__attribute__((interrupt)) void spi_isrRx(void);
//__attribute__((interrupt)) void dma5_isr(void);
__attribute__((interrupt)) void adc_isrMotor1(void);
__attribute__((interrupt)) void adc_isrMotor2(void);
//extern __interrupt void pwm_isrMotor1(void);
//extern __interrupt void pwm_isrMotor2(void);
__attribute__((interrupt)) void gpio_isrFaultMotor1(void);
__attribute__((interrupt)) void gpio_isrFaultMotor2(void);

//__attribute__((interrupt)) void cla1_isr1(void);
//CLA C Tasks defined in Cla1Tasks_C.cla
//__attribute__((interrupt)) void Cla1Task1(void);
//__attribute__((interrupt)) void Cla1Task8(void);

//static ComInterface comInterface =
//{
// .systemEnabled     = false,
// .lastCommandIndex  = 0,
// .time              = {0.0, 0.0},
// .lastMessageMs     = 0,
// .timeoutThreshold  = 0xFFFF,
// .rolloverErrorFlag = false,
// .canError          = SPI_SENSOR_STATUS_ERROR_NO_ERROR,
// .positionRef       = {0.0, 0.0},
// .velocityRef       = {0.0, 0.0},
// .iqRef             = {0.0, 0.0},
// .positionLsb       = CI_POSITION_LSB,
// .velocityLsb       = CI_VELOCITY_LSB,
// .iqLsb             = CI_IQ_LSB,
// .currentSatLsb     = CI_CURRENT_SAT_LSB,
// .resistanceLsb     = CI_LSBISTANCE_LSB,
// .voltageLsb        = CI_VOLTAGE_LSB,
// .kpLsb             = CI_KP_LSB,
// .kdLsb             = CI_KD_LSB,
//};

static const IT_TypeDef InterruptList[] =
{
// {.motorID = MOTOR_1,   .intNum = MOTOR1_IA_INT_CH, .intPeriphBase = MOTOR12_IVA_ADC_ADDR,  .intPeriphSrc = MOTOR1_IA_INT_NUM,  .intCpuGroup = INTERRUPT_CPU_INT10, .intAckGroup = INTERRUPT_ACK_GROUP10,   .handler = &adc_isrMotor1},
// {.motorID = MOTOR_2,   .intNum = MOTOR2_IA_INT_CH, .intPeriphBase = MOTOR12_IVA_ADC_ADDR,  .intPeriphSrc = MOTOR2_IA_INT_NUM,  .intCpuGroup = INTERRUPT_CPU_INT10, .intAckGroup = INTERRUPT_ACK_GROUP10,   .handler = &adc_isrMotor2},
//// {.motorID = MOTOR_1,   .intNum = INT_EPWM1,        .intPeriphBase = MOTOR1_PWM1_BASE,                                          .intCpuGroup = INTERRUPT_CPU_INT3,  .intAckGroup = INTERRUPT_ACK_GROUP3,    .handler = &pwm_isrMotor1},
//// {.motorID = MOTOR_2,   .intNum = INT_EPWM4,        .intPeriphBase = MOTOR2_PWM1_BASE,                                          .intCpuGroup = INTERRUPT_CPU_INT3,  .intAckGroup = INTERRUPT_ACK_GROUP3,    .handler = &pwm_isrMotor2},
// {.motorID = MOTOR_1,   .intNum = INT_XINT1,                                                                                    .intCpuGroup = INTERRUPT_CPU_INT1,  .intAckGroup = INTERRUPT_ACK_GROUP1,    .handler = &gpio_isrFault1},
// {.motorID = MOTOR_2,   .intNum = INT_XINT2,                                                                                    .intCpuGroup = INTERRUPT_CPU_INT1,  .intAckGroup = INTERRUPT_ACK_GROUP1,    .handler = &gpio_isrFault2},
 {
  .intNum           = MOTOR1_IA_INT_CH,
  .intAckGroup      = MOTOR1_IA_INT_ACK_GROUP,
  .p_intHandler     = &adc_isrMotor1,
 }, //
 {
  .intNum           = MOTOR2_IA_INT_CH,
  .intAckGroup      = MOTOR2_IA_INT_ACK_GROUP,
  .p_intHandler     = &adc_isrMotor2,
 }, //
// {
//  .intNum           = INT_CMTOCPUXIPC0,
//  .intAckGroup      = INTERRUPT_ACK_GROUP11,
//  .handler          = &ipc_isrStatusMsg,
// },
// {
//  .intNum           = INT_CMTOCPUXIPC1,
//  .intAckGroup      = INTERRUPT_ACK_GROUP11,
//  .handler          = &ipc_isrMsgCmd,
// },

 {
  .intNum           = DMA_COM_SPI_RX_INT,
  .intAckGroup      = DMA_COM_SPI_RX_INT_ACK_GROUP,
  .p_intHandler     = &dma_isrCmdMsg,
 },
// {
//  .intNum           = COM_SPI_RX_INT,
//  .intAckGroup      = INTERRUPT_ACK_GROUP6,
//  .p_intHandler     = &spi_isrRx,
// },
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
#if (defined CLA_ENABLE) && (CLA_ENABLE)
 {
  .intNum           = INT_CLA1_1,
  .intAckGroup      = INTERRUPT_ACK_GROUP11,
  .handler          = &cla1_isr1
 },
#endif
 // END OF ARRAY
 {.intNum = 0xFFFFFFFF},
};

/** @var    DRV_Cfg     DrvCfgList[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
static const DRV_Cfg DrvCfgList[] =
{
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = MOTOR1_DRV_SPI_CS, .gpioNumber_EN = MOTOR1_DRV_GPIO_EN},
 {.spiHandle = DRV_SPI_BASE, .gpioNumber_CS = MOTOR2_DRV_SPI_CS, .gpioNumber_EN = MOTOR2_DRV_GPIO_EN},
// // END OF ARRAY
// {.spiHandle = 0xFFFFFFFF},
};

/** @var    DRV_Reg     DrvRegList[]
 *  @brief  List all configuration registers for each DRVs.
 */
static DRV_Reg DrvRegList[] =
{
 {
   .Ctrl_Reg_02.ctrl02.CLR_FLT      = false,
   .Ctrl_Reg_02.ctrl02.BRAKE        = false,
   .Ctrl_Reg_02.ctrl02.COAST        = false,
   .Ctrl_Reg_02.ctrl02.PWM1_DIR     = false,
   .Ctrl_Reg_02.ctrl02.PWM1_COM     = false,
   .Ctrl_Reg_02.ctrl02.PWM_MODE     = DRV_PWMMODE_6,
   .Ctrl_Reg_02.ctrl02.OTW_REP      = true,                 // Over Temperature Warning will raise nFault
   .Ctrl_Reg_02.ctrl02.DIS_GDF      = false,
   .Ctrl_Reg_02.ctrl02.DIS_GDUV     = false,
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
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
 {
   .Ctrl_Reg_02.ctrl02.CLR_FLT      = false,
   .Ctrl_Reg_02.ctrl02.BRAKE        = false,
   .Ctrl_Reg_02.ctrl02.COAST        = false,
   .Ctrl_Reg_02.ctrl02.PWM1_DIR     = false,
   .Ctrl_Reg_02.ctrl02.PWM1_COM     = false,
   .Ctrl_Reg_02.ctrl02.PWM_MODE     = DRV_PWMMODE_6,
   .Ctrl_Reg_02.ctrl02.OTW_REP      = true,                 // Over Temperature Warning will raise nFault
   .Ctrl_Reg_02.ctrl02.DIS_GDF      = false,
   .Ctrl_Reg_02.ctrl02.DIS_GDUV     = false,
#if (defined UOMODRI_CONFIG_ENABLE) && (!UOMODRI_CONFIG_ENABLE)
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
static DRV8353 drv_cfg[] =
{
 {
  .drvCfgHandler    = &DrvCfgList[MOTOR_1],
  .drvRegHandler    = &DrvRegList[MOTOR_1],
  .enableTimeOut    = true,
  .rxTimeOut        = true,
 },
 {
  .drvCfgHandler    = &DrvCfgList[MOTOR_2],
  .drvRegHandler    = &DrvRegList[MOTOR_2],
  .enableTimeOut    = true,
  .rxTimeOut        = true,
 },
};

static const HAL_MOTOR_CFG hal_motor_cfg[] =
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
  .p_ITAcq          = &AdcIntList[0],
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
  .p_ITAcq          = &AdcIntList[1],
 },
};

//#pragma DATA_SECTION(motor_foc,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(motor_foc, "MSGRAM_CPU_TO_CM");
static FOC_STRUCT motor_foc[] =
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
  .motor_acq.p_adcRegs[3]               = MOTOR12_VBUS_ADC_REGS,
  .motor_acq.p_adcResultRegs[0]         = MOTOR12_IVA_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[1]         = MOTOR12_IVB_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[2]         = MOTOR12_IVC_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[3]         = MOTOR12_VBUS_ADC_RESULT_REGS,
  .motor_acq.p_vaMeasReg                = (volatile uint16_t *)(MOTOR1_VA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vbMeasReg                = (volatile uint16_t *)(MOTOR1_VB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vcMeasReg                = (volatile uint16_t *)(MOTOR1_VC_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg              = (volatile uint16_t *)(MOTOR12_VBUS_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_iaMeasReg                = (volatile int16_t *)(MOTOR1_IA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_ibMeasReg                = (volatile int16_t *)(MOTOR1_IB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_icMeasReg                = (volatile int16_t *)(MOTOR1_IC_ADC_PPB_RESULT_ADDR),
  .motor_acq.va                         = 0.0,
  .motor_acq.vb                         = 0.0,
  .motor_acq.vc                         = 0.0,
  .motor_acq.vbus                       = 0.0,
  .motor_acq.ia                         = 0.0,
  .motor_acq.ib                         = 0.0,
  .motor_acq.ic                         = 0.0,
//  .motor_acq.measNew                    = false,
  .motor_acq.vBusLpf.a                  = MOTOR12_VBUS_LPF_ALPHA,
  .motor_acq.vBusLpf.one_minus_a        = MOTOR12_VBUS_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.p_qepHandle                = (&QepCfgList[MOTOR_1]),//(const eQEP_TypeDef*)&QepCfgList[MOTOR_1],
  .motor_enc.p_qepRegs                  = MOTOR1_QEP_VAR,
  .motor_enc.polePairs                  = MOTOR1_POLES_PAIRS,
  .motor_enc.thetaMech[NEW]             = 0.0,
  .motor_enc.thetaMech[OLD]             = 0.0,
  .motor_enc.thetaElec                  = 0.0,
  .motor_enc.thetaDir                   = 0,
  .motor_enc.thetaMechScaler            = (float32_t)(MOTOR1_ENC_RESOLUTION_SCALE),
  .motor_enc.indexCompensated           = false,
  .motor_enc.indexTheta                 = 0.0,
  .motor_enc.indexedThetaMech           = 0.0,
  .motor_enc.thetaAbsolute              = 0.0,
  .motor_enc.turnNb                     = 0,
  .motor_enc.indexDetected              = false,
  .motor_enc.indexToggle                = false,
  .motor_enc.rollOverErrorFlag          = false,
  .motor_enc.rollOverErrorFlagEnable    = false,
  .motor_enc.speedEst.speedHighScaler   = (float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speedEst.speedLowScaler    = (float32_t)MOTOR1_ENC_SPEED_LOW_SCALE,
  .motor_enc.speedEst.speedMech         = (float32_t)PWM_FREQ,
  .motor_enc.speedEst.speedRef          = (float32_t)MOTOR1_ENC_SPEED_CUTOFF_FREQ,
  .motor_enc.speedEst.lpf.a             = (float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speedEst.lpf.one_minus_a   = (float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA,
  //---------------------------------------------------------------------------
  .motor_align.state                    = ALIGN_COUNT_UP,
  .motor_align.cnt                      = 0,
  .motor_align.cntMax                   = PWM_FREQ,
  .motor_align.idAlignRef               = 0.5,// 500mA alignment current
  .motor_align.idAlign                  = 0.0,
  //---------------------------------------------------------------------------
  .iParkLpf.a                           = MOTOR1_CURRENT_LPF_ALPHA,
  .iParkLpf.one_minus_a                 = MOTOR1_CURRENT_LPF_ONE_M_ALPHA,
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
  .piId.in                              = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].id)),
  .piId.set                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].idRef)),
  .piId.err                             = 0.0,
  .piId.kp                              = (float32_t)MOTOR1_ID_REGULATOR_KP_COEF,
  .piId.ki                              = (float32_t)MOTOR1_ID_REGULATOR_KI_COEF,
  .piId.integral                        = 0.0,
  .piId.ff                              = 0.0,
  .piId.sat                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus)),
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.in                              = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].iq)),
  .piIq.set                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].iqRef)),
  .piIq.err                             = 0.0,
  .piIq.kp                              = (float32_t)MOTOR1_IQ_REGULATOR_KP_COEF,
  .piIq.ki                              = (float32_t)MOTOR1_IQ_REGULATOR_KI_COEF,
  .piIq.integral                        = 0.0,
  .piIq.ff                              = 0.0,
  .piIq.sat                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].motor_acq.vbus)),
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPos.inTheta                        = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].motor_enc.thetaAbsolute)),
  .pdPos.setTheta                       = 0.0,//((float32_t *)(&comInterface.positionRef[MOTOR_1])),
  .pdPos.errTheta                       = 0.0,
  .pdPos.inSpeed                        = 0.0,//((float32_t *)(&motor_foc[MOTOR_1].motor_enc.speedEst.speedMech)),
  .pdPos.setSpeed                       = 0.0,//((float32_t *)(&comInterface.velocityRef[MOTOR_1])),
  .pdPos.errSpeed                       = 0.0,
  .pdPos.kp                             = (float32_t)MOTOR1_SPEED_REGULATOR_KP_COEF,
  .pdPos.kd                             = (float32_t)MOTOR1_SPEED_REGULATOR_KD_COEF,
  .pdPos.derivative                     = 0.0,
  .pdPos.ff                             = 0.0,
  .pdPos.sat                            = (float32_t)MOTOR1_SPEED_REGULATOR_SATURATION,//((float32_t *)(&pdSat)),
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resistanceEst                        = 0.0,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstLpf.a                          = 0.0,
  .resEstLpf.one_minus_a                = 0.0,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
  //--- FOC controller enable to start? ---------------------------------------
//  .motor_enable                         = false,
//  .system_enable                        = false,
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
  .motor_acq.p_adcRegs[3]               = MOTOR12_VBUS_ADC_REGS,
  .motor_acq.p_adcResultRegs[0]         = MOTOR12_IVA_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[1]         = MOTOR12_IVB_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[2]         = MOTOR12_IVC_ADC_RESULT_REGS,
  .motor_acq.p_adcResultRegs[3]         = MOTOR12_VBUS_ADC_RESULT_REGS,
  .motor_acq.p_vaMeasReg                = (volatile uint16_t *)(MOTOR2_VA_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vbMeasReg                = (volatile uint16_t *)(MOTOR2_VB_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vcMeasReg                = (volatile uint16_t *)(MOTOR2_VC_ADC_PPB_RESULT_ADDR),
  .motor_acq.p_vBusMeasReg              = (volatile uint16_t *)(MOTOR12_VBUS_ADC_PPB_RESULT_ADDR),
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
  .motor_acq.vBusLpf.a                  = MOTOR12_VBUS_LPF_ALPHA,
  .motor_acq.vBusLpf.one_minus_a        = MOTOR12_VBUS_LPF_ONE_M_ALPHA,
  //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
  .motor_enc.p_qepHandle                = (&QepCfgList[MOTOR_2]),//(const eQEP_TypeDef*)&QepCfgList[MOTOR_2],
  .motor_enc.p_qepRegs                  = MOTOR2_QEP_VAR,
  .motor_enc.polePairs                  = MOTOR2_POLES_PAIRS,
  .motor_enc.thetaMech[NEW]             = 0.0,
  .motor_enc.thetaMech[OLD]             = 0.0,
  .motor_enc.thetaElec                  = 0.0,
  .motor_enc.thetaDir                   = 0,
  .motor_enc.thetaMechScaler            = (float32_t)(MOTOR2_ENC_RESOLUTION_SCALE),
  .motor_enc.indexCompensated           = false,
  .motor_enc.indexTheta                 = 0.0,
  .motor_enc.indexedThetaMech           = 0.0,
  .motor_enc.thetaAbsolute              = 0.0,
  .motor_enc.turnNb                     = 0,
  .motor_enc.indexDetected              = false,
  .motor_enc.indexToggle                = false,
  .motor_enc.rollOverErrorFlag          = false,
  .motor_enc.rollOverErrorFlagEnable    = false,
  .motor_enc.speedEst.speedHighScaler   = (float32_t)MOTOR2_ENC_SPEED_HIGH_SCALE,
  .motor_enc.speedEst.speedLowScaler    = (float32_t)MOTOR2_ENC_SPEED_LOW_SCALE,
  .motor_enc.speedEst.speedMech         = (float32_t)PWM_FREQ,
  .motor_enc.speedEst.speedRef          = (float32_t)MOTOR2_ENC_SPEED_CUTOFF_FREQ,
  .motor_enc.speedEst.lpf.a             = (float32_t)MOTOR2_ENC_SPEED_LPF_ALPHA,
  .motor_enc.speedEst.lpf.one_minus_a   = (float32_t)MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA,
  //---------------------------------------------------------------------------
  .motor_align.state                    = ALIGN_COUNT_UP,
  .motor_align.cnt                      = 0,
  .motor_align.cntMax                   = PWM_FREQ,
  .motor_align.idAlignRef               = 0.5,// 500mA alignment current
  .motor_align.idAlign                  = 0.0,
  //---------------------------------------------------------------------------
  .iParkLpf.a                           = MOTOR2_CURRENT_LPF_ALPHA,
  .iParkLpf.one_minus_a                 = MOTOR2_CURRENT_LPF_ONE_M_ALPHA,
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
  .piId.in                              = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].id)),
  .piId.set                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].idRef)),
  .piId.err                             = 0.0,
  .piId.kp                              = (float32_t)MOTOR2_ID_REGULATOR_KP_COEF,
  .piId.ki                              = (float32_t)MOTOR2_ID_REGULATOR_KI_COEF,
  .piId.integral                        = 0.0,
  .piId.ff                              = 0.0,
  .piId.sat                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus)),
  //--- FOC - Initialize PI controller (Iq) -----------------------------------
  .piIq.in                              = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].iq)),
  .piIq.set                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].iqRef)),
  .piIq.err                             = 0.0,
  .piIq.kp                              = (float32_t)MOTOR2_IQ_REGULATOR_KP_COEF,
  .piIq.ki                              = (float32_t)MOTOR2_IQ_REGULATOR_KI_COEF,
  .piIq.integral                        = 0.0,
  .piIq.ff                              = 0.0,
  .piIq.sat                             = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].motor_acq.vbus)),
  //--- FOC - Initialize PD controller (Speed & position) ---------------------
  .pdPos.inTheta                        = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].motor_enc.thetaAbsolute)),
  .pdPos.setTheta                       = 0.0,//((float32_t *)(&comInterface.positionRef[MOTOR_2])),
  .pdPos.errTheta                       = 0.0,
  .pdPos.inSpeed                        = 0.0,//((float32_t *)(&motor_foc[MOTOR_2].motor_enc.speedEst.speedMech)),
  .pdPos.setSpeed                       = 0.0,//((float32_t *)(&comInterface.velocityRef[MOTOR_2])),
  .pdPos.errSpeed                       = 0.0,
  .pdPos.kp                             = (float32_t)MOTOR2_SPEED_REGULATOR_KP_COEF,
  .pdPos.kd                             = (float32_t)MOTOR2_SPEED_REGULATOR_KD_COEF,
  .pdPos.derivative                     = 0.0,
  .pdPos.ff                             = 0.0,
  .pdPos.sat                            = (float32_t)MOTOR2_SPEED_REGULATOR_SATURATION,//((float32_t *)(&pdSat)),
  //--- FOC - Q-axis resistance estimation ------------------------------------
  .resistanceEst                        = 0.0,
  //--- Resistance estimation low-pass filter ---------------------------------
  .resEstLpf.a                          = 0.0,
  .resEstLpf.one_minus_a                = 0.0,
  //--- Anti-coagging structure -----------------------------------------------
  //acog;
  //--- FOC controller enable to start? ---------------------------------------
//  .motor_enable                         = false,
//  .system_enable                        = false,
 },
};

/** @var    MOTOR_STRUCT    motor[]
 *  @brief  General structure array configuration structure for the Hardware Abstraction Layer.
 */
static MOTOR_STRUCT motor[] =
{
 {
//  .motorID          = MOTOR_1,
  .p_motorHalCfg                    = &hal_motor_cfg[MOTOR_1],
  .p_motorDRV                       = &drv_cfg[MOTOR_1],
  .p_motorFOC                       = &motor_foc[MOTOR_1],
  .clCycleNb                        = 0,
  .interruptCnt                     = 0,
//  .motorAlign.state                 = ALIGN_COUNT_UP,
//  .motorAlign.cnt                   = 0,
//  .motorAlign.cntMax                = PWM_FREQ,
//  .motorAlign.idAlignRef            = 0.5,// 500mA alignment current
//  .motorAlign.idAlign               = 0.0,
  .statorResEst                     = 1.0,
  .statorResEstLpf.a                = MOTOR1_STATOR_RESISTOR_LPF_ALPHA,
  .statorResEstLpf.one_minus_a      = MOTOR1_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
//  .enabled          = false,
//  .currentScaler    = ADC_CURRENT_SCALE,
//  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
//  .voltageScaler    = ADC_VOLTAGE_SCALE,
//  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
  //  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
  //  .voltageScaler    = ADC_VOLTAGE_SCALE,
  //  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
  .p_motorChAReg                    = (volatile uint16_t *)(MOTOR1_PWM1_CMD_ADDR),
  .p_motorChBReg                    = (volatile uint16_t *)(MOTOR1_PWM2_CMD_ADDR),
  .p_motorChCReg                    = (volatile uint16_t *)(MOTOR1_PWM3_CMD_ADDR),
 },
 {
//  .motorID          = MOTOR_2,
  .p_motorHalCfg                    = &hal_motor_cfg[MOTOR_2],
  .p_motorDRV                       = &drv_cfg[MOTOR_2],
  .p_motorFOC                       = &motor_foc[MOTOR_2],
  .clCycleNb                        = 0,
  .interruptCnt                     = 0,
//  .motorAlign.state                 = ALIGN_COUNT_UP,
//  .motorAlign.cnt                   = 0,
//  .motorAlign.cntMax                = PWM_FREQ,
//  .motorAlign.idAlignRef            = 0.5,// 500mA alignment current
//  .motorAlign.idAlign               = 0.0,
  .statorResEst                     = 1.0,
  .statorResEstLpf.a                = MOTOR2_STATOR_RESISTOR_LPF_ALPHA,
  .statorResEstLpf.one_minus_a      = MOTOR2_STATOR_RESISTOR_LPF_ONE_M_ALPHA,
//  .enabled          = false,
//  .currentScaler    = ADC_CURRENT_SCALE,
//  .currentScalerReg = ADC_CURRENT_SCALE / 0x8000L,
//  .voltageScaler    = ADC_VOLTAGE_SCALE,
//  .voltageScalerReg = ADC_VOLTAGE_SCALE / 0x10000L,
  .p_motorChAReg                    = (volatile uint16_t *)(MOTOR2_PWM1_CMD_ADDR),
  .p_motorChBReg                    = (volatile uint16_t *)(MOTOR2_PWM2_CMD_ADDR),
  .p_motorChCReg                    = (volatile uint16_t *)(MOTOR2_PWM3_CMD_ADDR),
 },
};

#endif  /*__UOMODRI_CONFIG_HANDLERS_H__*/
