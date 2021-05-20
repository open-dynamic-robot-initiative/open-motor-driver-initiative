#ifndef __UOMODRI_CONFIG_HANDLERS_H__
#define __UOMODRI_CONFIG_HANDLERS_H__

// drivers
#include "device.h"
#include "driverlib.h"

#include "hal.h"
#include "drv8353.h"
#include "foc.h"
#include "spi_communication.h"

#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"

// Communication dual buffers for initialization
extern DualBufferW dualBufferW;
extern DualBufferR dualBufferR;

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
   .Ctrl_Reg_07.ctrl07.CAL_MODE     = true},
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
   .Ctrl_Reg_07.ctrl07.CAL_MODE     = true},
};

/** @var    DRV8353     drv
 *  @brief  General DRV8353 configuration structure.
 */
static DRV8353 drv =
{.drvCfgHandler = DrvCfgList,
 .drvRegHandler = DrvRegList,
 .enableTimeOut = true,
 .rxTimeOut     = true
};

static MOTOR_HAL_CFG motor[] =
{
 {.motorID      = MOTOR_1,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[0], &PwmCounterCompareCfgList[1], &PwmCounterCompareCfgList[2]},
  .p_iConv      = {&AdcPPBsList[0], &AdcPPBsList[4], &AdcPPBsList[10]},
  .p_vConv      = {&AdcPPBsList[2], &AdcPPBsList[6], &AdcPPBsList[12]},
  .p_vbusConv   = &AdcPPBsList[8],
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
  .dataValid    = {false, false},
  //  .offsetCal      = true
 },
 {.motorID      = MOTOR_2,
  .p_pwmCntCmp  = {&PwmCounterCompareCfgList[3], &PwmCounterCompareCfgList[4], &PwmCounterCompareCfgList[5]},
  .p_iConv      = {&AdcPPBsList[1], &AdcPPBsList[5], &AdcPPBsList[11]},
  .p_vConv      = {&AdcPPBsList[3], &AdcPPBsList[7], &AdcPPBsList[13]},
  .p_vbusConv   = &AdcPPBsList[9],
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
  .dataValid    = {false, false},
  //  .offsetCal      = true
 },
 {.motorID      = NO_MOT,
 }
};

static FOC foc[] =
{
 {.enabled = true}

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
