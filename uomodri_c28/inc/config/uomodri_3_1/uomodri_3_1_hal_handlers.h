/*
 * File name: uomodri_3_1_hal_handlers.h
 * Description: Header file containing the uomodri V3 structure for hal config.
 */

#ifndef __UOMODRI_3_1_HAL_HANDLERS_H__
#define __UOMODRI_3_1_HAL_HANDLERS_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
//#include "f2838x_cla_typedefs.h"
//#include "driverlib.h"
//#include "f2838x_device.h"
//#include "/home/jmanhes/ti/c2000/C2000Ware_5_02_00_00/device_support/f2838x/common/include/device.h"
//#include <board.h>

//#include "uomodri_3_1_hal_defines.h"

//#include "../uomodri_user_defines.h"
//#include "../../communication.h"
//#include "../../hal.h"

//#include "uomodri_user_defines.h"
//#include "hal.h"
//#include "communication.h"

//#include "uomodri_3_1_hal_defines.h"
#include "hal.h"
#include "communication.h"
#include "../../uomodri_shared.h"
#include "uomodri_3_1_hal_defines.h"


/***********************************************************************
 * VARIABLES
 ***********************************************************************/
extern spi_rx_t     com_spi_rx;
extern spi_tx_t     com_spi_tx;
extern uint16_t     led_spi_tx[LED_MSG_TX_16BIT_LENGTH];

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void adcCalib_Task(void);
__attribute__((interrupt)) void cmdNew_Task(void);
__attribute__((interrupt)) void motor1_Task(void);
#if (USE_DUAL_AXIS_CONFIG == true)
__attribute__((interrupt)) void motor2_Task(void);
#endif
__attribute__((interrupt)) void Cla1Task5(void);
__attribute__((interrupt)) void Cla1Task6(void);
__attribute__((interrupt)) void Cla1Task7(void);
__attribute__((interrupt)) void Cla1Task8(void);

/***********************************************************************
 * GPIOs STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    GPIO_TypeDef GpioPinsCfgList[]
 *  @brief  List all the GPIO definitions except the analog pins.
 */
static const gpio_cfg_t GpioPinsCfgList[] =
{
#if (USB_BUS_ENABLE) && (!UART_BUS_ENABLE) && (CM_CORE_ENABLE)
 /*
  * USB Pins
  */
 {
  .pinNum           = USB_M_NUM,
  .fctMux           = USB_M_CFG,
  .pinType          = GPIO_PIN_TYPE_STD,
  .coreSelect       = GPIO_CORE_CM,
  .analogModeEnable = GPIO_ANALOG_ENABLED
 },
 {
  .pinNum           = USB_P_NUM,
  .fctMux           = USB_P_CFG,
  .pinType          = GPIO_PIN_TYPE_STD,
  .coreSelect       = GPIO_CORE_CM,
  .analogModeEnable = GPIO_ANALOG_ENABLED
 },
#elif (!USB_BUS_ENABLE) && (UART_BUS_ENABLE) && (CM_CORE_ENABLE)
 /*
  * UART Pins
  */
 {
  .pinNum           = UART_ON_USB_TX_NUM,
  .fctMux           = UART_ON_USB_TX_CFG,
  .pinType          = GPIO_PIN_TYPE_STD,
  .coreSelect       = GPIO_CORE_CM,
  .direction        = GPIO_DIR_MODE_OUT,
  .samplingInMode   = GPIO_QUAL_ASYNC,
  .setValue         = 0,
  .intEnable        = false,
 },
 {
  .pinNum           = UART_ON_USB_RX_NUM,
  .fctMux           = UART_ON_USB_RX_CFG,
  .pinType          = GPIO_PIN_TYPE_STD,
  .coreSelect       = GPIO_CORE_CM,
  .direction        = GPIO_DIR_MODE_IN,
  .samplingInMode   = GPIO_QUAL_ASYNC,
  .setValue         = 0,
  .intEnable        = false,
 },
#endif
 /* PWM MOTOR_1 configuration pins */
 MOTOR1_PWM1_CHA_DEF,
 MOTOR1_PWM1_CHB_DEF,
 MOTOR1_PWM2_CHA_DEF,
 MOTOR1_PWM2_CHB_DEF,
 MOTOR1_PWM3_CHA_DEF,
 MOTOR1_PWM3_CHB_DEF,
 /* PWM MOTOR_2 configuration pins */
 MOTOR2_PWM1_CHA_DEF,
 MOTOR2_PWM1_CHB_DEF,
 MOTOR2_PWM2_CHA_DEF,
 MOTOR2_PWM2_CHB_DEF,
 MOTOR2_PWM3_CHA_DEF,
 MOTOR2_PWM3_CHB_DEF,
 /* Quadrature encoder #1 */
 MOTOR1_ENC_CHA_DEF,
 MOTOR1_ENC_CHB_DEF,
 MOTOR1_ENC_CHI_DEF,
 /* Quadrature encoder #2 */
 MOTOR2_ENC_CHA_DEF,
 MOTOR2_ENC_CHB_DEF,
 MOTOR2_ENC_CHI_DEF,
 /* DRV communication */
 DRV_SPI_SIMO_DEF,
 DRV_SPI_SOMI_DEF,
 DRV_SPI_CLK_DEF,
 /* DRV1 configuration pins */
 MOTOR1_DRV_EN_DEF,
 MOTOR1_DRV_SPI_CS_DEF,
 MOTOR1_DRV_FAULT_DEF,
 /* DRV2 configuration pins */
 MOTOR2_DRV_EN_DEF,
 MOTOR2_DRV_SPI_CS_DEF,
 MOTOR2_DRV_FAULT_DEF,
 /* OMODRI communication */
 COM_SPI_SIMO_DEF,
 COM_SPI_SOMI_DEF,
 COM_SPI_CLK_DEF,
 COM_SPI_CS_DEF,
 /* RGB LED communication */
 LED_SPI_SIMO_DEF,
 /* RS485 BUS IO pins */
 RS485_TX_DEF,
 RS485_RX_DEF,
 CAN_SHDN_EN_DEF,
 CAN_RS485_MUX_SEL_DEF,
 RS485_TX_EN_DEF,
 /* Debug pins */
 DBG_PIN3_DEF,
 DBG_PIN2_DEF,
 DBG_PIN1_DEF,
 DBG_PIN0_DEF,
 /* END OF ARRAY */
 {.pinNum = UINT32_MAX},
};

/***********************************************************************
 * ADC STRUCTURES DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    ADC_Init_TypeDef AdcInitList[]
 *  @brief  List all the configurations defined
 *  for ADC Start-Of-Conversion (SOC) submodule.
 */
static const adc_ini_t  AdcInitList[] =
{
 /* ADCA - Ia_M1, Ia_M2 */
 MOTOR1_IA_ADC_INIT_DEF,
 MOTOR2_IA_ADC_INIT_DEF,
 /* ADCB - Ib_M1, Ib_M2 */
 MOTOR1_IB_ADC_INIT_DEF,
 MOTOR2_IB_ADC_INIT_DEF,
 /* ADCC - Ic_M1, Ic_M2 */
 MOTOR1_IC_ADC_INIT_DEF,
 MOTOR2_IC_ADC_INIT_DEF,
 /* ADCD - VBUS, VEXT1, VEXT2 */
 VBUS_ADC_INIT_DEF,
 VEXT1_ADC_INIT_DEF,
 VEXT2_ADC_INIT_DEF,
 /* END OF ARRAY */
 {.adcBase = UINT32_MAX},
};

/** @var    ADC_PPB_TypeDef AdcAcqList[]
 *  @brief  List all the configurations defined for ADC Post-Processing Block (PPB) submodule.
 */
static const adc_acq_t  AdcAcqList[] =
{
 /* ADCA - Ia_M1, Ia_M2 */
 MOTOR1_IA_ADC_ACQ_DEF,
 MOTOR2_IA_ADC_ACQ_DEF,
 /* ADCB - Ib_M1, Ib_M2 */
 MOTOR1_IB_ADC_ACQ_DEF,
 MOTOR2_IB_ADC_ACQ_DEF,
 /* ADCC - Ic_M1, Ic_M2 */
 MOTOR1_IC_ADC_ACQ_DEF,
 MOTOR2_IC_ADC_ACQ_DEF,
 /* ADCD - VBUS, VEXT1, VEXT2 */
 VBUS_ADC_ACQ_DEF,
 VEXT1_ADC_ACQ_DEF,
 VEXT2_ADC_ACQ_DEF,
 /* END OF ARRAY */
 {.adcBase = UINT32_MAX}
};

/** @var    ADC_TypeDef AdcCfgList[]
 *  @brief  List all the configurations defined for each ADC modules.
 */
static const adc_cfg_t  AdcCfgList[] =
{
 /* ADCA - Ia_M1, Ia_M2 */
 MOTOR12_IA_ADC_DEF,
 /* ADCB - Ib_M1, Ib_M2 */
 MOTOR12_IB_ADC_DEF,
 /* ADCC - Ic_M1, Ic_M2 */
 MOTOR12_IC_ADC_DEF,
 /* ADCD - VBUS, VEXT1, VEXT2 */
 VBUS_VEXT12_ADC_DEF,
 /* END OF ARRAY */
 {.adcBase = UINT32_MAX},
};

/***********************************************************************
 * PWM STRUCTURES DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    ePWM_TB_TypeDef PwmTimeBaseCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Time Base submodule.
 */
static const epwm_tb_t  PwmTimeBaseCfgList[] =
{
 /* MOTOR1 - PWM1/PWM2/PWM3 - Time Base config */
 MOTOR1_PWM1_TIME_BASE_DEF,
 MOTOR1_PWM2_TIME_BASE_DEF,
 MOTOR1_PWM3_TIME_BASE_DEF,
 /* MOTOR2 - PWM4/PWM5/PWM6 - Time Base config */
 MOTOR2_PWM1_TIME_BASE_DEF,
 MOTOR2_PWM2_TIME_BASE_DEF,
 MOTOR2_PWM3_TIME_BASE_DEF,
 /* END OF ARRAY */
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_CC_TypeDef PwmCounterCompareCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Capture/Compare submodule.
 */
static const epwm_cc_t  PwmCounterCompareCfgList[] =
{
 /* MOTOR1 - PWM1/PWM2/PWM3 - Capture/Compare config */
 MOTOR1_PWM1_CAPTURE_COMPARE_DEF,
 MOTOR1_PWM2_CAPTURE_COMPARE_DEF,
 MOTOR1_PWM3_CAPTURE_COMPARE_DEF,
 /* MOTOR2 - PWM4/PWM5/PWM6 - Capture/Compare config */
 MOTOR2_PWM1_CAPTURE_COMPARE_DEF,
 MOTOR2_PWM2_CAPTURE_COMPARE_DEF,
 MOTOR2_PWM3_CAPTURE_COMPARE_DEF,
 /* END OF ARRAY */
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_AQ_TypeDef PwmActionQualifierCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Action Qualifier submodule.
 */
static const epwm_aq_t  PwmActionQualifierCfgList[] =
{
 /* MOTOR1 - PWM1/PWM2/PWM3 - Action Qualifier config */
 MOTOR1_PWM1_ACTION_QUALIFIER_DEF,
 MOTOR1_PWM2_ACTION_QUALIFIER_DEF,
 MOTOR1_PWM3_ACTION_QUALIFIER_DEF,
 /* MOTOR2 - PWM4/PWM5/PWM6 - Action Qualifier config */
 MOTOR2_PWM1_ACTION_QUALIFIER_DEF,
 MOTOR2_PWM2_ACTION_QUALIFIER_DEF,
 MOTOR2_PWM3_ACTION_QUALIFIER_DEF,
 /* END OF ARRAY */
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_DB_TypeDef PwmDeadBandCfgList[]
 *  @brief  List all the configurations necessaries for ePWM DeadBand Unit submodule.
 */
static const epwm_db_t  PwmDeadBandCfgList[] =
{
 /* MOTOR1 - PWM1/PWM2/PWM3 - Dead Band config */
 MOTOR1_PWM1_DEAD_BAND_DEF,
 MOTOR1_PWM2_DEAD_BAND_DEF,
 MOTOR1_PWM3_DEAD_BAND_DEF,
 /* MOTOR2 - PWM4/PWM5/PWM6 - Dead Band config */
 MOTOR2_PWM1_DEAD_BAND_DEF,
 MOTOR2_PWM2_DEAD_BAND_DEF,
 MOTOR2_PWM3_DEAD_BAND_DEF,
 /* END OF ARRAY */
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_ET_TypeDef PwmAdcSocEventTriggerCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Event Trigger submodule.
 */
static const epwm_et_t  PwmAdcSocEventTriggerCfgList[] =
{
 /* MOTOR1 - PWM1 - ADC SOC event sync config  */
 MOTOR1_PWM1_ADC_SOC_EVT_TRIGGER_DEF,
 /* MOTOR2 - PWM4 - ADC SOC event sync config  */
 MOTOR2_PWM1_ADC_SOC_EVT_TRIGGER_DEF,
 /* END OF ARRAY */
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_TypeDef PwmCfgList[]
 *  @brief  List all the configurations necessaries for PWM module.
 */
static const epwm_cfg_t PwmCfgList[] =
{
 /* MOTOR1 - PWM1/PWM2/PWM3 - Global config */
 MOTOR1_PWM1_DEF,
 MOTOR1_PWM2_DEF,
 MOTOR1_PWM3_DEF,
 /* MOTOR2 - PWM4/PWM5/PWM6 - Global config */
 MOTOR2_PWM1_DEF,
 MOTOR2_PWM2_DEF,
 MOTOR2_PWM3_DEF,
 /* END OF ARRAY */
 {.epwmBase = UINT32_MAX},
};

/***********************************************************************
 * ENCODER STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    eQEP_TypeDef QepCfgList[]
 *  @brief  List all the configurations defined for each Quadrature Encoders modules.
 */
static const eqep_cfg_t QepCfgList[] =
{
 MOTOR1_QEP_DEF,
 MOTOR2_QEP_DEF,
 /* END OF ARRAY */
 {.eqepBase = UINT32_MAX},
};

/***********************************************************************
 * SCI/UART (RS485) STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const sci_cfg_t  sciCfgList[] =
{
 COM_SCI_DEF,
 /* END OF ARRAY */
 {.sciBase = UINT32_MAX},
};

/***********************************************************************
 * SPI STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    SPI_TypeDef spiCfgList[]
 *  @brief  List all the configurations defined for each SPI communication modules.
 */
static const spi_cfg_t  spiCfgList[] =
{
 DRV_SPI_DEF,
 COM_SPI_DEF,
 DBG_SPI_DEF,
 LED_SPI_DEF,
 /* END OF ARRAY */
 {.spiBase = UINT32_MAX},
};

/***********************************************************************
 * DMA STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    DMA_TypeDef dmaCfgList[]
 *  @brief  List all the DMA used for communication.
 */
static const dma_cfg_t  dmaCfgList[] =
{
 /* DMA for the RGB LED */
 DMA_LED_SPI_TX_DEF,
 /* DMA for SPI communication Tx */
 DMA_COM_SPI_TX_DEF,
 /* DMA for SPI communication Rx */
 DMA_COM_SPI_RX_DEF,
 /* END OF ARRAY */
 {.dmaChBase = UINT32_MAX},
};

/***********************************************************************
 * CLA STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const cla_cfg_t  claCfgList[] =
{
 /* CLA structure for ADC startup calibration - One Call */
 CLA_ADC_CALIB_DEF,
 /* CLA structure for uOmodri command update - 1ms */
 CLA_CMD_NEW_DEF,
 /* CLA structure for MOTOR1 control - 20us (40kHz) */
 CLA_MOTOR1_DEF,
 /* CLA structure for MOTOR2 control - 20us (40kHz) */
 CLA_MOTOR2_DEF,
 /* END OF ARRAY */
 {.p_claFunc = NULL},
};

/***********************************************************************
 * CLB STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const clb_cfg_t  clbCfgList[] =
{
 /* CLB config for RS485 reception */
 CLB_RS485_DEF,
};

/***********************************************************************
 * IPC STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const ipc_cfg_t  ipcCfgList[] =
{
 /* IPC between CPU1 & CM */
 IPC_CPU1_2_CM_DEF,
  /* END OF ARRAY */
 {.ipcType = IPC_TOTAL_NUM},
};

/***********************************************************************
 * CPU TIMER STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const tmr_cfg_t  timerCfgList[] =
{
 /* CPU timer 0 config */
 CPU_TIMER0_DEF,
 /* END OF ARRAY */
 {.cpuTimerBase = UINT32_MAX},
};

/***********************************************************************
 * GLOBALE HAL STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const hal_cfg_t hal =
{
 .p_gpioHandle  = GpioPinsCfgList,
 .p_adcHandle   = AdcCfgList,
 .p_epwmHandle  = PwmCfgList,
 .p_eqepHandle  = QepCfgList,
 .p_dmaHandle   = dmaCfgList,
 .p_ipcHandle   = ipcCfgList,
 .p_sciHandle   = sciCfgList,
 .p_spiHandle   = spiCfgList,
 .p_timerHandle = timerCfgList,
 .p_claHandle   = claCfgList,
 .p_clbHandle   = clbCfgList,
};

/** @var    drv_cfg_t   drv_cfg[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
#pragma DATA_SECTION(drv_cfg, "ramgs0")
static const drv_cfg_t  drv_cfg[] =
{
 {
  .spiHandle                = DRV_SPI_BASE,
  .gpioNumber_CS            = MOTOR1_DRV_SPI_CS_NUM,
  .gpioNumber_EN            = MOTOR1_DRV_EN_NUM,
  .gpioNumber_FAULT         = MOTOR1_DRV_FAULT_NUM,
 },
 {
  .spiHandle                = DRV_SPI_BASE,
  .gpioNumber_CS            = MOTOR2_DRV_SPI_CS_NUM,
  .gpioNumber_EN            = MOTOR2_DRV_EN_NUM,
  .gpioNumber_FAULT         = MOTOR2_DRV_FAULT_NUM,
 },
#endif
};

/** @var    drv_reg_t   drv_reg[]
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

#endif /* __UOMODRI_3_1_HAL_HANDLERS_H__ */
