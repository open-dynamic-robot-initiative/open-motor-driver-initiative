/*
 * File name: uomodri_2_handlers.h
 * Description: Header file containing the uomodri V3 structure for hal config.
 */

#ifndef __UOMODRI_2_HANDLERS_H__
#define __UOMODRI_2_HANDLERS_H__

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "f2838x_device.h"              // F2838x headerfile include file
#include "device.h"                     // F2838x Headerfile include File

#include "../../uomodri_user_defines.h"
#include "../../communication.h"
#include "../../drv8353.h"
#include "../../main.h"
#include "../../hal.h"

#include "uomodri_2_hal_defines.h"    // uOmodri v2.x configuration include file

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
extern spi_rx_t     com_spi_rx;
extern spi_tx_t     com_spi_tx;
extern uint16_t     led_spi_tx[LED_MSG_TX_16BIT_LENGTH];

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
extern __attribute__((interrupt)) void adcCalib_Task(void);
extern __attribute__((interrupt)) void cmdNew_Task(void);
extern __attribute__((interrupt)) void motor1_Task(void);
extern __attribute__((interrupt)) void motor2_Task(void);
extern __attribute__((interrupt)) void Cla1Task5(void);
extern __attribute__((interrupt)) void Cla1Task6(void);
extern __attribute__((interrupt)) void Cla1Task7(void);
extern __attribute__((interrupt)) void Cla1Task8(void);

/***********************************************************************
 * GPIOs STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    GPIO_TypeDef GpioPinsCfgList[]
 *  @brief  List all the GPIO definitions except the analog pins.
 */
static const gpio_cfg_t gpioPinsCfgList[] =
{
#if (USB_BUS_ENABLE) && (!UART_BUS_ENABLE) && (!defined CPU1)
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
#elif (!USB_BUS_ENABLE) && (UART_BUS_ENABLE) && (!defined CPU1)
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
  MOTOR1_PWM1_CHA_DEF,               /* MOTOR_1 - PWM1 ChA pin */
  MOTOR1_PWM1_CHB_DEF,               /* MOTOR_1 - PWM1 ChB pin */
  MOTOR1_PWM2_CHA_DEF,               /* MOTOR_1 - PWM2 ChA pin */
  MOTOR1_PWM2_CHB_DEF,               /* MOTOR_1 - PWM2 ChB pin */
  MOTOR1_PWM3_CHA_DEF,               /* MOTOR_1 - PWM3 ChA pin */
  MOTOR1_PWM3_CHB_DEF,               /* MOTOR_1 - PWM3 ChB pin */
  /* PWM MOTOR_2 configuration pins */
  MOTOR2_PWM1_CHA_DEF,               /* MOTOR_2 - PWM1 ChA pin */
  MOTOR2_PWM1_CHB_DEF,               /* MOTOR_2 - PWM1 ChB pin */
  MOTOR2_PWM2_CHA_DEF,               /* MOTOR_2 - PWM2 ChA pin */
  MOTOR2_PWM2_CHB_DEF,               /* MOTOR_2 - PWM2 ChB pin */
  MOTOR2_PWM3_CHA_DEF,               /* MOTOR_2 - PWM3 ChA pin */
  MOTOR2_PWM3_CHB_DEF,               /* MOTOR_2 - PWM3 ChA pin */
 /* Quadrature encoder #1 */
 MOTOR1_ENC_CHA_DEF,
 MOTOR1_ENC_CHB_DEF,
 MOTOR1_ENC_CHI_DEF,
 /* Quadrature encoder #2 */
 MOTOR2_ENC_CHA_DEF,
 MOTOR2_ENC_CHB_DEF,
 MOTOR2_ENC_CHI_DEF,
 /* DRV communication */
 DRV_SPI_SIMO_DEF,                  /* DRV SPI SIMO (MCU -> DRV) pin */
 DRV_SPI_SOMI_DEF,                  /* DRV SPI SOMI (DRV -> MCU) pin */
 DRV_SPI_CLK_DEF,                   /* DRV SPI SCLK (MCU -> DRV) pin */
 MOTOR1_DRV_SPI_CS_DEF,             /* MOTOR1 - DRV SPI CS\ (MCU -> DRV) pin */
 MOTOR2_DRV_SPI_CS_DEF,             /* MOTOR2 - DRV SPI CS\ (MCU -> DRV) pin */
 /* DRV1 configuration pins */
 MOTOR1_DRV_EN_DEF,                 /* MOTOR1 - DRV enable pin */
 MOTOR1_DRV_FAULT_DEF,              /* MOTOR1 - DRV fault pin */
 /* DRV2 configuration pins */
 MOTOR2_DRV_EN_DEF,                 /* MOTOR2 - DRV enable pin */
 MOTOR2_DRV_FAULT_DEF,              /* MOTOR2 - DRV fault pin */
 /* OMODRI communication */
 COM_SPI_SIMO_DEF,                  /* COM SPI SIMO (MASTER -> MCU) pin */
 COM_SPI_SOMI_DEF,                  /* COM SPI SOMI (MCU -> MASTER) pin */
 COM_SPI_CLK_DEF,                   /* COM SPI SCLK (MASTER -> MCU) pin */
 COM_SPI_CS_DEF,                    /* COM SPI CS\  (MASTER -> MCU) pin */
 /* RGB LED communication */
 LED_SPI_SIMO_DEF,                  /* LED SPI SIMO (MCU -> LED) pin */
 /* RS485 BUS IO pins */
 RS485_TX_DEF,                      /* RS485 Tx pin */
 RS485_RX_DEF,                      /* RS485 Rx pin */
 RS485_TX_EN_DEF,                   /* RS485 Tx enable pin */
 RS485_RX_EN_DEF,                   /* RS485 Rx enable pin */
 /* Debug pins */
 DBG_PIN3_DEF,                      /* Debug pin 3 */
 DBG_PIN2_DEF,                      /* Debug pin 2 */
 DBG_PIN1_DEF,                      /* Debug pin 1 */
 DBG_PIN0_DEF,                      /* Debug pin 0 */
 {.pinNum = UINT32_MAX},            /* END OF ARRAY */
};

/***********************************************************************
 * ADC STRUCTURES DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    ADC_Init_TypeDef AdcInitList[]
 *  @brief  List all the configurations defined
 *  for ADC Start-Of-Conversion (SOC) submodule.
 */
static const adc_ini_t  adcInitList[] =
{
 MOTOR1_IA_ADC_INIT_DEF,            /* ADCA - M1_Ia */
 MOTOR2_IA_ADC_INIT_DEF,            /* ADCA - M2_Ia */
 MOTOR1_IB_ADC_INIT_DEF,            /* ADCB - M1_Ib */
 MOTOR2_IB_ADC_INIT_DEF,            /* ADCB - M2_Ib */
 MOTOR1_IC_ADC_INIT_DEF,            /* ADCC - M1_Ic */
 MOTOR2_IC_ADC_INIT_DEF,            /* ADCC - M2_Ic */
 VBUS_ADC_INIT_DEF,                 /* ADCD - VBUS */
 VEXT1_ADC_INIT_DEF,                /* ADCD - VEXT1 */
 VEXT2_ADC_INIT_DEF,                /* ADCD - VEXT2 */
 {.adcBase = UINT32_MAX},           /* END OF ARRAY */
};

/** @var    ADC_PPB_TypeDef AdcAcqList[]
 *  @brief  List all the configurations defined for ADC Post-Processing Block (PPB) submodule.
 */
static const adc_acq_t  adcAcqList[] =
{
 MOTOR1_IA_ADC_ACQ_DEF,             /* ADCA - M1_Ia */
 MOTOR2_IA_ADC_ACQ_DEF,             /* ADCA - M2_Ia */
 MOTOR1_IB_ADC_ACQ_DEF,             /* ADCB - M1_Ib */
 MOTOR2_IB_ADC_ACQ_DEF,             /* ADCB - M2_Ib */
 MOTOR1_IC_ADC_ACQ_DEF,             /* ADCC - M1_Ic */
 MOTOR2_IC_ADC_ACQ_DEF,             /* ADCC - M2_Ic */
 VBUS_ADC_ACQ_DEF,                  /* ADCD - VBUS */
 VEXT1_ADC_ACQ_DEF,                 /* ADCD - VEXT1 */
 VEXT2_ADC_ACQ_DEF,                 /* ADCD - VEXT2 */
 {.adcBase = UINT32_MAX}            /* END OF ARRAY */
};

/** @var    ADC_IT_TypeDef AdcIntList[]
 *  @brief  List all the interrupts source generated by ADC modules.
 */
static const adc_int_t  AdcIntList[] =
{
 MOTOR1_IA_ADC_INT_DEF,             /* M1 - ADCA as IT source */
 MOTOR2_IA_ADC_INT_DEF,             /* M2 - ADCA as IT source */
 {.adcBase = UINT32_MAX}            /* END OF ARRAY */
};

/** @var    ADC_TypeDef AdcCfgList[]
 *  @brief  List all the configurations defined for each ADC modules.
 */
static const adc_cfg_t  adcCfgList[] =
{
 MOTOR12_IA_ADC_DEF,                /* ADCA - M1_Ia, M2_Ia */
 MOTOR12_IB_ADC_DEF,                /* ADCB - M1_Ib, M2_Ib */
 MOTOR12_IC_ADC_DEF,                /* ADCC - M1_Ic, M2_Ic */
 VBUS_VEXT12_ADC_DEF,               /* ADCD - VBUS, VEXT1, VEXT2 */
 {.adcBase = UINT32_MAX},           /* END OF ARRAY */
};

/***********************************************************************
 * PWM STRUCTURES DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    ePWM_TB_TypeDef PwmTimeBaseCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Time Base submodule.
 */
static const epwm_tb_t  pwmTimeBaseCfgList[] =
{
 MOTOR1_PWM1_TIME_BASE_DEF,         /* M1 - PWM1 - Time Base config */
 MOTOR1_PWM2_TIME_BASE_DEF,         /* M1 - PWM2 - Time Base config */
 MOTOR1_PWM3_TIME_BASE_DEF,         /* M1 - PWM3 - Time Base config */
 MOTOR2_PWM1_TIME_BASE_DEF,         /* M2 - PWM4 - Time Base config */
 MOTOR2_PWM2_TIME_BASE_DEF,         /* M2 - PWM5 - Time Base config */
 MOTOR2_PWM3_TIME_BASE_DEF,         /* M2 - PWM6 - Time Base config */
 {.epwmBase = UINT32_MAX},          /* END OF ARRAY */
};

/** @var    ePWM_CC_TypeDef PwmCounterCompareCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Capture/Compare submodule.
 */
static const epwm_cc_t  pwmCounterCompareCfgList[] =
{
 MOTOR1_PWM1_CAPTURE_COMPARE_DEF,   /* M1 - PWM1 - Capture/Compare config */
 MOTOR1_PWM2_CAPTURE_COMPARE_DEF,   /* M1 - PWM2 - Capture/Compare config */
 MOTOR1_PWM3_CAPTURE_COMPARE_DEF,   /* M1 - PWM3 - Capture/Compare config */
 MOTOR2_PWM1_CAPTURE_COMPARE_DEF,   /* M2 - PWM4 - Capture/Compare config */
 MOTOR2_PWM2_CAPTURE_COMPARE_DEF,   /* M2 - PWM5 - Capture/Compare config */
 MOTOR2_PWM3_CAPTURE_COMPARE_DEF,   /* M2 - PWM6 - Capture/Compare config */
 {.epwmBase = UINT32_MAX},          /* END OF ARRAY */
};

/** @var    ePWM_AQ_TypeDef PwmActionQualifierCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Action Qualifier submodule.
 */
static const epwm_aq_t  pwmActionQualifierCfgList[] =
{
 MOTOR1_PWM1_ACTION_QUALIFIER_DEF,  /* M1 - PWM1 - Action Qualifier config */
 MOTOR1_PWM2_ACTION_QUALIFIER_DEF,  /* M1 - PWM2 - Action Qualifier config */
 MOTOR1_PWM3_ACTION_QUALIFIER_DEF,  /* M1 - PWM3 - Action Qualifier config */
 MOTOR2_PWM1_ACTION_QUALIFIER_DEF,  /* M2 - PWM4 - Action Qualifier config */
 MOTOR2_PWM2_ACTION_QUALIFIER_DEF,  /* M2 - PWM5 - Action Qualifier config */
 MOTOR2_PWM3_ACTION_QUALIFIER_DEF,  /* M2 - PWM6 - Action Qualifier config */
 {.epwmBase = UINT32_MAX},          /* END OF ARRAY */
};

/** @var    ePWM_DB_TypeDef PwmDeadBandCfgList[]
 *  @brief  List all the configurations necessaries for ePWM DeadBand Unit submodule.
 */
static const epwm_db_t  pwmDeadBandCfgList[] =
{
 MOTOR1_PWM1_DEAD_BAND_DEF,         /* M1 - PWM1 - Dead Band config */
 MOTOR1_PWM2_DEAD_BAND_DEF,         /* M1 - PWM2 - Dead Band config */
 MOTOR1_PWM3_DEAD_BAND_DEF,         /* M1 - PWM3 - Dead Band config */
 MOTOR2_PWM1_DEAD_BAND_DEF,         /* M2 - PWM4 - Dead Band config */
 MOTOR2_PWM2_DEAD_BAND_DEF,         /* M2 - PWM5 - Dead Band config */
 MOTOR2_PWM3_DEAD_BAND_DEF,         /* M2 - PWM6 - Dead Band config */
 {.epwmBase = UINT32_MAX},          /* END OF ARRAY */
};

/** @var    ePWM_ET_TypeDef PwmAdcSocEventTriggerCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Event Trigger submodule.
 */
static const epwm_et_t  pwmAdcSocEventTriggerCfgList[] =
{
 MOTOR1_PWM1_ADC_SOC_EVT_TRIG_DEF,  /* M1 - PWM1 - ADC SOC event sync config  */
 MOTOR2_PWM1_ADC_SOC_EVT_TRIG_DEF,  /* M2 - PWM4 - ADC SOC event sync config  */
 {.epwmBase = UINT32_MAX},          /* END OF ARRAY */
};

/** @var    ePWM_TypeDef PwmCfgList[]
 *  @brief  List all the configurations necessaries for PWM module.
 */
static const epwm_cfg_t pwmCfgList[] =
{
 MOTOR1_PWM1_DEF,                   /* M1 - PWM1 - Global config */
 MOTOR1_PWM2_DEF,                   /* M1 - PWM2 - Global config */
 MOTOR1_PWM3_DEF,                   /* M1 - PWM3 - Global config */
 MOTOR2_PWM1_DEF,                   /* M2 - PWM4 - Global config */
 MOTOR2_PWM2_DEF,                   /* M2 - PWM5 - Global config */
 MOTOR2_PWM3_DEF,                   /* M2 - PWM6 - Global config */
 {.epwmBase = UINT32_MAX},          /* END OF ARRAY */
};

/***********************************************************************
 * ENCODER STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    eQEP_TypeDef QepCfgList[]
 *  @brief  List all the configurations defined for each Quadrature Encoders modules.
 */
static const eqep_cfg_t QepCfgList[] =
{
 MOTOR1_QEP_DEF,                    /* M1 - Quadrature Encoder config */
 MOTOR2_QEP_DEF,                    /* M2 - Quadrature Encoder config */
 {.eqepBase = UINT32_MAX},          /* END OF ARRAY */
};

/***********************************************************************
 * SCI/UART (RS485) STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const sci_cfg_t  sciCfgList[] =
{
 COM_SCI_DEF,                       /* SCI(UART)/RS485 Communication bus config */
 {.sciBase = UINT32_MAX},           /* END OF ARRAY */
};

/***********************************************************************
 * SPI STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    SPI_TypeDef spiCfgList[]
 *  @brief  List all the configurations defined for each SPI communication modules.
 */
static const spi_cfg_t  spiCfgList[] =
{
 DRV_SPI_DEF,                       /* SPI DRV8353 bus config */
 COM_SPI_DEF,                       /* SPI COMMUNICATION bus config */
 DBG_SPI_DEF,                       /* SPI DEBUG bus config */
 LED_SPI_DEF,                       /* SPI LED DIMMING bus config */
 {.spiBase = UINT32_MAX},           /* END OF ARRAY */
};

/***********************************************************************
 * DMA STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    DMA_TypeDef dmaCfgList[]
 *  @brief  List all the DMA used for communication.
 */
static const dma_cfg_t  dmaCfgList[] =
{
 DMA_LED_SPI_TX_DEF,                /* DMA for the RGB LED */
 DMA_COM_SPI_TX_DEF,                /* DMA for SPI COMMUNICATION Tx */
 DMA_COM_SPI_RX_DEF,                /* DMA for SPI COMMUNICATION Rx */
 {.dmaChBase = UINT32_MAX},         /* END OF ARRAY */
};

/***********************************************************************
 * CLA STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const cla_cfg_t  claCfgList[] =
{
 CLA_ADC_CALIB_DEF,                 /* CLA structure for ADC startup calibration - One Call */
 CLA_CMD_NEW_DEF,                   /* CLA structure for uOmodri command update - 1ms */
 CLA_MOTOR1_DEF,                    /* CLA structure for MOTOR1 control - 20us (40kHz) */
 CLA_MOTOR2_DEF,                    /* CLA structure for MOTOR2 control - 20us (40kHz) */
 {.p_claFunc = NULL},               /* END OF ARRAY */
};

/***********************************************************************
 * CLB STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const clb_cfg_t  clbCfgList[] =
{
  CLB_RS485_DEF,                    /* CLB config for RS485 reception */
};

/***********************************************************************
 * IPC STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const ipc_cfg_t  ipcCfgList[] =
{
 IPC_CPU1_2_CM_DEF,                 /* IPC between CPU1 & CM */
 {.ipcType = IPC_TOTAL_NUM},        /* END OF ARRAY */
};

/***********************************************************************
 * CPU TIMER STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const tmr_cfg_t  timerCfgList[] =
{
 CPU_TIMER0_DEF,                    /* CPU timer 0 config */
 {.cpuTimerBase = UINT32_MAX},      /* END OF ARRAY */
};

/***********************************************************************
 * GLOBALE HAL STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const hal_cfg_t  hal =
{
 .p_gpioHandle  = gpioPinsCfgList,
 .p_adcHandle   = adcCfgList,
 .p_epwmHandle  = pwmCfgList,
 .p_eqepHandle  = QepCfgList,
 .p_dmaHandle   = dmaCfgList,
 .p_ipcHandle   = ipcCfgList,
 .p_sciHandle   = sciCfgList,
 .p_spiHandle   = spiCfgList,
 .p_timerHandle = timerCfgList,
 .p_claHandle   = claCfgList,
 .p_clbHandle   = clbCfgList,
};

/***********************************************************************
 * DRV STRUCTURE IO DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    drv_cfg_t   drv_cfg[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
#pragma DATA_SECTION(drv_cfg, "ramgs0")
static const drv_cfg_t  drv_cfg[] =
{
 MOTOR1_DRV_CFG_DEF,
 MOTOR2_DRV_CFG_DEF,
};

/***********************************************************************
 * DRV STRUCTURE REGISTERS DECLARATION & INITIALIZATION
 ***********************************************************************/
/** @var    drv_reg_t   drv_reg[]
 *  @brief  List all configuration registers for each DRVs.
 */
#pragma DATA_SECTION(drv_reg, "ramgs0")
static drv_reg_t drv_reg[] =
{
 MOTOR1_DRV_REG_DEF,
 MOTOR2_DRV_REG_DEF,
};

#endif /* __UOMODRI_2_HANDLERS_H__ */
