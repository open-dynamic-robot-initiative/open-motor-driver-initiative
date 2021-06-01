#ifndef __UOMODRI_HAL_CONFIG_HANDLERS_H__
#define __UOMODRI_HAL_CONFIG_HANDLERS_H__

// drivers
#include "device.h"
#include "driverlib.h"

#include "hal.h"
#include "spi_communication.h"
#include "uomodri_user_defines.h"

// Communication dual buffers for initialization
extern DualBufferW dualBufferW;
extern DualBufferR dualBufferR;

/** @var    GPIO_TypeDef GpioPinsCfgList[]
 *  @brief  List all the GPIO definitions except the analog pins.
 */
static const GPIO_TypeDef GpioPinsCfgList[] =
{
 // Debug Pins
 {.pinNumber = DEBUG_INT_PIN1,      .fctMux = DEBUG_INT_PIN1_CFG,   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = DEBUG_INT_PIN2,      .fctMux = DEBUG_INT_PIN2_CFG,   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 // Quadrature encoder #1
 {.pinNumber = ENC1_CHA,            .fctMux = ENC1_CHA_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,    .setValue = 0},
 {.pinNumber = ENC1_CHB,            .fctMux = ENC1_CHB_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,    .setValue = 0},
 {.pinNumber = ENC1_CHI,            .fctMux = ENC1_CHI_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,    .setValue = 0},
 // Quadrature encoder #2
 {.pinNumber = ENC2_CHA,            .fctMux = ENC2_CHA_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,    .setValue = 0},
 {.pinNumber = ENC2_CHB,            .fctMux = ENC2_CHB_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,    .setValue = 0},
 {.pinNumber = ENC2_CHI,            .fctMux = ENC2_CHI_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,    .setValue = 0},
 // SPIA pins
 {.pinNumber = DRV_SPI_SIMO,        .fctMux = DRV_SPI_SIMO_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = DRV_SPI_SOMI,        .fctMux = DRV_SPI_SOMI_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_3SAMPLE,    .setValue = 0},
 {.pinNumber = DRV_SPI_CLK,         .fctMux = DRV_SPI_CLK_CFG,      .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 // DRV1 pins
 {.pinNumber = DRV1_GPIO_EN,        .fctMux = DRV1_GPIO_EN_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR1_PWM1_CHA,     .fctMux = MOTOR1_PWM1_CHA_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR1_PWM1_CHB,     .fctMux = MOTOR1_PWM1_CHB_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = MOTOR1_PWM2_CHA,     .fctMux = MOTOR1_PWM2_CHA_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR1_PWM2_CHB,     .fctMux = MOTOR1_PWM2_CHB_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = MOTOR1_PWM3_CHA,     .fctMux = MOTOR1_PWM3_CHA_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR1_PWM3_CHB,     .fctMux = MOTOR1_PWM3_CHB_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = DRV1_GPIO_CS,        .fctMux = DRV1_GPIO_CS_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = DRV1_GPIO_NFAULT,    .fctMux = DRV1_GPIO_NFAULT_CFG, .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_3SAMPLE,    .setValue = 0},
 // DRV2 pins
 {.pinNumber = DRV2_GPIO_EN,        .fctMux = DRV2_GPIO_EN_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR2_PWM1_CHA,     .fctMux = MOTOR2_PWM1_CHA_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR2_PWM1_CHB,     .fctMux = MOTOR2_PWM1_CHB_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = MOTOR2_PWM2_CHA,     .fctMux = MOTOR2_PWM2_CHA_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR2_PWM2_CHB,     .fctMux = MOTOR2_PWM2_CHB_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = MOTOR2_PWM3_CHA,     .fctMux = MOTOR2_PWM3_CHA_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 0},
 {.pinNumber = MOTOR2_PWM3_CHB,     .fctMux = MOTOR2_PWM3_CHB_CFG,  .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = DRV2_GPIO_CS,        .fctMux = DRV2_GPIO_CS_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,      .setValue = 1},
 {.pinNumber = DRV2_GPIO_NFAULT,    .fctMux = DRV2_GPIO_NFAULT_CFG, .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,   .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_3SAMPLE,    .setValue = 0},
 // END OF ARRAY
 {.pinNumber = 0xFFFFFFFF},
};

 /** @var    ADC_Init_TypeDef AdcInitList[]
  *  @brief  List all the configurations defined for ADC Start-Of-Conversion (SOC) submodule.
  */
static const ADC_INIT_TypeDef AdcInitList[] =
{
 /* ADCA - ADCINA0 : Ia Motor1, ADCINA1 : Ia Motor2, ADCINA2 : Va Motor1, ADCINA3 : Va Motor2 */
 {.adcBase = M1_VA_ADC_ADDR,    .adcChannel = M1_VA_CH,     .adcSOCNumber = M1_VA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT1},
 {.adcBase = M2_VA_ADC_ADDR,    .adcChannel = M2_VA_CH,     .adcSOCNumber = M2_VA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT2},
 {.adcBase = M1_IA_ADC_ADDR,    .adcChannel = M1_IA_CH,     .adcSOCNumber = M1_IA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT3},
 {.adcBase = M2_IA_ADC_ADDR,    .adcChannel = M2_IA_CH,     .adcSOCNumber = M2_IA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT4},
 /* ADCB - ADCINB0 : Ib Motor1, ADCINB1 : Ib Motor2, ADCINB2 : Vb Motor1, ADCINB3 : Vb Motor2 */
 {.adcBase = M1_VB_ADC_ADDR,    .adcChannel = M1_VB_CH,     .adcSOCNumber = M1_VB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT1},
 {.adcBase = M2_VB_ADC_ADDR,    .adcChannel = M2_VB_CH,     .adcSOCNumber = M2_VB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT2},
 {.adcBase = M1_IB_ADC_ADDR,    .adcChannel = M1_IB_CH,     .adcSOCNumber = M1_IB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT3},
 {.adcBase = M2_IB_ADC_ADDR,    .adcChannel = M2_IB_CH,     .adcSOCNumber = M2_IB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT4},
 /* ADCC - ADCINC2 : Vbus Motor1, ADCINC3 : Vbus Motor2, ADCINC4 : Vpot Motor1, ADCINC5 : Vpot Motor2 */
 {.adcBase = M12_VBUS_ADC_ADDR, .adcChannel = M12_VBUS_CH,  .adcSOCNumber = M12_VBUS_SOC_NUM,   .adcTriggerSrc = ADC_TRIGGER_EVT1},
 /* ADCD - ADCIND0 : Ic Motor1, ADCIND1 : Ic Motor2, ADCIND2 : Vc Motor1, ADCIND3 : Vc Motor2 */
 {.adcBase = M1_VC_ADC_ADDR,    .adcChannel = M1_VC_CH,     .adcSOCNumber = M1_VC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT1},
 {.adcBase = M2_VC_ADC_ADDR,    .adcChannel = M2_VC_CH,     .adcSOCNumber = M2_VC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT2},
 {.adcBase = M1_IC_ADC_ADDR,    .adcChannel = M1_IC_CH,     .adcSOCNumber = M1_IC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT3},
 {.adcBase = M2_IC_ADC_ADDR,    .adcChannel = M2_IC_CH,     .adcSOCNumber = M2_IC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT4},

 // END OF ARRAY
 {.adcBase = 0xFFFFFFFF},
};

/** @var    ADC_PPB_TypeDef AdcAcqList[]
 *  @brief  List all the configurations defined for ADC Post-Processing Block (PPB) submodule.
 */
static const ADC_ACQ_TypeDef AdcAcqList[] =
{
 /* ADCA : Va_M1, Va_M2, Ia_M1, Ia_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = M1_VA_ADC_ADDR,    .adcResultBase = M1_VA_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_VA_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M1_VA_SOC_NUM,      .adcPPBNumber = M1_VA_PPB_NUM},
 {.adcBase = M2_VA_ADC_ADDR,    .adcResultBase = M2_VA_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_VA_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M2_VA_SOC_NUM,      .adcPPBNumber = M2_VA_PPB_NUM},
 {.adcBase = M1_IA_ADC_ADDR,    .adcResultBase = M1_IA_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_IA_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M1_IA_SOC_NUM,      .adcPPBNumber = M1_IA_PPB_NUM},
 {.adcBase = M2_IA_ADC_ADDR,    .adcResultBase = M2_IA_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_IA_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M2_IA_SOC_NUM,      .adcPPBNumber = M2_IA_PPB_NUM},
 /* ADCB : Vb_M1, Vb_M2, Ib_M1, Ib_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = M1_VB_ADC_ADDR,    .adcResultBase = M1_VB_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_VB_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M1_VB_SOC_NUM,      .adcPPBNumber = M1_VB_PPB_NUM},
 {.adcBase = M2_VB_ADC_ADDR,    .adcResultBase = M2_VB_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_VB_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M2_VB_SOC_NUM,      .adcPPBNumber = M2_VB_PPB_NUM},
 {.adcBase = M1_IB_ADC_ADDR,    .adcResultBase = M1_IB_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_IB_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M1_IB_SOC_NUM,      .adcPPBNumber = M1_IB_PPB_NUM},
 {.adcBase = M2_IB_ADC_ADDR,    .adcResultBase = M2_IB_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_IB_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M2_IB_SOC_NUM,      .adcPPBNumber = M2_IB_PPB_NUM},
 /* ADCC : Vbus_M12; SOC0; PPB1 */
 {.adcBase = M12_VBUS_ADC_ADDR, .adcResultBase = M12_VBUS_ADC_RESULT_ADDR,  .adcPPBResultBase = M12_VBUS_ADC_PPB_RESULT_ADDR,   .adcSOCNumber = M12_VBUS_SOC_NUM,   .adcPPBNumber = M12_VBUS_PPB_NUM},
 /* ADCD : Vc_M1, Vc_M2, Ic_M1, Ic_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = M1_VC_ADC_ADDR,    .adcResultBase = M1_VC_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_VC_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M1_VC_SOC_NUM,      .adcPPBNumber = M1_VC_PPB_NUM},
 {.adcBase = M2_VC_ADC_ADDR,    .adcResultBase = M2_VC_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_VC_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M2_VC_SOC_NUM,      .adcPPBNumber = M2_VC_PPB_NUM},
 {.adcBase = M1_IC_ADC_ADDR,    .adcResultBase = M1_IC_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_IC_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M1_IC_SOC_NUM,      .adcPPBNumber = M1_IC_PPB_NUM},
 {.adcBase = M2_IC_ADC_ADDR,    .adcResultBase = M2_IC_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_IC_ADC_PPB_RESULT_ADDR,      .adcSOCNumber = M2_IC_SOC_NUM,      .adcPPBNumber = M2_IC_PPB_NUM},
 // END OF ARRAY
 {.adcBase = 0xFFFFFFFF}
};

/** @var    ADC_IT_TypeDef AdcIntList[]
 *  @brief  List all the interrupts source generated by ADC modules.
 */
static const ADC_IT_TypeDef AdcIntList[] =
{
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M1_VA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER1},
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M2_VA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER2},
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M1_IA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER3},
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M2_IA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER4},
 // END OF ARRAY
 {.adcBase = 0xFFFFFFFF}
};

/** @var    ADC_TypeDef AdcCfgList[]
 *  @brief  List all the configurations defined for each ADC modules.
 */
static const ADC_TypeDef AdcCfgList[] =
{
 {.adcBase = M12_IVA_ADC_ADDR,  .adcResultBase = M12_IVA_ADC_RESULT_ADDR,   .adcInit = &AdcInitList[0], .adcAcq = &AdcAcqList[0],   .adcIt = &AdcIntList[0]},
 {.adcBase = M12_IVB_ADC_ADDR,  .adcResultBase = M12_IVB_ADC_RESULT_ADDR,   .adcInit = &AdcInitList[4], .adcAcq = &AdcAcqList[4],   .adcIt = &AdcIntList[1]},
 {.adcBase = M12_VBUS_ADC_ADDR, .adcResultBase = M12_VBUS_ADC_RESULT_ADDR,  .adcInit = &AdcInitList[8], .adcAcq = &AdcAcqList[8],   .adcIt = &AdcIntList[2]},
 {.adcBase = M12_IVC_ADC_ADDR,  .adcResultBase = M12_IVC_ADC_RESULT_ADDR,   .adcInit = &AdcInitList[9], .adcAcq = &AdcAcqList[9],   .adcIt = &AdcIntList[3]},
 // END OF ARRAY
 {.adcBase = 0xFFFFFFFF},
};

/** @var    eQEP_TypeDef QepCfgList[]
 *  @brief  List all the configurations defined for each Quadrature Encoders modules.
 */
static const eQEP_TypeDef QepCfgList[] =
{
 {.eqepBase = EQEP1_BASE, .config = (EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_SWAP)},
 {.eqepBase = EQEP2_BASE, .config = (EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_SWAP)},
 // END OF ARRAY
 {.eqepBase = 0xFFFFFFFF},
};

/** @var    SPI_TypeDef spiCfgList[]
 *  @brief  List all the configurations defined for each SPI communication modules.
 */
static const SPI_TypeDef spiCfgList[] =
{
 {.spiBase = DRV_SPI_BASE, .bitRate = 400000U,  .dataWidth = 16U, .mode = SPI_MODE_MASTER,  .protocol = SPI_PROT_POL0PHA0, .txLevel = SPI_FIFO_TXEMPTY, .rxLevel= SPI_FIFO_RXEMPTY},
 {.spiBase = COM_SPI_BASE, .bitRate = 6000000U, .dataWidth = 16U, .mode = SPI_MODE_SLAVE,   .protocol = SPI_PROT_POL0PHA1, .txLevel = SPI_FIFO_TX14,    .rxLevel= SPI_FIFO_RX1},
 // END OF ARRAY
 {.spiBase = 0xFFFFFFFF},
};

static const DMA_TypeDef dmaCfgList[] =
{
 // DMA base register initialization
 // DMA for SPI communication Tx
 {.dmaChBase = DMA_CH5_BASE, .transferSize = SPI_COM_LENGTH, .srcWrapSize = SPI_COM_LENGTH, .destWrapSize = 0,
  .config = ( DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT),
  .burstSize = 1, .destStep = 0, .srcStep = 1,
  .destAddr = (uint16_t*)(COM_SPI_BASE + SPI_O_TXBUF), .srcAddr = dualBufferW.comPackets, .trigger = DMA_TRIGGER_SPIBTX
 },
 // DMA for SPI communication Rx
 {.dmaChBase = DMA_CH6_BASE, .transferSize = SPI_COM_LENGTH, .srcWrapSize = 0, .destWrapSize = SPI_COM_LENGTH,
  .config = (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT),
  .burstSize = 1, .destStep = 1, .srcStep = 0,
  .destAddr = dualBufferR.comPackets, .srcAddr = (uint16_t*)(COM_SPI_BASE + SPI_O_RXBUF), .trigger = DMA_TRIGGER_SPIBRX
 },
 // END OF ARRAY
 {.dmaChBase = 0xFFFFFFFF},
};

/** @var    ePWM_TB_TypeDef PwmTimeBaseCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Time Base submodule.
 */
static const ePWM_TB_TypeDef PwmTimeBaseCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .phaseCount = PWM_TBPHS,        .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO},
 {.epwmBase = MOTOR1_PWM2_BASE, .phaseCount = PWM_TBPHS,        .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR1_PWM3_BASE, .phaseCount = PWM_TBPHS,        .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR2_PWM1_BASE, .phaseCount = PWM_HALF_TBPRD,   .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR2_PWM2_BASE, .phaseCount = PWM_HALF_TBPRD,   .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR2_PWM3_BASE, .phaseCount = PWM_HALF_TBPRD,   .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 // END OF ARRAY
 {.epwmBase = 0xFFFFFFFF},
};

/** @var    ePWM_CC_TypeDef PwmCounterCompareCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Capture/Compare submodule.
 */
static const ePWM_CC_TypeDef PwmCounterCompareCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .compCount = PWM_CCCMPx, .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR1_PWM2_BASE, .compCount = PWM_CCCMPx, .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR1_PWM3_BASE, .compCount = PWM_CCCMPx, .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR2_PWM1_BASE, .compCount = PWM_CCCMPx, .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR2_PWM2_BASE, .compCount = PWM_CCCMPx, .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR2_PWM3_BASE, .compCount = PWM_CCCMPx, .compModule = EPWM_COUNTER_COMPARE_A},
 // END OF ARRAY
 {.epwmBase = 0xFFFFFFFF},
};

/** @var    ePWM_AQ_TypeDef PwmActionQualifierCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Action Qualifier submodule.
 */
static const ePWM_AQ_TypeDef PwmActionQualifierCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR1_PWM2_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR1_PWM3_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR2_PWM1_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR2_PWM2_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR2_PWM3_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 // END OF ARRAY
 {.epwmBase = 0xFFFFFFFF},
};

/** @var    ePWM_DB_TypeDef PwmDeadBandCfgList[]
 *  @brief  List all the configurations necessaries for ePWM DeadBand Unit submodule.
 */
static const ePWM_DB_TypeDef PwmDeadBandCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .redCount = PWM_DBRED, .fedCount = PWM_DBFED, .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR1_PWM2_BASE, .redCount = PWM_DBRED, .fedCount = PWM_DBFED, .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR1_PWM3_BASE, .redCount = PWM_DBRED, .fedCount = PWM_DBFED, .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR2_PWM1_BASE, .redCount = PWM_DBRED, .fedCount = PWM_DBFED, .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR2_PWM2_BASE, .redCount = PWM_DBRED, .fedCount = PWM_DBFED, .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR2_PWM3_BASE, .redCount = PWM_DBRED, .fedCount = PWM_DBFED, .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = 0xFFFFFFFF},
};

/** @var    ePWM_ET_TypeDef PwmAdcSocEventTriggerCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Event Trigger submodule.
 */
static const ePWM_ET_TypeDef PwmAdcSocEventTriggerCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_A, .adcSOCSource = EPWM_SOC_TBCTR_PERIOD},
 {.epwmBase = MOTOR1_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_B, .adcSOCSource = EPWM_SOC_TBCTR_ZERO},
 {.epwmBase = MOTOR2_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_A, .adcSOCSource = EPWM_SOC_TBCTR_PERIOD},
 {.epwmBase = MOTOR2_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_B, .adcSOCSource = EPWM_SOC_TBCTR_ZERO},
 // END OF ARRAY
 {.epwmBase = 0xFFFFFFFF},
};

/** @var    ePWM_IT_TypeDef PwmInterruptsEventCfgList[]
 *  @brief  List all the configurations necessaries for ePWM IT Event submodule.
 */
static const ePWM_IT_TypeDef PwmInterruptsEventCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .intEvtCount = 1, .intSource = EPWM_INT_TBCTR_ZERO},
// {.epwmBase = MOTOR2_PWM1_BASE, .intEvtCount = 0, .intSource = EPWM_INT_TBCTR_ZERO},
 // END OF ARRAY
 {.epwmBase = 0xFFFFFFFF},
};

/** @var    ePWM_TypeDef PwmCfgList[]
 *  @brief  List all the configurations necessaries for PWM module.
 */
static const ePWM_TypeDef PwmCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .epwmTimeBase = &PwmTimeBaseCfgList[0], .epwmCaptureCompare = &PwmCounterCompareCfgList[0],
  .epwmActionQualifier = &PwmActionQualifierCfgList[0], .epwmDeadband = &PwmDeadBandCfgList[0], .epwmProgChopper = NULL,
  .epwmAdcSocEventTrigger = &PwmAdcSocEventTriggerCfgList[0], .epwmInterruptsEvent = &PwmInterruptsEventCfgList[0]
 },
 {.epwmBase = MOTOR1_PWM2_BASE, .epwmTimeBase = &PwmTimeBaseCfgList[1], .epwmCaptureCompare = &PwmCounterCompareCfgList[1],
  .epwmActionQualifier = &PwmActionQualifierCfgList[1], .epwmDeadband = &PwmDeadBandCfgList[1], .epwmProgChopper = NULL,
  .epwmAdcSocEventTrigger = NULL, .epwmInterruptsEvent = NULL
 },
 {.epwmBase = MOTOR1_PWM3_BASE, .epwmTimeBase = &PwmTimeBaseCfgList[2], .epwmCaptureCompare = &PwmCounterCompareCfgList[2],
  .epwmActionQualifier = &PwmActionQualifierCfgList[2], .epwmDeadband = &PwmDeadBandCfgList[2], .epwmProgChopper = NULL,
  .epwmAdcSocEventTrigger = NULL, .epwmInterruptsEvent = NULL
 },
 {.epwmBase = MOTOR2_PWM1_BASE, .epwmTimeBase = &PwmTimeBaseCfgList[3], .epwmCaptureCompare = &PwmCounterCompareCfgList[3],
  .epwmActionQualifier = &PwmActionQualifierCfgList[3], .epwmDeadband = &PwmDeadBandCfgList[3], .epwmProgChopper = NULL,
  .epwmAdcSocEventTrigger = &PwmAdcSocEventTriggerCfgList[2], .epwmInterruptsEvent = NULL
 },
 {.epwmBase = MOTOR2_PWM2_BASE, .epwmTimeBase = &PwmTimeBaseCfgList[4], .epwmCaptureCompare = &PwmCounterCompareCfgList[4],
  .epwmActionQualifier = &PwmActionQualifierCfgList[4], .epwmDeadband = &PwmDeadBandCfgList[4], .epwmProgChopper = NULL,
  .epwmAdcSocEventTrigger = NULL, .epwmInterruptsEvent = NULL
 },
 {.epwmBase = MOTOR2_PWM3_BASE, .epwmTimeBase = &PwmTimeBaseCfgList[5], .epwmCaptureCompare = &PwmCounterCompareCfgList[5],
  .epwmActionQualifier = &PwmActionQualifierCfgList[5], .epwmDeadband = &PwmDeadBandCfgList[5], .epwmProgChopper = NULL,
  .epwmAdcSocEventTrigger = NULL, .epwmInterruptsEvent = NULL
 },
 // END OF ARRAY
 {.epwmBase = 0xFFFFFFFF},
};

/** @var    HAL     hal
 *  @brief  General configuration structure for the Hardware Abstraction Layer.
 */
static const HAL hal =
{.gpioHandler = GpioPinsCfgList,
 .adcHandler  = AdcCfgList,
 .epwmHandler = PwmCfgList,
 .eqepHandler = QepCfgList,
 .dmaHandler  = NULL,
 .spiHandler  = spiCfgList
};

#endif  /*__UOMODRI_HAL_CONFIG_HANDLERS_H__*/
