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

// Communication dual buffers for initialization
extern DualBufferW dualBufferW;
extern DualBufferR dualBufferR;

/******************************************************************************************************
*                                               GPIO pins list definition
*******************************************************************************************************/
/** @var    GPIO_TypeDef GpioPinsCfgList[]
 *  @brief  List all the GPIO definitions except the analog pins.
 */
static const GPIO_TypeDef GpioPinsCfgList[] =
{
 // Debug Pins
 {DEBUG_INT_PIN1,   DEBUG_INT_PIN1_CFG,     GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {DEBUG_INT_PIN2,   DEBUG_INT_PIN2_CFG,     GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 // Quadrature encoder #1
 {ENC1_CHA,         ENC1_CHA_CFG,           GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_6SAMPLE,    0},
 {ENC1_CHB,         ENC1_CHB_CFG,           GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_6SAMPLE,    0},
 {ENC1_CHI,         ENC1_CHI_CFG,           GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_6SAMPLE,    0},
 // Quadrature encoder #2
 {ENC2_CHA,         ENC2_CHA_CFG,           GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_6SAMPLE,    0},
 {ENC2_CHB,         ENC2_CHB_CFG,           GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_6SAMPLE,    0},
 {ENC2_CHI,         ENC2_CHI_CFG,           GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_6SAMPLE,    0},
 // SPIA pins
 {DRV_SPI_SIMO,     DRV_SPI_SIMO_CFG,       GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {DRV_SPI_SOMI,     DRV_SPI_SOMI_CFG,       GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_3SAMPLE,    0},
 {DRV_SPI_CLK,      DRV_SPI_CLK_CFG,        GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 // DRV1 pins
 {DRV1_GPIO_EN,     DRV1_GPIO_EN_CFG,       GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR1_PWM1_CHA,  MOTOR1_PWM1_CHA_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR1_PWM1_CHB,  MOTOR1_PWM1_CHB_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {MOTOR1_PWM2_CHA,  MOTOR1_PWM2_CHA_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR1_PWM2_CHB,  MOTOR1_PWM2_CHB_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {MOTOR1_PWM3_CHA,  MOTOR1_PWM3_CHA_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR1_PWM3_CHB,  MOTOR1_PWM3_CHB_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {DRV1_GPIO_CS,     DRV1_GPIO_CS_CFG,       GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {DRV1_GPIO_NFAULT, DRV1_GPIO_NFAULT_CFG,   GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_3SAMPLE,    0},
 // DRV2 pins
 {DRV2_GPIO_EN,     DRV2_GPIO_EN_CFG,       GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR2_PWM1_CHA,  MOTOR2_PWM1_CHA_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR2_PWM1_CHB,  MOTOR2_PWM1_CHB_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {MOTOR2_PWM2_CHA,  MOTOR2_PWM2_CHA_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR2_PWM2_CHB,  MOTOR2_PWM2_CHB_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {MOTOR2_PWM3_CHA,  MOTOR2_PWM3_CHA_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      0},
 {MOTOR2_PWM3_CHB,  MOTOR2_PWM3_CHB_CFG,    GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {DRV2_GPIO_CS,     DRV2_GPIO_CS_CFG,       GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_OUT, GPIO_QUAL_ASYNC,      1},
 {DRV2_GPIO_NFAULT, DRV2_GPIO_NFAULT_CFG,   GPIO_PIN_TYPE_STD, GPIO_CORE_CPU1, GPIO_DIR_MODE_IN,  GPIO_QUAL_3SAMPLE,    0},
 // END OF ARRAY
 {.pinNumber = 0xFFFFFFFF},
};

 /** @var    ADC_SOC_TypeDef AdcSOCsList[]
  *  @brief  List all the configurations defined for ADC Start-Of-Conversion (SOC) submodule.
  */
static const ADC_SOC_TypeDef AdcSOCsList[] =
{
 /* ADCA - ADCINA0 : Ia Motor1, ADCINA1 : Ia Motor2, ADCINA2 : Va Motor1, ADCINA3 : Va Motor2 */
 {.adcBase = M1_IA_ADC_ADDR,    .adcChannel = M1_IA_CH,     .adcSOCNumber = M1_IA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT1},
 {.adcBase = M2_IA_ADC_ADDR,    .adcChannel = M2_IA_CH,     .adcSOCNumber = M2_IA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT2},
 {.adcBase = M1_VA_ADC_ADDR,    .adcChannel = M1_VA_CH,     .adcSOCNumber = M1_VA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT3},
 {.adcBase = M2_VA_ADC_ADDR,    .adcChannel = M2_VA_CH,     .adcSOCNumber = M2_VA_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT4},
 /* ADCB - ADCINB0 : Ib Motor1, ADCINB1 : Ib Motor2, ADCINB2 : Vb Motor1, ADCINB3 : Vb Motor2 */
 {.adcBase = M1_IB_ADC_ADDR,    .adcChannel = M1_IB_CH,     .adcSOCNumber = M1_IB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT1},
 {.adcBase = M2_IB_ADC_ADDR,    .adcChannel = M2_IB_CH,     .adcSOCNumber = M2_IB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT2},
 {.adcBase = M1_VB_ADC_ADDR,    .adcChannel = M1_VB_CH,     .adcSOCNumber = M1_VB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT3},
 {.adcBase = M2_VB_ADC_ADDR,    .adcChannel = M2_VB_CH,     .adcSOCNumber = M2_VB_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT4},
 /* ADCC - ADCINC2 : Vbus Motor1, ADCINC3 : Vbus Motor2, ADCINC4 : Vpot Motor1, ADCINC5 : Vpot Motor2 */
 {.adcBase = M1_VBUS_ADC_ADDR,  .adcChannel = M1_VBUS_CH,   .adcSOCNumber = M1_VBUS_SOC_NUM,    .adcTriggerSrc = ADC_TRIGGER_EVT1},
 {.adcBase = M2_VBUS_ADC_ADDR,  .adcChannel = M2_VBUS_CH,   .adcSOCNumber = M2_VBUS_SOC_NUM,    .adcTriggerSrc = ADC_TRIGGER_EVT2},
 /* ADCD - ADCIND0 : Ic Motor1, ADCIND1 : Ic Motor2, ADCIND2 : Vc Motor1, ADCIND3 : Vc Motor2 */
 {.adcBase = M1_IC_ADC_ADDR,    .adcChannel = M1_IC_CH,     .adcSOCNumber = M1_IC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT1},
 {.adcBase = M2_IC_ADC_ADDR,    .adcChannel = M2_IC_CH,     .adcSOCNumber = M2_IC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT2},
 {.adcBase = M1_VC_ADC_ADDR,    .adcChannel = M1_VC_CH,     .adcSOCNumber = M1_VC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT3},
 {.adcBase = M2_VC_ADC_ADDR,    .adcChannel = M2_VC_CH,     .adcSOCNumber = M2_VC_SOC_NUM,      .adcTriggerSrc = ADC_TRIGGER_EVT4},
 // END OF ARRAY
 {.adcBase = 0xFFFFFFFF},
};

/** @var    ADC_PPB_TypeDef AdcPPBsList[]
 *  @brief  List all the configurations defined for ADC Post-Processing Block (PPB) submodule.
 */
static const ADC_PPB_TypeDef AdcPPBsList[] =
{
 /* ADCA : Ia_M1, Ia_M2, Va_M1, Va_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = M1_IA_ADC_ADDR,    .adcResultBase = M1_IA_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_IA_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M1_IA_SOC_NUM,   .adcPPBNumber = M1_IA_PPB_NUM},
 {.adcBase = M2_IA_ADC_ADDR,    .adcResultBase = M2_IA_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_IA_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M2_IA_SOC_NUM,   .adcPPBNumber = M2_IA_PPB_NUM},
 {.adcBase = M1_VA_ADC_ADDR,    .adcResultBase = M1_VA_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_VA_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M1_VA_SOC_NUM,   .adcPPBNumber = M1_VA_PPB_NUM},
 {.adcBase = M2_VA_ADC_ADDR,    .adcResultBase = M2_VA_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_VA_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M2_VA_SOC_NUM,   .adcPPBNumber = M2_VA_PPB_NUM},
 /* ADCB : Ib_M1, Ib_M2, Vb_M1, Vb_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = M1_IB_ADC_ADDR,    .adcResultBase = M1_IB_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_IB_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M1_IB_SOC_NUM,   .adcPPBNumber = M1_IB_PPB_NUM},
 {.adcBase = M2_IB_ADC_ADDR,    .adcResultBase = M2_IB_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_IB_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M2_IB_SOC_NUM,   .adcPPBNumber = M2_IB_PPB_NUM},
 {.adcBase = M1_VB_ADC_ADDR,    .adcResultBase = M1_VB_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_VB_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M1_VB_SOC_NUM,   .adcPPBNumber = M1_VB_PPB_NUM},
 {.adcBase = M2_VB_ADC_ADDR,    .adcResultBase = M2_VB_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_VB_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M2_VB_SOC_NUM,   .adcPPBNumber = M2_VB_PPB_NUM},
 /* ADCC : Vbus_M1, Vbus_M2; SOC0 - 1; PPB1 - 2 */
 {.adcBase = M1_VBUS_ADC_ADDR,  .adcResultBase = M1_VBUS_ADC_RESULT_ADDR,   .adcPPBResultBase = M1_VBUS_ADC_PPB_RESULT_ADDR,.adcSOCNumber = M1_VBUS_SOC_NUM, .adcPPBNumber = M1_VBUS_PPB_NUM},
 {.adcBase = M2_VBUS_ADC_ADDR,  .adcResultBase = M2_VBUS_ADC_RESULT_ADDR,   .adcPPBResultBase = M2_VBUS_ADC_PPB_RESULT_ADDR,.adcSOCNumber = M2_VBUS_SOC_NUM, .adcPPBNumber = M2_VBUS_PPB_NUM},
 /* ADCD : Ic_M1, Ic_M2, Vc_M1, Vc_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = M1_IC_ADC_ADDR,    .adcResultBase = M1_IC_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_IC_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M1_IC_SOC_NUM,   .adcPPBNumber = M1_IC_PPB_NUM},
 {.adcBase = M2_IC_ADC_ADDR,    .adcResultBase = M2_IC_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_IC_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M2_IC_SOC_NUM,   .adcPPBNumber = M2_IC_PPB_NUM},
 {.adcBase = M1_VC_ADC_ADDR,    .adcResultBase = M1_VC_ADC_RESULT_ADDR,     .adcPPBResultBase = M1_VC_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M1_VC_SOC_NUM,   .adcPPBNumber = M1_VC_PPB_NUM},
 {.adcBase = M2_VC_ADC_ADDR,    .adcResultBase = M2_VC_ADC_RESULT_ADDR,     .adcPPBResultBase = M2_VC_ADC_PPB_RESULT_ADDR,  .adcSOCNumber = M2_VC_SOC_NUM,   .adcPPBNumber = M2_VC_PPB_NUM},
 // END OF ARRAY
 {.adcBase = 0xFFFFFFFF}
};

/** @var    ADC_IT_TypeDef AdcIntList[]
 *  @brief  List all the interrupts source generated by ADC modules.
 */
static const ADC_IT_TypeDef AdcIntList[] =
{
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M1_IA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER1},
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M2_IA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER2},
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M1_VA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER3},
 {.adcBase = M12_IVA_ADC_ADDR, .adcEOCNumber = M2_VA_SOC_NUM,   .adcIntNumber = ADC_INT_NUMBER4},
 // END OF ARRAY
 {.adcBase = 0xFFFFFFFF}
};

/** @var    ADC_TypeDef AdcCfgList[]
 *  @brief  List all the configurations defined for each ADC modules.
 */
static const ADC_TypeDef AdcCfgList[] =
{
 {.adcBase = M12_IVA_ADC_ADDR,  .adcResultBase = M12_IVA_ADC_RESULT_ADDR,   .adcSoc = &AdcSOCsList[0],  .adcPpb = &AdcPPBsList[0],  .adcIt = &AdcIntList[0]},
 {.adcBase = M12_IVB_ADC_ADDR,  .adcResultBase = M12_IVB_ADC_RESULT_ADDR,   .adcSoc = &AdcSOCsList[4],  .adcPpb = &AdcPPBsList[4],  .adcIt = &AdcIntList[1]},
 {.adcBase = M12_VBUS_ADC_ADDR, .adcResultBase = M12_VBUS_ADC_RESULT_ADDR,  .adcSoc = &AdcSOCsList[8],  .adcPpb = &AdcPPBsList[8],  .adcIt = &AdcIntList[2]},
 {.adcBase = M12_IVC_ADC_ADDR,  .adcResultBase = M12_IVC_ADC_RESULT_ADDR,   .adcSoc = &AdcSOCsList[12], .adcPpb = &AdcPPBsList[10], .adcIt = &AdcIntList[3]},
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
 //{epwmBase, phaseCount, syncOutputMode}
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
 //{epwmBase, compCount, compModule}
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
 //{epwmBase, actionsEvt, epwmOutput}
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
 //{epwmBase, redCount, fedCount, DelayMode}
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
 //{epwmBase, adcSOCEvtCount, adcSOCType, adcSOCSource}
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
 //{epwmBase, intEvtCount, intSource}
 {.epwmBase = MOTOR1_PWM1_BASE, .intEvtCount = 1, .intSource = EPWM_INT_TBCTR_PERIOD},
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

/** @var    DRV_Cfg     DrvCfgList[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
static DRV_Cfg DrvCfgList[] =
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

/** @var    HAL     hal
 *  @brief  General configuration structure for the Hardware Abstraction Layer.
 */
static HAL hal =
{.gpioHandler = GpioPinsCfgList,
 .adcHandler  = AdcCfgList,
 .epwmHandler = PwmCfgList,
 .eqepHandler = QepCfgList,
 .dmaHandler  = NULL,
 .spiHandler  = spiCfgList
};


/** @var    DRV8353     drv
 *  @brief  General DRV8353 configuration structure.
 */
static DRV8353 drv =
{.drvCfgHandler = DrvCfgList,
 .drvRegHandler = DrvRegList
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
