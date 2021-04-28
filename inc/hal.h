/*
 * Filename: hal.h
 * Description: Header file for the Hardware Abstraction Layer
 */

#ifndef __HAL_H__
#define __HAL_H__

#include "driverlib.h"
#include "device.h"

//#include "drv8353.h"

/**
 * @enum    Motor_Sel
 * @brief   Easier motor identification
 */
typedef enum
{
    MOTOR_1 = 0,    /*!< Select Motor_1 */
    MOTOR_2  = 1    /*!< Select Motor_2 */
} Motor_Sel;

/**
 * @enum    EPWM_DelayModeEnable
 * @brief   Enable or disable Deadband delay unit.
 */
typedef enum
{
    EPWM_DELAY_DISABLE = 0, /*!< Disable/bypass Deadband delay mode. */
    EPWM_DELAY_ENABLE = 1   /*!< Enable Deadband delay mode. */
} EPWM_DelayModeEnable;

/**
 * @struct  ePWM_TB_TypeDef
 * @brief   ePWM - Time Base submodule structure definition.
 */
typedef struct  _ePWM_Time_Base_TypeDef_
{
    uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    uint16_t phaseCount;                            /*!< Phase counter. Sets offset (min value). */
    //EPWM_TimeBaseCountMode counterMode;             /*!< Counting direction (up-count, down-count or up and down count). \see EPWM_TimeBaseCountMode */
    uint16_t syncOutputMode;                        /*!< Output sync signal generation. \see EPWM_SyncOutPulseMode */
} ePWM_TB_TypeDef;

/**
 * @struct  ePWM_CC_TypeDef
 * @brief   ePWM - Counter Compare submodule structure definition.
 */
typedef struct _ePWM_Counter_Compare_TypeDef_
{
    uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    uint16_t compCount;                             /*!< Compare reference register. */
    EPWM_CounterCompareModule compModule;           /*!< Compare module (A, B, C or D). \see EPWM_CounterCompareModule */
} ePWM_CC_TypeDef;

/**
 * @struct  ePWM_AQ_TypeDef
 * @brief   ePWM - Action Qualifier submodule structure definition.
 */
typedef struct _ePWM_Action_Qualifier_TypeDef_
{
    uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    uint16_t actionsEvt;                            /*!< Actions associated to events matching detection. \see EPWM_ActionQualifierEventAction */
    EPWM_ActionQualifierOutputModule epwmOutput;    /*!< Output channel output (A or B). \see EPWM_ActionQualifierOutputModule */
} ePWM_AQ_TypeDef;

/**
 * @struct  ePWM_DB_TypeDef
 * @brief   ePWM - DeadBand Unit submodule structure definition.
 */
typedef struct _ePWM_Deadband_TypeDef_
{
    uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    uint16_t redCount;                              /*!< Rising Edge Delay counter. */
    uint16_t fedCount;                              /*!< Falling Edge Delay counter. */
    EPWM_DelayModeEnable DelayMode;                 /*!< Enable deadband unit. */
} ePWM_DB_TypeDef;

/**
 * @struct  ePWM_PC_TypeDef
 * @brief   ePWM - Chopper submodule structure definition.
 */
typedef struct _ePWM_Chopper_TypeDef_
{
    uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    uint16_t dutyCycleCount;                        /*!< Programmable chopping (carrier) frequency */
    uint16_t freqDiv;
    uint16_t firstPulseWidth;                       /*!< Programmable duty cycle of second and subsequent pulses */
} ePWM_PC_TypeDef;

/**
 * @struct  ePWM_ET_TypeDef
 * @brief   ePWM - Event trigger \& interrupts submodule structure definition.
 */
typedef struct _ePWM_Event_Trigger_TypeDef_
{
    uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    uint16_t intEvtCount;                           /*!< Delays by N events before generating an IT. */
    uint16_t intSource;                             /*!< PWM events source IT. <em> EPWM_INT_TBCTR_ZERO, _PERIOD, _ZERO_OR_PERIOD, _U_CMPx, _D_CMPx,...</em>  */
    uint16_t adcSOCEvtCount;                        /*!< Delays by N events before generating an ADC SOC event. */
    EPWM_ADCStartOfConversionType adcSOCType;       /*!< ADC Start of Conversion (SOC) target \em EPWM_SOC_A or \em EPWM_SOC_B */
    EPWM_ADCStartOfConversionSource adcSOCSource;   /*!< PWM events source for a ADC SOC. \see EPWM_ADCStartOfConversionSource */
} ePWM_ET_TypeDef;

/**
 * @struct  ePWM_TypeDef
 * @brief   ePWM global Hardware Abstraction Layer structure
 */
typedef struct _ePWM_Cfg_TypeDef_
{
    const uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    const ePWM_TB_TypeDef* epwmTimeBase;
    const ePWM_CC_TypeDef* epwmCaptureCompare;
    const ePWM_AQ_TypeDef* epwmActionQualifier;
    const ePWM_DB_TypeDef* epwmDeadband;
    const ePWM_PC_TypeDef* epwmProgChopper;
    const ePWM_ET_TypeDef* epwmEventTrigger;
} ePWM_TypeDef;

/**
 * @struct  eQEP_TypeDef
 * @brief   eQEP global Hardware Abstraction Layer structure
 */
typedef struct _eQEP_Cfg_TypeDef_
{
    uint32_t eqepBase;                              /*!< eQEP peripheral address. */
    uint16_t config;
} eQEP_TypeDef;

/**
 * @struct  SPI_TypeDef
 * @brief   SPI global Hardware Abstraction Layer structure
 */
typedef struct _SPI_Cfg_TypeDef_
{
    uint32_t spiBase;                               /*!< ADC peripheral address. */
    uint32_t bitRate;
    uint16_t dataWidth;
    SPI_Mode mode;
    SPI_TransferProtocol protocol;
    SPI_TxFIFOLevel txLevel;
    SPI_RxFIFOLevel rxLevel;
    bool spiFifoEnable;
} SPI_TypeDef;

typedef struct _DMA_Cfg_TypeDef_
{
    uint32_t dmaBase;
    uint32_t transferSize;//    uint32_t dmaRxBase;
    uint32_t srcWrapSize;
    uint32_t destWrapSize;
    uint32_t config;
    uint16_t burstSize;
    int16_t destStep;
    int16_t srcStep;
    const void* destAddr;
    const void* srcAddr;
    DMA_Trigger trigger;
} DMA_TypeDef;

//typedef struct _CLA_Cfg_TypeDef_
//{
//
//} CLA_TypeDef;

/**
 * @struct  GPIO_TypeDef
 * @brief   GPIO structure definition for all IOs except analog ones.
 */
typedef struct _GPIO_Cfg_TypeDef_
{
    uint32_t pinNumber;                             /*!< GPIO pin number (GPIO0 - GPIO168). */
    uint32_t fctMux;                                /*!< GPIO configuration functions (peripheral affectation matrix). \see pin_map.h */
    uint32_t pinType;                               /*!< GPIO pad configuration :
    - \em GPIO_PIN_TYPE_STD (Push-pull output or floating input),
    - \em GPIO_PIN_TYPE_PULLUP (Pull-up enable for input),
    - \em GPIO_PIN_TYPE_INVERT (Invert polarity on input),
    - \em GPIO_PIN_TYPE_OD (Open-drain on output) */
    GPIO_CoreSelect coreSelect;                     /*!< GPIO core dependency.
    - \em GPIO_CORE_CPU1,
    - \em GPIO_CORE_CPU1_CLA1,
    - \em GPIO_CORE_CPU2,
    - \em GPIO_CORE_CPU2_CLA1 */
    GPIO_Direction direction;                       /*!< GPIO direction :
    - \em GPIO_DIR_MODE_IN,
    - \em GPIO_DIR_MODE_OUT */
    GPIO_QualificationMode samplingInMode;          /*!< GPIO input sampling mode :
    - \em GPIO_QUAL_SYNC,
    - \em GPIO_QUAL_3SAMPLE,
    - \em GPIO_QUAL_6SAMPLE,
    - \em GPIO_QUAL_ASYNC */
    bool setValue;                                  /*!< GPIO initial value if defined as output. */
//    GPIO_ExternalIntNum extIntNum;
} GPIO_TypeDef;

/**
 * @struct  ADC_SOC_TypeDef
 * @brief   ADC start-of-conversion configuration handler
 */
typedef struct ADC_SOC_TypeDef
{
    uint32_t adcBase;                               /*!< ADC peripheral address. */
    ADC_Channel adcChannel;                         /*!< ADC sampling channel. */
    ADC_SOCNumber adcSOCNumber;                     /*!< ADC Start-Of-Conversion (SOC) number (0-15). */
    ADC_Trigger adcTriggerSrc;                      /*!< ADC SOC trigger source. */
} ADC_SOC_TypeDef;

/**
 * @struct  ADC_PPB_TypeDef
 * @brief   ADC Post-Processing Block configuration handler
 */
typedef struct _ADC_PPB_TypeDef_
{
    uint32_t adcBase;                               /*!< ADC peripheral address. */
    uint16_t adcCalib;
    uint16_t adcOffset;
    ADC_SOCNumber adcSOCNumber;
    ADC_PPBNumber adcPPBNumber;
} ADC_PPB_TypeDef;

/**
 * @struct  ADC_IT_TypeDef
 * @brief   ADC Interrupt source configuration handler
 */
typedef struct _ADC_IT_TypeDef_
{
    uint32_t adcBase;                               /*!< ADC peripheral address. */
    ADC_SOCNumber adcEOCNumber;                     /*!< ADC End-Of-Conversion (EOC) number (0-15). */
    ADC_IntNumber adcIntNumber;                     /*!< IT source (0-3) associated to ADC \& SOC number. */
} ADC_IT_TypeDef;

/**
 * @struct  ADC_TypeDef
 * @brief   ADC module configuration handler
 */
typedef struct _ADC_TypeDef_
{
    const uint32_t adcBase;                         /*!< ADC peripheral address. */
    const ADC_SOC_TypeDef* adcSoc;
    const ADC_PPB_TypeDef* adcPpb;
    const ADC_IT_TypeDef*  adcIt;
} ADC_TypeDef;

// Debug-only structure used to store motor data
typedef struct _MotorDebug_
{
    // ADC measurements
//    float32_t vBus;
//    float32_t vShunt[3];
//    float32_t vShuntOffset[3];

    float32_t iaOffset[4];
    float32_t ibOffset[4];
    float32_t icOffset[4];
    float32_t vbusOffset[4];
    float32_t vaOffset[4];
    float32_t vbOffset[4];
    float32_t vcOffset[4];
//    float32_t vpotOffset[4];

    // QEP measurements
    uint32_t position;
    uint16_t period;
    uint16_t statusErrorCounter;

    uint32_t interruptNb;

    bool    calibMode;
} MotorDebug;

// Hardware Abstraction Layer structure
typedef struct _HAL_
{
    const GPIO_TypeDef* gpioHandler;
    const ADC_TypeDef*  adcHandler;
    const ePWM_TypeDef* epwmHandler;
    const eQEP_TypeDef* eqepHandler;
    const SPI_TypeDef*  spiHandler;
    const DMA_TypeDef*  dmaHandler;
//    const CLA_TypeDef*  claHandler;
} HAL;

typedef struct _ACQ_OFFSET_
{
    float32_t ia;
    float32_t ib;
    float32_t ic;
    float32_t vbus;
    float32_t va;
    float32_t vb;
    float32_t vc;
} ACQ_OFFSET;

typedef struct _ACQ_
{
    int16_t ia;
    int16_t ib;
    int16_t ic;
    int16_t vbus;
    int16_t va;
    int16_t vb;
    int16_t vc;
    int16_t vpot;
} ACQ;

// Motor Hardware Abstraction Layer structure
typedef struct _MOTOR_ACQ_
{
    ACQ_OFFSET offset;
    ACQ raw;
    uint32_t encoderPos;

    bool dataValid[2];
    bool offsetCal;

} MOTOR_ACQ;

/*
 * Function prototypes
 */

void HAL_init(HAL*);
extern void HAL_Interrupts_init();

extern void HAL_setupInterrupts();
extern void HAL_enableInterrupts();

// ADC interrupt service routine
extern __interrupt void adc1ISR(void);
extern __interrupt void adc2ISR(void);
extern __interrupt void adc3ISR(void);
extern __interrupt void adc4ISR(void);

void ADC_offsetCalibration(MOTOR_ACQ*);

#endif
