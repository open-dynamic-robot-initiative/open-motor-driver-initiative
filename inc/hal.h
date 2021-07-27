/*
 * Filename: hal.h
 * Description: Header file for the Hardware Abstraction Layer
 */

#ifndef __HAL_H__
#define __HAL_H__

#include "driverlib.h"
#include "device.h"
#include "f2838x_device.h"

#include "uomodri_user_defines.h"

/**
 * @enum    EPWM_DelayModeEnable
 * @brief   Enable or disable Deadband delay unit.
 */
typedef enum
{
    EPWM_DELAY_DISABLE  = 0,                        /*!< Disable/bypass Deadband delay mode. */
    EPWM_DELAY_ENABLE   = 1                         /*!< Enable Deadband delay mode. */
} EPWM_DelayModeEnable;

/**
 * @struct  ePWM_TB_TypeDef
 * @brief   ePWM - Time Base submodule structure definition.
 */
typedef struct  _ePWM_Time_Base_TypeDef_
{
    uint32_t    epwmBase;                           /*!< ePWM peripheral address. */
    uint16_t    phaseCount;                         /*!< Phase counter. Sets offset (min value). */
    //EPWM_TimeBaseCountMode counterMode;             /*!< Counting direction (up-count, down-count or up and down count). \see EPWM_TimeBaseCountMode */
    uint16_t    syncOutputMode;                     /*!< Output sync signal generation. \see EPWM_SyncOutPulseMode */
} ePWM_TB_TypeDef;

/**
 * @struct  ePWM_CC_TypeDef
 * @brief   ePWM - Counter Compare submodule structure definition.
 */
typedef struct _ePWM_Counter_Compare_TypeDef_
{
    uint32_t                    epwmBase;           /*!< ePWM peripheral address. */
    uint16_t                    compCount;          /*!< Compare reference register. */
    EPWM_CounterCompareModule   compModule;         /*!< Compare module (A, B, C or D). \see EPWM_CounterCompareModule */
} ePWM_CC_TypeDef;

/**
 * @struct  ePWM_AQ_TypeDef
 * @brief   ePWM - Action Qualifier submodule structure definition.
 */
typedef struct _ePWM_Action_Qualifier_TypeDef_
{
    uint32_t                            epwmBase;   /*!< ePWM peripheral address. */
    uint16_t                            actionsEvt; /*!< Actions associated to events matching detection. \see EPWM_ActionQualifierEventAction */
    EPWM_ActionQualifierOutputModule    epwmOutput; /*!< Output channel output (A or B). \see EPWM_ActionQualifierOutputModule */
} ePWM_AQ_TypeDef;

/**
 * @struct  ePWM_DB_TypeDef
 * @brief   ePWM - DeadBand Unit submodule structure definition.
 */
typedef struct _ePWM_Deadband_TypeDef_
{
    uint32_t                epwmBase;               /*!< ePWM peripheral address. */
    uint16_t                redCount;               /*!< Rising Edge Delay counter. */
    uint16_t                fedCount;               /*!< Falling Edge Delay counter. */
    EPWM_DelayModeEnable    DelayMode;              /*!< Enable deadband unit. */
} ePWM_DB_TypeDef;

/**
 * @struct  ePWM_PC_TypeDef
 * @brief   ePWM - Chopper submodule structure definition.
 */
typedef struct _ePWM_Chopper_TypeDef_
{
    uint32_t    epwmBase;                           /*!< ePWM peripheral address. */
    uint16_t    dutyCycleCount;                     /*!< Programmable chopping (carrier) frequency */
    uint16_t    freqDiv;
    uint16_t    firstPulseWidth;                    /*!< Programmable duty cycle of second and subsequent pulses */
} ePWM_PC_TypeDef;

/**
 * @struct  ePWM_ET_TypeDef
 * @brief   ePWM - ADC SOC events trigger submodule structure definition.
 */
typedef struct _ePWM_ADC_SOC_Event_Trigger_TypeDef_
{
    uint32_t                        epwmBase;       /*!< ePWM peripheral address. */
    uint16_t                        adcSOCEvtCount; /*!< Delays by N events before generating an ADC SOC event. */
    EPWM_ADCStartOfConversionType   adcSOCType;     /*!< ADC Start of Conversion (SOC) target \em EPWM_SOC_A or \em EPWM_SOC_B */
    EPWM_ADCStartOfConversionSource adcSOCSource;   /*!< PWM events source for a ADC SOC. \see EPWM_ADCStartOfConversionSource */
} ePWM_ET_TypeDef;

/**
 * @struct  ePWM_IT_TypeDef
 * @brief   ePWM - Interrupts Events submodule structure definition.
 */
typedef struct _ePWM_Interrupts_Event_TypeDef_
{
    uint32_t epwmBase;                              /*!< ePWM peripheral address. */
    uint16_t intEvtCount;                           /*!< Delays by N events before generating an IT. */
    uint16_t intSource;                             /*!< PWM events source IT. <em> EPWM_INT_TBCTR_ZERO, _PERIOD, _ZERO_OR_PERIOD, _U_CMPx, _D_CMPx,...</em>  */
} ePWM_IT_TypeDef;

/**
 * @struct  ePWM_TypeDef
 * @brief   ePWM global Hardware Abstraction Layer structure
 */
typedef struct _ePWM_Cfg_TypeDef_
{
    const uint32_t          epwmBase;               /*!< ePWM peripheral address. */
    const ePWM_TB_TypeDef*  epwmTimeBase;
    const ePWM_CC_TypeDef*  epwmCaptureCompare;
    const ePWM_AQ_TypeDef*  epwmActionQualifier;
    const ePWM_DB_TypeDef*  epwmDeadband;
    const ePWM_PC_TypeDef*  epwmProgChopper;
    const ePWM_ET_TypeDef*  epwmAdcSocEventTrigger;
    const ePWM_IT_TypeDef*  epwmInterruptsEvent;
    const uint32_t          epwmAdcSocSrcExtern;
} ePWM_TypeDef;

/**
 * @struct  eQEP_TypeDef
 * @brief   eQEP global Hardware Abstraction Layer structure
 */
typedef struct _eQEP_Cfg_TypeDef_
{
    uint32_t            eqepBase;                   /*!< eQEP peripheral address. */
    uint32_t            eqepMaxResolution;
    uint32_t            eqepUnitTimerPeriod;
    uint16_t            eqepConfig;
    EQEP_StrobeSource   eqepStrobeSrc;
} eQEP_TypeDef;

/**
 * @struct  SPI_TypeDef
 * @brief   SPI global Hardware Abstraction Layer structure
 */
typedef struct _SPI_Cfg_TypeDef_
{
    uint32_t                spiBase;                /*!< ADC peripheral address. */
    uint32_t                bitRate;
    uint16_t                dataWidth;
    SPI_Mode                mode;
    SPI_TransferProtocol    protocol;
    SPI_TxFIFOLevel         txLevel;
    SPI_RxFIFOLevel         rxLevel;
} SPI_TypeDef;

/**
 * @struct  DMA_TypeDef
 * @brief   DMA global Hardware Abstraction Layer structure
 */
typedef struct _DMA_Cfg_TypeDef_
{
    uint32_t    dmaChBase;
    uint32_t    transferSize;//    uint32_t dmaRxBase;
    uint32_t    srcWrapSize;
    uint32_t    destWrapSize;
    uint32_t    config;
    uint16_t    burstSize;
    int16_t     destStep;
    int16_t     srcStep;
    const void* destAddr;
    const void* srcAddr;
    DMA_Trigger trigger;
} DMA_TypeDef;

/**
 * @struct  CLA_TypeDef
 * @brief   CLA global Hardware Abstraction Layer structure - To be updated
 */
typedef struct _CLA_Cfg_TypeDef_
{
    uint32_t claBase;
    //TODO
} CLA_TypeDef;

/**
 * @struct  GPIO_TypeDef
 * @brief   GPIO structure definition for all IOs except analog ones.
 */
typedef struct _GPIO_Cfg_TypeDef_
{
    uint32_t                pinNumber;              /*!< GPIO pin number (GPIO0 - GPIO168). */
    uint32_t                fctMux;                 /*!< GPIO configuration functions (peripheral affectation matrix). \see pin_map.h */
    uint32_t 	            pinType;                /*!< GPIO pad configuration :
    - \em GPIO_PIN_TYPE_STD (Push-pull output or floating input),
    - \em GPIO_PIN_TYPE_PULLUP (Pull-up enable for input),
    - \em GPIO_PIN_TYPE_INVERT (Invert polarity on input),
    - \em GPIO_PIN_TYPE_OD (Open-drain on output) */
    GPIO_CoreSelect         coreSelect;             /*!< GPIO core dependency.
    - \em GPIO_CORE_CPU1,
    - \em GPIO_CORE_CPU1_CLA1,
    - \em GPIO_CORE_CPU2,
    - \em GPIO_CORE_CPU2_CLA1 */
    GPIO_Direction          direction;              /*!< GPIO direction :
    - \em GPIO_DIR_MODE_IN,
    - \em GPIO_DIR_MODE_OUT */
    GPIO_QualificationMode	samplingInMode;         /*!< GPIO input sampling mode :
    - \em GPIO_QUAL_SYNC,
    - \em GPIO_QUAL_3SAMPLE,
    - \em GPIO_QUAL_6SAMPLE,
    - \em GPIO_QUAL_ASYNC */
    GPIO_ExternalIntNum     extIntNum;              /*!< GPIO external interrupt source number */
    GPIO_IntType            intType;                /*!< GPIO interrupt event source */
    bool                    intEnable;              /*!< GPIO can be configured as IT source */
    bool                    setValue;               /*!< GPIO initial value if defined as output. */
//    GPIO_ExternalIntNum extIntNum;
} GPIO_TypeDef;

/**
 * @struct  ADC_Init_TypeDef
 * @brief   ADC initial configuration (start-of-conversion) handler
 */
typedef struct _ADC_INIT_TypeDef_
{
    uint32_t        adcBase;                        /*!< ADC peripheral address. */
    ADC_Channel     adcChannel;                     /*!< ADC sampling channel. */
    ADC_SOCNumber   adcSOCNumber;                   /*!< ADC Start-Of-Conversion (SOC) number (0-15). */
    ADC_Trigger     adcTriggerSrc;                  /*!< ADC SOC trigger source. */
} ADC_INIT_TypeDef;

/**
 * @struct  ADC_ACQ_TypeDef
 * @brief   ADC acquisition results (base and Post-Processing Block) configuration handler
 */
typedef struct _ADC_ACQ_TypeDef_
{
    uint32_t            adcBase;                    /*!< ADC peripheral address. */
    volatile uint16_t*  adcResultReg;
    //volatile uint16_t*  adcPPBResultReg;
//    uint16_t adcCalib;
//    uint16_t adcOffset;
    ADC_SOCNumber       adcSOCNumber;
    ADC_PPBNumber       adcPPBNumber;
} ADC_ACQ_TypeDef;

/**
 * @struct  ADC_IT_TypeDef
 * @brief   ADC Interrupt source configuration handler
 */
typedef struct _ADC_IT_TypeDef_
{
    uint32_t        adcBase;                        /*!< ADC peripheral address. */
    ADC_SOCNumber   adcEOCNumber;                   /*!< ADC End-Of-Conversion (EOC) number (0-15). */
    ADC_IntNumber   adcIntNumber;                   /*!< IT source (0-3) associated to ADC \& SOC number. */
} ADC_IT_TypeDef;

/**
 * @struct  ADC_TypeDef
 * @brief   ADC module configuration handler
 */
typedef struct _ADC_TypeDef_
{
    const uint32_t          adcBase;                /*!< ADC peripheral address. */
    const uint32_t          adcResultBase;          /*!< ADC results address. */
    const ADC_INIT_TypeDef* adcInit;                /*!< Pointer on ADC start-of-conversion configuration handler */
    const ADC_ACQ_TypeDef*  adcAcq;                 /*!< Pointer on ADC result configuration handler */
    const ADC_IT_TypeDef*   adcIt;                  /*!< Pointer on ADC Interrupt source configuration handler */
} ADC_TypeDef;

typedef struct _IT_TypeDef_
{
//    const ID_ENUM   motorID;
    uint32_t    intNum;
    uint32_t    intPeriphBase;
    uint16_t    intPeriphSrc;
    uint16_t    intCpuGroup;
    uint16_t    intAckGroup;
    void        (*handler)(void);
} IT_TypeDef;

/**
 * @struct  HAL_
 * @brief   General HAL structure handler for the project
 */
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

// Motor Hardware Abstraction Layer structure
typedef struct _HAL_MOTOR_CFG_
{
    const ePWM_CC_TypeDef*  p_pwmCntCmp[3];
    const ADC_ACQ_TypeDef*  p_iAcq[3];
    const ADC_ACQ_TypeDef*  p_vAcq[3];
    const ADC_ACQ_TypeDef*  p_vbusAcq;
    const ADC_IT_TypeDef*   p_ITAcq;
    const eQEP_TypeDef*     p_qep;
    const ID_ENUM           motorID;
} HAL_MOTOR_CFG;

/*
 * Function prototypes
 */

void HAL_init(const HAL*);
void HAL_IT_init(const IT_TypeDef*);

// ADC interrupt service routine
//extern __interrupt void adc1ISR(void);
//extern __interrupt void adc2ISR(void);
//extern __interrupt void adc3ISR(void);
//extern __interrupt void adc4ISR(void);
//extern __interrupt void isrMotor1(void);

void HAL_ADC_offsetCalib(const HAL_MOTOR_CFG*);

#endif
