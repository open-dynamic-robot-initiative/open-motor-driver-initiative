/*
 * Filename: hal.h
 * Description: Header file for the Hardware Abstraction Layer
 */

#ifndef __HAL_H__
#define __HAL_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "f2838x_device.h"
#include "device.h"

#include "uomodri_user_defines.h"

/***********************************************************************
 * ENUMARATIONS
 ***********************************************************************/
typedef enum __sci_MultiProcCtrl_enum__
{
    SCI_MULTI_PROC_NONE = 0,
    SCI_MULTI_PROC_ADDR = 1,
    SCI_MULTI_PROC_IDLE = 2
} sci_MultiProcCtrl_e;

typedef enum __clb_RS485AddrDef_enum__
{
    RS485_CLB_USE_NO_ADDRESS    = 0,
    RS485_CLB_USE_UID_ADDRESS   = 1,
    RS485_CLB_FORCE_ADDRESS     = 2
} clb_RS485AddrDef_e;

/***********************************************************************
 * HARDWARE ABSTRACTION LAYER (HAL) CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  epwm_tb_t
 * @brief   ePWM - Time Base submodule structure definition.
 */
typedef struct __epwm_time_base_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            phaseCount;                 /*!< Phase counter. Sets offset (min value). */
    uint16_t                            syncOutputMode;             /*!< Output sync signal generation. \see EPWM_SyncOutPulseMode */
    EPWM_SyncInPulseSource              syncInputSrc;               /*!< Select the sync input signal.*/
} epwm_tb_t;

/**
 * @struct  epwm_cc_t
 * @brief   ePWM - Counter Compare submodule structure definition.
 */
typedef struct __epwm_counter_compare_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            compCount;                  /*!< Compare reference register. */
    EPWM_CounterCompareModule           compModule;                 /*!< Compare module (A, B, C or D). \see EPWM_CounterCompareModule */
} epwm_cc_t;

/**
 * @struct  epwm_aq_t
 * @brief   ePWM - Action Qualifier submodule structure definition.
 */
typedef struct __epwm_action_qualifier_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            actionsEvt;                 /*!< Actions associated to events matching detection. \see EPWM_ActionQualifierEventAction */
    EPWM_ActionQualifierOutputModule    epwmOutput;                 /*!< Output channel output (A or B). \see EPWM_ActionQualifierOutputModule */
} epwm_aq_t;

/**
 * @struct  epwm_db_t
 * @brief   ePWM - DeadBand Unit submodule structure definition.
 */
typedef struct __epwm_deadband_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            redCount;                   /*!< Rising Edge Delay counter. */
    uint16_t                            fedCount;                   /*!< Falling Edge Delay counter. */
    bool_t                              DelayMode;                  /*!< Enable deadband unit. */
} epwm_db_t;

/**
 * @struct  epwm_pc_t
 * @brief   ePWM - Chopper submodule structure definition.
 */
typedef struct __epwm_chopper_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            dutyCycleCount;             /*!< Programmable chopping (carrier) frequency */
    uint16_t                            freqDiv;
    uint16_t                            firstPulseWidth;            /*!< Programmable duty cycle of second and subsequent pulses */
} epwm_pc_t;

/**
 * @struct  epwm_et_t
 * @brief   ePWM - ADC SOC events trigger submodule structure definition.
 */
typedef struct __epwm_adc_soc_event_trigger_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            adcSOCEvtCount;             /*!< Delays by N events before generating an ADC SOC event. */
    EPWM_ADCStartOfConversionType       adcSOCType;                 /*!< ADC Start of Conversion (SOC) target \em EPWM_SOC_A or \em EPWM_SOC_B */
    EPWM_ADCStartOfConversionSource     adcSOCSource;               /*!< PWM events source for a ADC SOC. \see EPWM_ADCStartOfConversionSource */
} epwm_et_t;

/**
 * @struct  epwm_it_t
 * @brief   ePWM - Interrupts Events submodule structure definition.
 */
typedef struct __epwm_interrupt_events_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            intEvtCount;                /*!< Delays by N events before generating an IT. */
    uint16_t                            intSource;                  /*!< PWM events source IT. <em> EPWM_INT_TBCTR_ZERO, _PERIOD, _ZERO_OR_PERIOD, _U_CMPx, _D_CMPx,...</em>  */
} epwm_it_t;

/**
 * @struct  epwm_cfg_t
 * @brief   ePWM global Hardware Abstraction Layer structure
 */
typedef struct __epwm_config_t__
{
    const uint32_t                      epwmBase;                   /*!< ePWM peripheral address. */
    const epwm_tb_t*    const           p_epwmTimeBase;
    const epwm_cc_t*    const           p_epwmCounterCompare;
    const epwm_aq_t*    const           p_epwmActionQualifier;
    const epwm_db_t*    const           p_epwmDeadband;
    const epwm_pc_t*    const           p_epwmProgChopper;
    const epwm_et_t*    const           p_epwmAdcSocEventTrigger;
    const epwm_it_t*    const           p_epwmInterruptsEvent;
    const uint32_t                      epwmAdcSocSrcExtern;
} epwm_cfg_t;

/**
 * @struct  eqep_cfg_t
 * @brief   eQEP global Hardware Abstraction Layer structure
 */
typedef struct __eqep_config_t__
{
    const uint32_t                      eqepBase;                   /*!< eQEP peripheral address. */
    const uint32_t                      eqepMaxResolution;
    const uint32_t                      eqepUnitTimerPeriod;
    const uint16_t                      eqepConfig;
    const EQEP_StrobeSource             eqepStrobeSrc;
} eqep_cfg_t;

/**
 * @struct  spi_cfg_t
 * @brief   SPI global Hardware Abstraction Layer structure
 */
typedef struct __spi_config_t__
{
    const uint32_t                      spiBase;                    /*!< ADC peripheral address. */
    const uint32_t                      bitRate;
    const uint16_t                      dataWidth;
    const SPI_Mode                      mode;
    const SPI_TransferProtocol          protocol;
    const SPI_TxFIFOLevel               txLevel;
    const SPI_RxFIFOLevel               rxLevel;
    const uint16_t                      intSrc;
    const bool_t                        highSpeedMode;
    const bool_t                        intEnable;                  /*!< GPIO can be configured as IT source */
} spi_cfg_t;

/**
 * @struct  sci_cfg_t
 * @brief   SCI/UART global Hardware Abstraction Layer structure
 */
typedef struct __sci_config_t__
{
    uint32_t                            sciBase;                    /*!< ADC peripheral address. */
    uint32_t                            bitRate;
    uint32_t                            sciMode;
    SCI_TxFIFOLevel                     txLevel;
    SCI_RxFIFOLevel                     rxLevel;
    uint32_t                            intSrc;
    bool_t                              intEnable;                  /*!< GPIO can be configured as IT source */
    sci_MultiProcCtrl_e                 multiProcMode;
} sci_cfg_t;

/**
 * @struct  dma_cfg_t
 * @brief   DMA global Hardware Abstraction Layer structure
 */
typedef struct __dma_config_t__
{
    const uint32_t                      dmaChBase;
    const void*                         p_srcAddr;
    const void*                         p_dstAddr;
    const uint32_t                      transferSize;
    const uint32_t                      srcWrapSize;
    const uint32_t                      dstWrapSize;
    const uint32_t                      config;
    const uint16_t                      burstSize;
    const int16_t                       srcStep;
    const int16_t                       dstStep;
    const int16_t                       srcWrapStep;
    const int16_t                       dstWrapStep;
    const DMA_Trigger                   trigger;
    const DMA_InterruptMode             intMode;
    const bool_t                        intEnable;
} dma_cfg_t;

/**
 * @struct  cla_cfg_t
 * @brief   CLA global Hardware Abstraction Layer structure - To be updated
 */
typedef struct __cla_config_t__
{
    uint32_t                            progRAM;
    uint32_t                            dataRAM;
    uint16_t                            claTaskFlg;
    CLA_TaskNumber                      claTaskNum;
    CLA_MVECTNumber                     claIntVect;
    CLA_Trigger                         claTrigSrc;
    void                                (*p_claFunc)(void);
    bool_t                              claForceTask;
} cla_cfg_t;

/**
 * @struct  clb_cfg_t
 * @brief   CLB global Hardware Abstraction Layer structure
 */
typedef struct __clb_config_t__
{
    clb_RS485AddrDef_e                  clb_Addr1Spec;
    clb_RS485AddrDef_e                  clb_Addr2Spec;
    uint32_t                            clb_Addr1Force;
    uint32_t                            clb_Addr2Force;
} clb_cfg_t;

/**
 * @struct  gpio_cfg_t
 * @brief   GPIO structure definition for all IOs except analog ones.
 */
typedef struct __gpio_config_t__
{
    const uint32_t                      pinNum;                     /*!< GPIO pin number (GPIO0 - GPIO168). */
    const uint32_t                      fctMux;                     /*!< GPIO configuration functions (peripheral affectation matrix). \see pin_map.h */
    const uint32_t                      pinType;                    /*!< GPIO pad configuration :
                                                                        - \em GPIO_PIN_TYPE_STD (Push-pull output or floating input),
                                                                        - \em GPIO_PIN_TYPE_PULLUP (Pull-up enable for input),
                                                                        - \em GPIO_PIN_TYPE_INVERT (Invert polarity on input),
                                                                        - \em GPIO_PIN_TYPE_OD (Open-drain on output) */
    const GPIO_CoreSelect               coreSelect;                 /*!< GPIO core dependency.
                                                                        - \em GPIO_CORE_CPU1,
                                                                        - \em GPIO_CORE_CPU1_CLA1,
                                                                        - \em GPIO_CORE_CPU2,
                                                                        - \em GPIO_CORE_CPU2_CLA1 */
    const GPIO_Direction                direction;                  /*!< GPIO direction :
                                                                        - \em GPIO_DIR_MODE_IN,
                                                                        - \em GPIO_DIR_MODE_OUT */
    const GPIO_QualificationMode        samplingInMode;             /*!< GPIO input sampling mode :
                                                                        - \em GPIO_QUAL_SYNC,
                                                                        - \em GPIO_QUAL_3SAMPLE,
                                                                        - \em GPIO_QUAL_6SAMPLE,
                                                                        - \em GPIO_QUAL_ASYNC */
    const GPIO_AnalogMode               analogModeEnable;           /*!< GPIO is dedicated to USB */
    const GPIO_ExternalIntNum           extIntNum;                  /*!< GPIO external interrupt source number */
    const GPIO_IntType                  intType;                    /*!< GPIO interrupt event source */
    const bool_t                        intEnable;                  /*!< GPIO can be configured as IT source */
    const bool_t                        setValue;                   /*!< GPIO initial value if defined as output. */
} gpio_cfg_t;

/**
 * @struct  adc_ini_t
 * @brief   ADC initial configuration (start-of-conversion) handler
 */
typedef struct __adc_initalization_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const ADC_Channel                   adcChannel;                 /*!< ADC sampling channel. */
    const ADC_SOCNumber                 adcSOCNum;                  /*!< ADC Start-Of-Conversion (SOC) number (0-15). */
    const ADC_Trigger                   adcTrigSrc;                 /*!< ADC SOC trigger source. */
} adc_ini_t;

/**
 * @struct  adc_acq_t
 * @brief   ADC acquisition results (base and Post-Processing Block) configuration handler
 */
typedef struct __adc_acquisitions_results_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const uint32_t                      adcResultBase;
    const ADC_SOCNumber                 adcSOCNum;
    const ADC_PPBNumber                 adcPPBNum;
} adc_acq_t;

/**
 * @struct  adc_int_t
 * @brief   ADC Interrupt source configuration handler
 */
typedef struct __adc_interrupts_events_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const ADC_SOCNumber                 intTrig;                    /*!< ADC End-Of-Conversion (EOC) number (0-15). */
    const ADC_IntNumber                 adcIntNum;                  /*!< IT source (0-3) associated to ADC \& SOC number. */
} adc_int_t;

/**
 * @struct  adc_cfg_t
 * @brief   ADC module configuration handler
 */
typedef struct __Aadc_config_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const adc_ini_t*    const           p_adcIni;                   /*!< Pointer on ADC start-of-conversion configuration handler */
    const adc_acq_t*    const           p_adcAcq;                   /*!< Pointer on ADC result configuration handler */
    const adc_int_t*    const           p_adcInt;                   /*!< Pointer on ADC Interrupt source configuration handler */
} adc_cfg_t;

/**
 * @struct  int_cfg_t
 * @brief   Interrupt source configuration handler
 */
typedef struct __interrupts_config_t__
{
    uint32_t                            intNum;
    uint16_t                            intAckGroup;
    void                                (*p_intHandler)(void);
} int_cfg_t;

/**
 * @struct  ipc_cfg_t
 * @brief   IPC module configuration handler
 */
typedef struct __ipc_config_t__
{
    IPC_Type_t                          ipcType;
    uint32_t                            ipcBase;
    volatile IPC_MessageQueue_t*        p_ipcMsgQ;
    uint32_t                            ipcIntMsgQL;
    uint32_t                            ipcIntMsgQR;
    uint32_t                            ipcFlag;
    uint32_t                            ipcInt;
    void                                (*p_ipcIntHandler)(void);
    bool_t                              ipcMsgQEnable;
    bool_t                              ipcIntEnable;
} ipc_cfg_t;

/**
 * @struct  tmr_cfg_t
 * @brief   Timer0/1/2 module configuration handler
 */
typedef struct __cpu_timer_config_t__
{
    uint32_t                            cpuTimerBase;
    uint32_t                            periodCount;                /*!< in uSeconds */
    uint16_t                            prescaler;
    bool_t                              intEnable;
} tmr_cfg_t;

/**
 * @struct  hal_cfg_t
 * @brief   General HAL structure handler for the project
 */
typedef struct __hal_config_t__
{
    const gpio_cfg_t*   const           p_gpioHandle;
    const adc_cfg_t*    const           p_adcHandle;
    const epwm_cfg_t*   const           p_epwmHandle;
    const eqep_cfg_t*   const           p_eqepHandle;
    const sci_cfg_t*    const           p_sciHandle;
    const spi_cfg_t*    const           p_spiHandle;
    const ipc_cfg_t*    const           p_ipcHandle;
    const dma_cfg_t*    const           p_dmaHandle;
    const tmr_cfg_t*    const           p_timerHandle;
    const cla_cfg_t*    const           p_claHandle;
    const clb_cfg_t*    const           p_clbHandle;
} hal_cfg_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void HAL_ini(const hal_cfg_t*);
void HAL_INT_ini(const int_cfg_t*, void (*)(void));
void HAL_DMA_ini(const dma_cfg_t*);

#endif /* __HAL_H__ */
