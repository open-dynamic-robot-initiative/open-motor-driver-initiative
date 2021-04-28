/*
 * File name: hal.c
 * Description: Source file containing Hardware Abstraction Layer source code
 */

#include "hal.h"

#include "uomodri_config_handlers.h"

// ADC interrupt service routine
extern __interrupt void adc1ISR(void);
extern __interrupt void adc2ISR(void);
extern __interrupt void adc3ISR(void);
extern __interrupt void adc4ISR(void);

/* Static function signatures */
// Device HAL
static void HAL_Bases_init(HAL*);
static void HAL_GPIO_init(HAL*);
static void HAL_ADC_init(HAL*);
static void HAL_ePWM_init(HAL*);
static void HAL_eQEP_init(HAL*);
static void HAL_SPI_init(HAL*);
static void HAL_DMA_init(HAL*);
//static void HAL_CLA_init(HAL*);


// Function definitions
/**
 * @brief       Initialize all peripherals necessaries.
 * @param[in]   *hal    Pointer on the general HAL structure definition
 */
void HAL_init(HAL* hal)
{
    // Initialize Hardware Abstraction Layer structure
    HAL_Bases_init(hal);
    // Initialize GPIO pins
    HAL_GPIO_init(hal);
    // Initialize CLA
//    HAL_CLA_init(hal);
    // Initialize ADC
    HAL_ADC_init(hal);
    // Initialize ePWM
    HAL_ePWM_init(hal);
    // Initialize eQEP
    HAL_eQEP_init(hal);
    // Initialize DMA
    HAL_DMA_init(hal);
    // Initialize SPIA device
    HAL_SPI_init(hal);
}

/**
 * @brief       Link HAL structure definition and initialization parameters.
 * @param[in]   *hal    Pointer on the general HAL structure definition
 */
static void HAL_Bases_init(HAL* hal)
{
    // GPIO initialization
    hal->gpioHandler = GpioPinsCfgList;
    // ADC initialization
    hal->adcHandler = AdcCfgList;
    // PWM initialization
    hal->epwmHandler = PwmCfgList;
    // QEP initialization
    hal->eqepHandler = QepCfgList;
    // DMA initialization
    hal->dmaHandler = NULL;
    // SPI bus initialization
    hal->spiHandler = spiCfgList;
}

/**
 * @brief       GPIO initialization function.
 * @param[in]   *hal    Pointer on the general HAL structure
 */
static void HAL_GPIO_init(HAL* hal)
{
    const GPIO_TypeDef* p_gpio = hal->gpioHandler;
    uint32_t            pinNum = p_gpio->pinNumber;

    while(pinNum != 0xFFFFFFFF)
    {
        GPIO_setMasterCore(pinNum, p_gpio->coreSelect);
        GPIO_setPinConfig(p_gpio->fctMux);
        GPIO_setDirectionMode(pinNum, p_gpio->direction);
        GPIO_setPadConfig(pinNum, p_gpio->pinType);
        if(p_gpio->direction == GPIO_DIR_MODE_IN)
            GPIO_setQualificationMode(pinNum, p_gpio->samplingInMode);
        else
            GPIO_writePin(pinNum, (uint32_t) p_gpio->setValue);
        pinNum = (++p_gpio)->pinNumber;
    }
}

/**
 * @brief       ADC initialization function.
 * @param[in]   *hal    Pointer on the general HAL structure
 */
static void HAL_ADC_init(HAL* hal)
{
    const ADC_TypeDef*     p_adcCfg = hal->adcHandler;
    const ADC_SOC_TypeDef* p_adcSOC = p_adcCfg->adcSoc;
#ifdef ADC_PPB_ENABLE
    const ADC_PPB_TypeDef* p_adcPPB = p_adcCfg->adcPpb;
#endif
    const ADC_IT_TypeDef*  p_adcIT  = p_adcCfg->adcIt;
    uint32_t               adcBase  = p_adcCfg->adcBase;

    while(adcBase != 0xFFFFFFFF)
    {
        /* Sets ADC clock to 50MHz (ADCCLK (derived from PERx.SYSCLK))
         * Set converting input mode : "ADC_MODE_SINGLE_ENDED" or "ADC_MODE_DIFFERENTIAL"
         * Set converting resolution : "ADC_RESOLUTION_12BIT" or "ADC_RESOLUTION_16BIT"
         * Set IT raising  : "ADC_PULSE_END_OF_ACQ_WIN" or "ADC_PULSE_END_OF_CONV"
         * Enables ADC converter
         */
        ADC_setPrescaler(adcBase, PERxSYSCLK_TO_ADCCLK_DIV);
        ADC_setMode(adcBase, ADC_RESOLUTION, ADC_SIGNAL_MODE);
        ADC_setInterruptPulseMode(adcBase, ADC_PULSE_END_MODE);
        ADC_enableConverter(adcBase);

        // Let some time to power up ADCs
        DEVICE_DELAY_US(1500U);

        while(p_adcSOC->adcBase == adcBase)
        {
            /* Set priorities according round-robin
             * Set the input channel, SOC number, trigger source & S/H window
             * SOC source will be PWM, no IT.
             */
            ADC_setSOCPriority(adcBase, ADC_SOC_PRIORITY);
            ADC_setupSOC(adcBase, p_adcSOC->adcSOCNumber, p_adcSOC->adcTriggerSrc, p_adcSOC->adcChannel, ADC_SAMPLING_WINDOW);
            ADC_setInterruptSOCTrigger(adcBase, p_adcSOC->adcSOCNumber, ADC_INT_SOC_TRIGGER_NONE);
            p_adcSOC++;
        }

#ifdef ADC_PPB_ENABLE
        while(p_adcPPB->adcBase == adcBase)
        {
            /* Post processing block initialization, associate with ADC_SOC_NUMBER0
             * Initialize PPB offset to zero
             */
            ADC_setupPPB(adcBase, p_adcPPB->adcPPBNumber, p_adcPPB->adcSOCNumber);
            ADC_setPPBCalibrationOffset(adcBase, p_adcPPB->adcPPBNumber, p_adcPPB->adcCalib);
            ADC_setPPBReferenceOffset(adcBase, p_adcPPB->adcPPBNumber, p_adcPPB->adcOffset);
            p_adcPPB++;
        }
#endif

        while(p_adcIT->adcBase == adcBase)
        {
            ADC_setInterruptSource(adcBase, p_adcIT->adcIntNumber, p_adcIT->adcEOCNumber);
            ADC_clearInterruptStatus(adcBase, p_adcIT->adcIntNumber);
            ADC_enableInterrupt(adcBase, p_adcIT->adcIntNumber);
            p_adcIT++;
        }
        adcBase = (++p_adcCfg)->adcBase;
    }
}

/**
 * @brief       ePWM global peripheral initialization function.
 * @param[in]   *hal    Pointer on the general HAL structure
 */
static void HAL_ePWM_init(HAL* hal)
{
    const ePWM_TypeDef*    p_pwmCfg             = hal->epwmHandler;
    const ePWM_TB_TypeDef* p_pwmTimeBase        = p_pwmCfg->epwmTimeBase;
    const ePWM_CC_TypeDef* p_pwmCounterCompare  = p_pwmCfg->epwmCaptureCompare;
    const ePWM_AQ_TypeDef* p_pwmActionQualifier = p_pwmCfg->epwmActionQualifier;
    const ePWM_DB_TypeDef* p_pwmDeadBand        = p_pwmCfg->epwmDeadband;
    const ePWM_PC_TypeDef* p_pwmProgChopper     = p_pwmCfg->epwmProgChopper;
    const ePWM_ET_TypeDef* p_pwmEventTrigger    = p_pwmCfg->epwmEventTrigger;
    uint32_t               pwmBase              = p_pwmCfg->epwmBase;

    // Disable global PWM clock during initialization
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
    // Set correct frequency for EPWMCLK (max = PLLSYSCLK / 2)
    SysCtl_setEPWMClockDivider(SYSCTL_EPWMCLK_DIV_2);

    // Check that main pointers are valid.
    if((p_pwmTimeBase != NULL) && (p_pwmCounterCompare != NULL) && (p_pwmActionQualifier != NULL) && (p_pwmDeadBand != NULL))
    {
        while(pwmBase != 0xFFFFFFFF)
        {
            /* Time-Base (TB) Submodule Registers
             * Set the time base clock and the high speed time base clock count prescaler
             * Defines Shadow load mode : "SHADOW_LOAD" or "DIRECT_LOAD"
             * Defines Shadow mode event : "EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO", "EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC" or "EPWM_SHADOW_LOAD_MODE_SYNC"
             * Set Global Shadow mode event
             * Set time base period register
             * Set phase shift register
             * Set initial value to time base counter
             * Set time base counter mode : "COUNTER_MODE_UP", "COUNTER_MODE_DOWN", "COUNTER_MODE_UP_DOWN" and "COUNTER_MODE_STOP_FREEZE"
             */
            while(p_pwmTimeBase->epwmBase == pwmBase)
            {
                EPWM_setClockPrescaler(pwmBase, EPWMCLK_TO_TBCLK_DIV, HRPWMCLK_TO_TBCLK_DIV);
                EPWM_setPeriodLoadMode(pwmBase, PWM_SHADOW_MODE_ET);
#if PWM_SHADOW_MODE_EN
                EPWM_selectPeriodLoadEvent(pwmBase, PWM_SHADOW_MODE_EVENT_ET);
                EPWM_setGlobalLoadTrigger(pwmBase, PWM_SHADOW_MODE_EVENT_GLOBAL);
                EPWM_enableGlobalLoad(pwmBase);
#else
                EPWM_disableGlobalLoad(pwmBase);
#endif
                EPWM_setTimeBasePeriod(pwmBase, PWM_TBPRD);
                (pwmBase == MOTOR1_PWM1_BASE) ? (EPWM_disablePhaseShiftLoad(pwmBase)) : (EPWM_enablePhaseShiftLoad(pwmBase));
                EPWM_setPhaseShift(pwmBase, p_pwmTimeBase->phaseCount);
                EPWM_setTimeBaseCounterMode(pwmBase, PWM_COUNTER_MODE);
                EPWM_setTimeBaseCounter(pwmBase, PWM_TBCTR);
                EPWM_enableSyncOutPulseSource(pwmBase, p_pwmTimeBase->syncOutputMode);
                p_pwmTimeBase++;
            }

            /* Counter Compare (CC) Submodule Registers
             * Scan all the Counter-Compare structure array.
             * Set or disable the shadow mode.
             * Set compare value associated to module.
             */
            while(p_pwmCounterCompare->epwmBase == pwmBase)
            {
#if PWM_SHADOW_MODE_EN
                EPWM_setCounterCompareShadowLoadMode(pwmBase, p_pwmCounterCompare->compModule, PWM_SHADOW_MODE_EVENT_CC);
#else
                EPWM_disableCounterCompareShadowLoadMode(pwmBase, p_pwmCounterCompare->compModule);
#endif
                EPWM_setCounterCompareValue(pwmBase, p_pwmCounterCompare->compModule, p_pwmCounterCompare->compCount);
                p_pwmCounterCompare++;
            }

            /* Action Qualifier (AQ) SubModule Registers
             * Scan all the Action Qualifier structure array.
             * Set or disable the shadow mode.
             * Setup Action qualifier event outputs.
             * Reinitialize the pointer for next PWM channel config.
             */
            while(p_pwmActionQualifier->epwmBase == pwmBase)
            {
                EPWM_ActionQualifierModule aqModule = (EPWM_ActionQualifierModule) p_pwmActionQualifier->epwmOutput;
#if PWM_SHADOW_MODE_EN
                EPWM_setActionQualifierShadowLoadMode(pwmBase, aqModule, PWM_SHADOW_MODE_EVENT_AQ);
#else
                EPWM_disableActionQualifierShadowLoadMode(pwmBase, aqModule);
#endif
                EPWM_setActionQualifierActionComplete(pwmBase, p_pwmActionQualifier->epwmOutput, p_pwmActionQualifier->actionsEvt);
                p_pwmActionQualifier++;
            }

            /* DeadBand Unit (DB) SubModule Registers
             * Sets Dead Band Counter clock rate
             * S1 & S0 : Dead Band delay bypass (FED & RED active)
             * S2 & S3 : Dead Band delay polarity inverting
             * S4 & S5 : RED & FED input affectation (use ePWMxA, ePWMxB or both)
             * S6 & S7 : Dead Band output swap
             * Set Rising Edge Deadband delay Falling Edge Deadband delay
             * Force chopper disable on all channels
             */
            while(p_pwmDeadBand->epwmBase == pwmBase)
            {
                // S1 & S0 : Dead Band delay bypass (FED & RED active)
                EPWM_setDeadBandDelayMode(pwmBase, EPWM_DB_RED, false);
                EPWM_setDeadBandDelayMode(pwmBase, EPWM_DB_FED, false);
                /* DeadBand Unit SubModule enable */
                if(p_pwmDeadBand->DelayMode == EPWM_DELAY_ENABLE)
                {
#if PWM_SHADOW_MODE_EN
                    EPWM_setDeadBandControlShadowLoadMode(pwmBase, PWM_SHADOW_MODE_EVENT_DB);
                    EPWM_setRisingEdgeDelayCountShadowLoadMode(pwmBase, PWM_SHADOW_MODE_EVENT_DB_RED);
                    EPWM_setFallingEdgeDelayCountShadowLoadMode(pwmBase, PWM_SHADOW_MODE_EVENT_DB_FED);
#else
                    EPWM_disableDeadBandControlShadowLoadMode(pwmBase);
                    EPWM_disableRisingEdgeDelayCountShadowLoadMode(pwmBase);
                    EPWM_disableFallingEdgeDelayCountShadowLoadMode(pwmBase);
#endif
                    /* Sets Dead Band Counter clock rate */
                    EPWM_setDeadBandCounterClock(pwmBase, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
                    /* S4 & S5 : RED & FED use ePWMxA as input */
                    EPWM_setRisingEdgeDeadBandDelayInput(pwmBase, EPWM_DB_INPUT_EPWMA);
                    EPWM_setFallingEdgeDeadBandDelayInput(pwmBase, EPWM_DB_INPUT_EPWMA);
                    // S1 & S0 : Dead Band delay bypass disable (FED & RED active)
                    EPWM_setDeadBandDelayMode(pwmBase, EPWM_DB_RED, true);
                    EPWM_setDeadBandDelayMode(pwmBase, EPWM_DB_FED, true);
                    // S2 & S3 : Dead Band delay polarity in Active High Complementary (AHC) - (S3 = 1, S2 = 0)
                    EPWM_setDeadBandDelayPolarity(pwmBase, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
                    EPWM_setDeadBandDelayPolarity(pwmBase, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
                    // Set Rising Edge Deadband delay Falling Edge Deadband delay
                    EPWM_setRisingEdgeDelayCount(pwmBase, p_pwmDeadBand->redCount);
                    EPWM_setFallingEdgeDelayCount(pwmBase, p_pwmDeadBand->fedCount);
                }
                // S6 & S7 : Dead Band output swap
                EPWM_setDeadBandOutputSwapMode(pwmBase, EPWM_DB_OUTPUT_A, false);
                EPWM_setDeadBandOutputSwapMode(pwmBase, EPWM_DB_OUTPUT_B, false);
                // Force disable chopper mode on all A & B channels
                EPWM_disableChopper(pwmBase);
                p_pwmDeadBand++;
            }

            /* Chopper SubModule Registers
             * Programmable chopping (carrier) frequency
             * Programmable pulse width of first pulse
             * Programmable duty cycle of second and subsequent pulses
             */
            while((p_pwmProgChopper != NULL) && (p_pwmProgChopper->epwmBase == pwmBase))
            {
                EPWM_enableChopper(pwmBase);
                EPWM_setChopperDutyCycle(pwmBase, p_pwmProgChopper->dutyCycleCount);
                EPWM_setChopperFreq(pwmBase, p_pwmProgChopper->freqDiv);
                EPWM_setChopperFirstPulseWidth(pwmBase, p_pwmProgChopper->firstPulseWidth);
                p_pwmProgChopper++;
            }

            /* Event-Trigger (ET) Submodule Registers
             * Disable ADC trigger & IT generation while initialization is in progress.
             */
            while((p_pwmEventTrigger != NULL) && (p_pwmEventTrigger->epwmBase == pwmBase))
            {
                // Disable ADC SOC events generation while init is in progress.
                EPWM_disableADCTriggerEventCountInit(pwmBase, p_pwmEventTrigger->adcSOCType);
                EPWM_disableADCTrigger(pwmBase, p_pwmEventTrigger->adcSOCType);
                // If adcSOCEvtCount == 0 -> no ADC SOC events are generated
                if(p_pwmEventTrigger->adcSOCEvtCount)
                {
                    EPWM_setADCTriggerSource(pwmBase, p_pwmEventTrigger->adcSOCType, p_pwmEventTrigger->adcSOCSource);
                    EPWM_setADCTriggerEventPrescale(pwmBase, p_pwmEventTrigger->adcSOCType, p_pwmEventTrigger->adcSOCEvtCount);
                    EPWM_clearADCTriggerFlag(pwmBase, p_pwmEventTrigger->adcSOCType);
                    EPWM_enableADCTrigger(pwmBase, p_pwmEventTrigger->adcSOCType);
                }
                // Disable IT generation while init is in progress.
                EPWM_disableInterruptEventCountInit(pwmBase);
                EPWM_disableInterrupt(pwmBase);
                // If IntEvtCount == 0 -> no Interrupts are generated
                if(p_pwmEventTrigger->intEvtCount)
                {
                    EPWM_setInterruptSource(pwmBase, p_pwmEventTrigger->intSource);
                    EPWM_setInterruptEventCount(pwmBase, p_pwmEventTrigger->intEvtCount);
                    EPWM_clearEventTriggerInterruptFlag(pwmBase);
                    EPWM_enableInterrupt(pwmBase);
                }
                p_pwmEventTrigger++;
            }
// Configure synchronization
//        EPWM_setCountModeAfterSync(pwmbase, EPWM_COUNT_MODE_UP_AFTER_SYNC);
//        EPWM_setSyncOutPulseMode(pwmbase, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
            pwmBase = (++p_pwmCfg)->epwmBase;
        }
        // End of PWM initialization. Enable global PWM clock.
        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
        EPWM_forceSyncPulse(MOTOR1_PWM1_BASE);
    }
    return;
}

/**
 * @brief       Quadrature encoder initialization function.
 * @param[in]   *hal    Pointer on the general HAL structure
 */
static void HAL_eQEP_init(HAL* hal)
{
    const eQEP_TypeDef* p_qepCfg = hal->eqepHandler;
    uint32_t            qepBase  = p_qepCfg->eqepBase;

    if(p_qepCfg != NULL)
    {
        while(qepBase != 0xFFFFFFFF)
        {
            /* Configure the decoder for up-count mode, counting both on rising & falling edges (quadrature encoders)
             * Position & watchdog counter, unit & capture timer are all unaffected by an emulation suspend.
             * Position counter latched on a unit time out and on rising edge of index pulse.
             * Position counter is reset when reaching max position (1 turn). on an index event.
             * Set unit timer frequency to PWM frequency for speed measurement.
             * Compare unit is useless.
             */
            EQEP_setDecoderConfig(qepBase, p_qepCfg->config);
            EQEP_setEmulationMode(qepBase, QEP_EMUL_MODE);
            EQEP_setLatchMode(qepBase, QEP_LATCH_MODE);
            EQEP_setPositionCounterConfig(qepBase, QEP_RESET_POSITION_MODE, QEP_RESET_POSITION_MAX);
            EQEP_disableUnitTimer(qepBase);
            EQEP_disableCompare(qepBase);
            EQEP_disableWatchdog(qepBase);
            EQEP_disableInterrupt(qepBase, EQEP_INT_GLOBAL);
            // Configure the edge capture unit
            EQEP_setCaptureConfig(qepBase, QEP_CAPTURE_CLOCK_DIV, QEP_UNIT_POS_EVENT_DIV);
            // Enable edge capture unit
            EQEP_enableCapture(qepBase);
            // Enable eQEP module
            EQEP_enableModule(qepBase);
            qepBase = (++p_qepCfg)->eqepBase;
        }
    }
    return;
}

/**
 * @brief       SPI bus initialization function.
 * @param[in]   *hal    Pointer on the general HAL structure
 */
static void HAL_SPI_init(HAL* hal)
{
    const SPI_TypeDef* p_spiCfg = hal->spiHandler;
    uint32_t           spiBase  = p_spiCfg->spiBase;

    if(p_spiCfg != NULL)
    {
        while(spiBase != 0xFFFFFFFF)
        {
            /* Disable SPI before configuration
             * Set SPI configuration
             * Disable loopback
             * Free run (debug mode)
             */
            SPI_disableModule(spiBase);
            SPI_disableLoopback(spiBase);
            SPI_setConfig(spiBase, DEVICE_LSPCLK_FREQ, p_spiCfg->protocol, p_spiCfg->mode, p_spiCfg->bitRate, p_spiCfg->dataWidth);
            SPI_setEmulationMode(spiBase, SPI_EMULATION_FREE_RUN);
            /* Configure FIFO. */
            (p_spiCfg->spiFifoEnable) ? (SPI_enableFIFO(spiBase)) : (SPI_disableFIFO(spiBase));
            SPI_resetTxFIFO(spiBase);
            SPI_resetRxFIFO(spiBase);
            SPI_setFIFOInterruptLevel(spiBase, p_spiCfg->txLevel, p_spiCfg->rxLevel);
            /* Clear flags & disable interrupts */
            SPI_clearInterruptStatus(spiBase, SPI_INT_RXFF | SPI_INT_TXFF | SPI_INT_RXFF_OVERFLOW);
            SPI_disableInterrupt(spiBase, SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RX_OVERRUN | SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            /* Enable SPI module */
            SPI_enableModule(spiBase);
            spiBase = (++p_spiCfg)->spiBase;
        }
    }
    return;
}

/**
 * @brief       DMA initialization function.
 * @param[in]   *hal    Pointer on the general HAL structure
 */
static void HAL_DMA_init(HAL* hal)
{
    const DMA_TypeDef* p_dmaCfg = hal->dmaHandler;
    uint32_t           dmaBase  = p_dmaCfg->dmaBase;

    if(p_dmaCfg != NULL)
    {
        /* Global hardware reset of the DMA controller. */
        DMA_initController();
        DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

        while(dmaBase != 0xFFFFFFFF)
        {
            /* DMA channel initialization
             * Configure direction, words per burst
             */
            DMA_configAddresses(dmaBase, p_dmaCfg->destAddr, p_dmaCfg->srcAddr);
            DMA_configBurst(dmaBase, p_dmaCfg->burstSize, p_dmaCfg->srcStep, p_dmaCfg->destStep);
            DMA_configTransfer(dmaBase, p_dmaCfg->transferSize, p_dmaCfg->srcStep, p_dmaCfg->destStep);
            DMA_configWrap(dmaBase, p_dmaCfg->srcWrapSize, p_dmaCfg->srcStep, p_dmaCfg->destWrapSize, p_dmaCfg->destStep);
            DMA_configMode(dmaBase, p_dmaCfg->trigger, p_dmaCfg->config);
            /* Clear flags & disable interrupts */
            DMA_disableOverrunInterrupt(dmaBase);
            DMA_disableInterrupt(dmaBase);
            /* Enable Trigger source & DMA channel */
            DMA_clearTriggerFlag(dmaBase);
            DMA_enableTrigger(dmaBase);
            DMA_startChannel(dmaBase);
            dmaBase = (++p_dmaCfg)->dmaBase;
        }
    }
    return;
}

/**
 * @brief       Interrupts initialization function.
 */
void HAL_Interrupts_init(void)
{
    // Setup Interrupts
    HAL_setupInterrupts();
    // Enable Interrupts
    HAL_enableInterrupts();
}

void HAL_setupInterrupts(void)
{
    // Attach ISR to interrupt
    Interrupt_register(INT_ADCA1, &adc1ISR);
    Interrupt_register(INT_ADCA2, &adc2ISR);
    Interrupt_register(INT_ADCA3, &adc3ISR);
    Interrupt_register(INT_ADCA4, &adc4ISR);
}

void HAL_enableInterrupts(void)
{
    // Enable EPWM interrupts
    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_ADCA2);
    Interrupt_enable(INT_ADCA3);
    Interrupt_enable(INT_ADCA4);
    // Enable interrupt group 1 (ePWM 1 and 4 interrupts are here)
    Interrupt_enableInCPU(INTERRUPT_CPU_INT1);
    Interrupt_enableInCPU(INTERRUPT_CPU_INT10);
    // enable global interrupt
    EINT;
}

void ADC_offsetCalibration(MOTOR_ACQ* motor)
{
    MOTOR_ACQ* p_motor1 = &motor[MOTOR_1];
    MOTOR_ACQ* p_motor2 = &motor[MOTOR_2];
    uint16_t calibCnt;

    // force no switching with low side transistors closed
    EPWM_setCounterCompareValue(MOTOR1_PWM1_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);
    EPWM_setCounterCompareValue(MOTOR1_PWM2_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);
    EPWM_setCounterCompareValue(MOTOR1_PWM3_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);
    EPWM_setCounterCompareValue(MOTOR2_PWM1_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);
    EPWM_setCounterCompareValue(MOTOR2_PWM2_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);
    EPWM_setCounterCompareValue(MOTOR2_PWM3_BASE, EPWM_COUNTER_COMPARE_A, PWM_TBPRD);

    p_motor1->offsetCal = true;
    p_motor2->offsetCal = true;

    //    motorDebug->calibMode = true;

    for (calibCnt = 0; calibCnt < 10000; calibCnt++)
    {
        if (calibCnt > 1000)
        {
            // Wait data acquisition of Ia, Ib, Ic & Vbus of Motor1. Blocking.
            while(p_motor1->dataValid[0] == false);
            p_motor1->dataValid[0]  = false;
            p_motor1->offset.ia     = 0.998f * p_motor1->offset.ia   + 0.002f * p_motor1->raw.ia;
            p_motor1->offset.ib     = 0.998f * p_motor1->offset.ib   + 0.002f * p_motor1->raw.ib;
            p_motor1->offset.ic     = 0.998f * p_motor1->offset.ic   + 0.002f * p_motor1->raw.ic;
            p_motor1->offset.vbus   = 0.998f * p_motor1->offset.vbus + 0.002f * p_motor1->raw.vbus;

            // Wait data acquisition of Ia, Ib, Ic & Vbus of Motor2. Blocking.
            while(p_motor2->dataValid[0] == false);
            p_motor2->dataValid[0]  = false;
            p_motor2->offset.ia     = 0.998f * p_motor2->offset.ia   + 0.002f * p_motor2->raw.ia;
            p_motor2->offset.ib     = 0.998f * p_motor2->offset.ib   + 0.002f * p_motor2->raw.ib;
            p_motor2->offset.ic     = 0.998f * p_motor2->offset.ic   + 0.002f * p_motor2->raw.ic;
            p_motor2->offset.vbus   = 0.998f * p_motor2->offset.vbus + 0.002f * p_motor2->raw.vbus;

            // Wait data acquisition of Va, Vb, Vc & Vpot of Motor1. Blocking.
            while(p_motor1->dataValid[1] == false);
            p_motor1->dataValid[1]  = false;
            p_motor1->offset.va     = 0.998f * p_motor1->offset.va   + 0.002f * p_motor1->raw.va;
            p_motor1->offset.vb     = 0.998f * p_motor1->offset.vb   + 0.002f * p_motor1->raw.vb;
            p_motor1->offset.vc     = 0.998f * p_motor1->offset.vc   + 0.002f * p_motor1->raw.vc;

            // Wait data acquisition of Va, Vb, Vc & Vpot of Motor2. Blocking.
            while(p_motor2->dataValid[1] == false);
            p_motor2->dataValid[1]  = false;
            p_motor2->offset.va     = 0.998f * p_motor2->offset.va   + 0.002f * p_motor2->raw.va;
            p_motor2->offset.vb     = 0.998f * p_motor2->offset.vb   + 0.002f * p_motor2->raw.vb;
            p_motor2->offset.vc     = 0.998f * p_motor2->offset.vc   + 0.002f * p_motor2->raw.vc;
        }
        else if (calibCnt == 1000)
        {
            while(p_motor1->dataValid[0] == false);
            p_motor1->dataValid[0]  = false;
            p_motor1->offset.ia     = p_motor1->raw.ia;
            p_motor1->offset.ib     = p_motor1->raw.ib;
            p_motor1->offset.ic     = p_motor1->raw.ic;
            p_motor1->offset.vbus   = p_motor1->raw.vbus;

            while(p_motor2->dataValid[0] == false);
            p_motor2->dataValid[0]  = false;
            p_motor2->offset.ia     = p_motor2->raw.ia;
            p_motor2->offset.ib     = p_motor2->raw.ib;
            p_motor2->offset.ic     = p_motor2->raw.ic;
            p_motor2->offset.vbus   = p_motor2->raw.vbus;

            while(p_motor1->dataValid[1] == false);
            p_motor1->dataValid[1]  = false;
            p_motor1->offset.va     = p_motor1->raw.va;
            p_motor1->offset.vb     = p_motor1->raw.vb;
            p_motor1->offset.vc     = p_motor1->raw.vc;

            while(p_motor2->dataValid[1] == false);
            p_motor2->dataValid[1]  = false;
            p_motor2->offset.va     = p_motor2->raw.va;
            p_motor2->offset.vb     = p_motor2->raw.vb;
            p_motor2->offset.vc     = p_motor2->raw.vc;
        }
        else
        {
            while(p_motor1->dataValid[0] == false);
            p_motor1->dataValid[0] = false;

            while(p_motor2->dataValid[0] == false);
            p_motor2->dataValid[0] = false;

            while(p_motor1->dataValid[1] == false);
            p_motor1->dataValid[1] = false;

            while(p_motor2->dataValid[1] == false);
            p_motor2->dataValid[1] = false;
        }
    }

    p_motor1->offsetCal = false;
    p_motor2->offsetCal = false;
}
