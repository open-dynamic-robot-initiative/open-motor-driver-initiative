/*
 * File name: hal.c
 * Description: Source file containing Hardware Abstraction Layer source code
 */

#include "hal.h"
#include "uomodri_hal_config_handlers.h"

/* Static function signatures */
// Device HAL
void HAL_GPIO_init(const GPIO_TypeDef*);
void HAL_ADC_init(const ADC_TypeDef*);
void HAL_ePWM_init(const ePWM_TypeDef*);
void HAL_eQEP_init(const eQEP_TypeDef*);
void HAL_SPI_init(const SPI_TypeDef*);
void HAL_DMA_init(const DMA_TypeDef*);
//static void HAL_CLA_init(HAL*);

// Function definitions
/**
 * @brief       Initialize all peripherals necessaries.
 * @param[in]   *p_hal  Pointer on the general HAL structure definition
 */
void HAL_init(const HAL* p_hal)
{
    if(p_hal != NULL)
    {
        // Initialize GPIO pins
        HAL_GPIO_init(p_hal->gpioHandler);
        // Initialize CLA
        //    HAL_CLA_init(p_hal);
        // Initialize ADC
        HAL_ADC_init(p_hal->adcHandler);
        // Initialize ePWM
        HAL_ePWM_init(p_hal->epwmHandler);
        // Initialize eQEP
        HAL_eQEP_init(p_hal->eqepHandler);
        // Initialize DMA
        HAL_DMA_init(p_hal->dmaHandler);
        // Initialize SPIA device
        HAL_SPI_init(p_hal->spiHandler);
    }

    return;
}

/**
 * @brief       GPIO initialization function.
 * @param[in]   *p_gpio     Pointer on the GPIO structure definition
 */
void HAL_GPIO_init(const GPIO_TypeDef* p_gpio)
{
    if(p_gpio != NULL)
    {
        uint32_t pinNum = p_gpio->pinNumber;

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
            if(p_gpio->intEnable)
            {
                GPIO_setInterruptPin(pinNum, p_gpio->extIntNum);
                GPIO_setInterruptType(p_gpio->extIntNum, p_gpio->intType);
                GPIO_enableInterrupt(p_gpio->extIntNum);
            }
            pinNum = (++p_gpio)->pinNumber;
        }
    }

    return;
}

/**
 * @brief       ADC initialization function.
 * @param[in]   *p_adcCfg   Pointer on the ADC configuration structure
 */
void HAL_ADC_init(const ADC_TypeDef* p_adcCfg)
{
    if((p_adcCfg != NULL) && (p_adcCfg->adcInit != NULL) && (p_adcCfg->adcAcq != NULL))
    {
        const ADC_INIT_TypeDef* p_adcInit   = p_adcCfg->adcInit;
        const ADC_ACQ_TypeDef*  p_adcAcq    = p_adcCfg->adcAcq;
        const ADC_IT_TypeDef*   p_adcIT     = p_adcCfg->adcIt;
        uint32_t                adcBase     = p_adcCfg->adcBase;

        while(adcBase != 0xFFFFFFFF)
        {
            /* Sets ADC clock to 50MHz (ADCCLK (derived from PERx.SYSCLK))
             * Set converting input mode : "ADC_MODE_SINGLE_ENDED" or "ADC_MODE_DIFFERENTIAL"
             * Set converting resolution : "ADC_RESOLUTION_12BIT" or "ADC_RESOLUTION_16BIT"
             * Set IT raising  : "ADC_PULSE_END_OF_ACQ_WIN" or "ADC_PULSE_END_OF_CONV"
             * Enables ADC converter
             */
            ADC_setPrescaler(adcBase, PERxSYSCLK_TO_ADCCLK_DIV);
            ADC_setMode(adcBase, ADC_RESOLUTION_BIT, ADC_SIGNAL_MODE);
            ADC_setInterruptPulseMode(adcBase, ADC_PULSE_END_MODE);
            ADC_enableConverter(adcBase);

            // Let some time to power up ADCs
            DEVICE_DELAY_US(1500U);

            while(p_adcInit->adcBase == adcBase)
            {
                /* Set priorities according round-robin
                 * Set the input channel, SOC number, trigger source & S/H window
                 * SOC source will be PWM, no IT.
                 */
                ADC_setSOCPriority(adcBase, ADC_SOC_PRIORITY);
                ADC_setupSOC(adcBase, p_adcInit->adcSOCNumber, p_adcInit->adcTriggerSrc, p_adcInit->adcChannel, ADC_SAMPLING_WINDOW);
                ADC_setInterruptSOCTrigger(adcBase, p_adcInit->adcSOCNumber, ADC_INT_SOC_TRIGGER);
                p_adcInit++;
            }

            while(p_adcAcq->adcBase == adcBase)
            {
                /* Post processing block initialization, associate with ADC_SOC_NUMBER0
                 * Initialize PPB offset to zero
                 */
                ADC_setupPPB(adcBase, p_adcAcq->adcPPBNumber, p_adcAcq->adcSOCNumber);
                p_adcAcq++;
            }

            if(p_adcCfg->adcIt != NULL)
            {
                while(p_adcIT->adcBase == adcBase)
                {
                    ADC_setInterruptSource(adcBase, p_adcIT->adcIntNumber, p_adcIT->adcEOCNumber);
                    ADC_enableContinuousMode(adcBase, p_adcIT->adcIntNumber);
                    ADC_enableInterrupt(adcBase, p_adcIT->adcIntNumber);
                    ADC_clearInterruptStatus(adcBase, p_adcIT->adcIntNumber);
                    p_adcIT++;
                }
            }
            adcBase = (++p_adcCfg)->adcBase;
        }
    }

    return;
}

/**
 * @brief       ADC calibration function for the PPB (Post Processing Block)
 * @param[in]   *p_motor    Pointer on the motor configuration structure
 */
void HAL_ADC_offsetCalib(const HAL_MOTOR_CFG* p_halMotorCfg)
{
    if(p_halMotorCfg != NULL)
    {
        const ePWM_CC_TypeDef*  p_pwmCC     = NULL;
        const ADC_IT_TypeDef*   p_ITConv    = NULL;
        float32_t               i[3]        = {0.0f, 0.0f, 0.0f};
        float32_t               v[3]        = {0.0f, 0.0f, 0.0f};
        uint16_t                calibCnt    = 0;
        uint8_t                 loop        = 0;

        // force no switching with low side transistors closed
        for(loop = 0; loop < 3; loop++)
        {
            p_pwmCC = p_halMotorCfg->p_pwmCntCmp[loop];
            EPWM_setCounterCompareValue(p_pwmCC->epwmBase, p_pwmCC->compModule, p_pwmCC->compCount);
        }

        for (calibCnt = 0; calibCnt < 10000; calibCnt++)
        {
            /* Wait data acquisition for Ia, Ib, Ic . Blocking.
             Va, Vb, Vc & Vbus will be also available. */
            p_ITConv = p_halMotorCfg->p_ITConv;
            // Wait IT flag event
            while(ADC_getInterruptStatus(p_ITConv->adcBase, p_ITConv->adcIntNumber) == false);
            if (calibCnt > 1000)
                for(loop = 0; loop < 3; loop++)
                {
                    i[loop] = 0.998f * i[loop] + 0.002f * (*p_halMotorCfg->p_iConv[loop]->adcResultReg);
                    v[loop] = 0.998f * v[loop] + 0.002f * (*p_halMotorCfg->p_vConv[loop]->adcResultReg);
                }
            else
                for(loop = 0; loop < 3; loop++)
                {
                    i[loop] = *(p_halMotorCfg->p_iConv[loop]->adcResultReg);
                    v[loop] = *(p_halMotorCfg->p_vConv[loop]->adcResultReg);
                }

            // clear flags
            for(loop = 0; loop < 3; loop++)
            {
                // Clear the interrupt overflow flag
                ADC_clearInterruptOverflowStatus(p_halMotorCfg->p_iConv[loop]->adcBase, p_ITConv->adcIntNumber);
                // Clear the interrupt flag
                ADC_clearInterruptStatus(p_halMotorCfg->p_iConv[loop]->adcBase, p_ITConv->adcIntNumber);
            }
        }

        // Set offsets in PPBs
        for(loop = 0; loop < 3; loop++)
        {
            // Sets the post processing block reference offset for tensions
            ADC_setPPBReferenceOffset(p_halMotorCfg->p_vConv[loop]->adcBase, p_halMotorCfg->p_vConv[loop]->adcPPBNumber, (uint16_t)v[loop]);
            // Sets the post processing block reference offset for tensions
            ADC_setPPBReferenceOffset(p_halMotorCfg->p_iConv[loop]->adcBase, p_halMotorCfg->p_iConv[loop]->adcPPBNumber, (uint16_t)i[loop]);
            // Enables two's complement capability
            ADC_enablePPBTwosComplement(p_halMotorCfg->p_iConv[loop]->adcBase, p_halMotorCfg->p_iConv[loop]->adcPPBNumber);
        }
    }

    return;
}

/**
 * @brief       ePWM global peripheral initialization function.
 * @param[in]   *p_pwmCfg   Pointer on the PWM configuration structure
 */
void HAL_ePWM_init(const ePWM_TypeDef* p_pwmCfg)
{
    if((p_pwmCfg != NULL) && ( p_pwmCfg->epwmTimeBase != NULL) && (p_pwmCfg->epwmCaptureCompare) &&
            (p_pwmCfg->epwmActionQualifier != NULL) && (p_pwmCfg->epwmDeadband != NULL))
    {
        const ePWM_TB_TypeDef* p_pwmTimeBase        = p_pwmCfg->epwmTimeBase;
        const ePWM_CC_TypeDef* p_pwmCounterCompare  = p_pwmCfg->epwmCaptureCompare;
        const ePWM_AQ_TypeDef* p_pwmActionQualifier = p_pwmCfg->epwmActionQualifier;
        const ePWM_DB_TypeDef* p_pwmDeadBand        = p_pwmCfg->epwmDeadband;
        uint32_t               pwmBase              = p_pwmCfg->epwmBase;

        // Disable global PWM clock during initialization
        SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
        // Set correct frequency for EPWMCLK (max = PLLSYSCLK / 2)
        SysCtl_setEPWMClockDivider(SYSCTL_EPWMCLK_DIV_2);

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
                EPWM_setTimeBasePeriod(pwmBase, PWM_TIMEBASE_CNT);
                (pwmBase == MOTOR1_PWM1_BASE) ? (EPWM_disablePhaseShiftLoad(pwmBase)) : (EPWM_enablePhaseShiftLoad(pwmBase));
                EPWM_setPhaseShift(pwmBase, p_pwmTimeBase->phaseCount);
                EPWM_setTimeBaseCounterMode(pwmBase, PWM_COUNTER_MODE);
                EPWM_setTimeBaseCounter(pwmBase, PWM_INITIAL_CNT_VAL);
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
            if(p_pwmCfg->epwmProgChopper != NULL)
            {
                const ePWM_PC_TypeDef* p_pwmProgChopper = p_pwmCfg->epwmProgChopper;

                while(p_pwmProgChopper->epwmBase == pwmBase)
                {
                    EPWM_enableChopper(pwmBase);
                    EPWM_setChopperDutyCycle(pwmBase, p_pwmProgChopper->dutyCycleCount);
                    EPWM_setChopperFreq(pwmBase, p_pwmProgChopper->freqDiv);
                    EPWM_setChopperFirstPulseWidth(pwmBase, p_pwmProgChopper->firstPulseWidth);
                    p_pwmProgChopper++;
                }
            }

            /* ADC SOC Event-Trigger (ET) Submodule Registers
             * Disable ADC trigger & IT generation while initialization is in progress.
             */
            if(p_pwmCfg->epwmAdcSocEventTrigger != NULL)
            {
                const ePWM_ET_TypeDef* p_pwmAdcSocEvtTrigger = p_pwmCfg->epwmAdcSocEventTrigger;

                while(p_pwmAdcSocEvtTrigger->epwmBase == pwmBase)
                {
                    // Disable ADC SOC events generation while init is in progress.
                    EPWM_disableADCTriggerEventCountInit(pwmBase, p_pwmAdcSocEvtTrigger->adcSOCType);
                    EPWM_disableADCTrigger(pwmBase, p_pwmAdcSocEvtTrigger->adcSOCType);
                    // If adcSOCEvtCount == 0 -> no ADC SOC events are generated
                    if(p_pwmAdcSocEvtTrigger->adcSOCEvtCount)
                    {
                        EPWM_setADCTriggerSource(pwmBase, p_pwmAdcSocEvtTrigger->adcSOCType, p_pwmAdcSocEvtTrigger->adcSOCSource);
                        EPWM_setADCTriggerEventPrescale(pwmBase, p_pwmAdcSocEvtTrigger->adcSOCType, p_pwmAdcSocEvtTrigger->adcSOCEvtCount);
                        EPWM_clearADCTriggerFlag(pwmBase, p_pwmAdcSocEvtTrigger->adcSOCType);
                        EPWM_enableADCTrigger(pwmBase, p_pwmAdcSocEvtTrigger->adcSOCType);
                    }
                    p_pwmAdcSocEvtTrigger++;
                }
            }

            /* Interrupt event (IT) Submodule Registers
             * Disable IT generation while initialization is in progress.
             */
            if(p_pwmCfg->epwmInterruptsEvent != NULL)
            {
                const ePWM_IT_TypeDef* p_pwmInterruptsEvent = p_pwmCfg->epwmInterruptsEvent;

                while(p_pwmInterruptsEvent->epwmBase == pwmBase)
                {
                    // Disable IT generation while init is in progress.
                    EPWM_disableInterruptEventCountInit(pwmBase);
                    EPWM_disableInterrupt(pwmBase);
                    // If IntEvtCount == 0 -> no Interrupts are generated
                    if(p_pwmInterruptsEvent->intEvtCount)
                    {
                        EPWM_setInterruptSource(pwmBase, p_pwmInterruptsEvent->intSource);
                        EPWM_setInterruptEventCount(pwmBase, p_pwmInterruptsEvent->intEvtCount);
                        EPWM_clearEventTriggerInterruptFlag(pwmBase);
                        EPWM_enableInterrupt(pwmBase);
                    }
                    p_pwmInterruptsEvent++;
                }
            }
            if(p_pwmCfg->epwmAdcSocSrcExtern)
                SysCtl_enableExtADCSOCSource(p_pwmCfg->epwmAdcSocSrcExtern);
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
 * @param[in]   *p_qepCfg   Pointer on the QEP configuration structure
 */
void HAL_eQEP_init(const eQEP_TypeDef* p_qepCfg)
{
    if(p_qepCfg != NULL)
    {
        uint32_t qepBase = p_qepCfg->eqepBase;

        while(qepBase != 0xFFFFFFFF)
        {
            // Configure the decoder for up-count mode, counting both on rising & falling edges (quadrature encoders)
            EQEP_setDecoderConfig(qepBase, p_qepCfg->eqepConfig);
            // Position & watchdog counter, unit & capture timer are all unaffected by an emulation suspend.
            EQEP_setEmulationMode(qepBase, QEP_EMUL_MODE);
            // Position counter latched on a unit time out and on rising edge of index pulse.
            EQEP_setLatchMode(qepBase, QEP_LATCH_MODE);
            // Position counter strobe source define : external GPIO, ADCSOCA or ADCSOCB.
            EQEP_setStrobeSource(qepBase, p_qepCfg->eqepStrobeSrc);
            // Position counter is reset when reaching max position (1 turn).
            EQEP_setPositionCounterConfig(qepBase, QEP_RESET_POSITION_MODE, p_qepCfg->eqepMaxResolution);
            // Set unit timer for speed measurement and enable it.
            EQEP_enableUnitTimer(qepBase, p_qepCfg->eqepUnitTimerPeriod);
            // Disable Compare, watchdog and IT units.
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
 * @param[in]   *p_spiCfg   Pointer on the SPI configuration structure
 */
void HAL_SPI_init(const SPI_TypeDef* p_spiCfg)
{
    if(p_spiCfg != NULL)
    {
        uint32_t spiBase = p_spiCfg->spiBase;

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
            /* Clear flags & disable interrupts */
            SPI_clearInterruptStatus(spiBase, SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RX_OVERRUN | SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            SPI_disableInterrupt(spiBase, SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RX_OVERRUN | SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            /* Configure FIFO if required. */
            SPI_disableFIFO(spiBase);
            if((p_spiCfg->rxLevel != SPI_FIFO_RXEMPTY) || (p_spiCfg->txLevel != SPI_FIFO_TXEMPTY))
            {
                SPI_enableFIFO(spiBase);
                SPI_resetTxFIFO(spiBase);
                SPI_resetRxFIFO(spiBase);
                SPI_setFIFOInterruptLevel(spiBase, p_spiCfg->txLevel, p_spiCfg->rxLevel);
                SPI_enableInterrupt(spiBase, SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            }
            /* Enable SPI module */
            SPI_enableModule(spiBase);
            spiBase = (++p_spiCfg)->spiBase;
        }
    }

    return;
}

/**
 * @brief       DMA initialization function.
 * @param[in]   *p_dmaCfg   Pointer on the DMA configuration structure
 */
void HAL_DMA_init(const DMA_TypeDef* p_dmaCfg)
{
    if(p_dmaCfg != NULL)
    {
        uint32_t dmaChBase = p_dmaCfg->dmaChBase;

        /* Global hardware reset of the DMA controller. */
        DMA_initController();
        DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

        while(dmaChBase != 0xFFFFFFFF)
        {
            /* DMA channel initialization
             * Configure direction, words per burst
             */
            DMA_configAddresses(dmaChBase, p_dmaCfg->destAddr, p_dmaCfg->srcAddr);
            DMA_configBurst(dmaChBase, p_dmaCfg->burstSize, p_dmaCfg->srcStep, p_dmaCfg->destStep);
            DMA_configTransfer(dmaChBase, p_dmaCfg->transferSize, p_dmaCfg->srcStep, p_dmaCfg->destStep);
            DMA_configWrap(dmaChBase, p_dmaCfg->srcWrapSize, p_dmaCfg->srcStep, p_dmaCfg->destWrapSize, p_dmaCfg->destStep);
            DMA_configMode(dmaChBase, p_dmaCfg->trigger, p_dmaCfg->config);
            /* Clear flags & disable interrupts */
            DMA_disableOverrunInterrupt(dmaChBase);
            DMA_disableInterrupt(dmaChBase);
            /* Enable Trigger source & DMA channel */
            DMA_clearTriggerFlag(dmaChBase);
            DMA_enableTrigger(dmaChBase);
            dmaChBase = (++p_dmaCfg)->dmaChBase;
        }
    }

    return;
}

/**
 * @brief       Interrupts initialization function.
 */
void HAL_IT_init(const IT_TypeDef* p_ITCfg)
{
    if(p_ITCfg != NULL)
    {
        uint32_t intNum = p_ITCfg->intNum;
        // Clear & set IT flags & enable IT bits in a known state.
        Interrupt_initModule();
        // Setup Interrupts
        while(intNum != 0xFFFFFFFF)
        {
            // Attach ISR to interrupt
            Interrupt_register(intNum, p_ITCfg->handler);
            // Enable Interrupts
            Interrupt_enable(intNum);
            intNum = (++p_ITCfg)->intNum;
        }
        // enable global interrupt
        EINT;
    }

    return;
}
