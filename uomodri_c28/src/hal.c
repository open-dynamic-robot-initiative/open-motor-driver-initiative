/*
 * File name: hal.c
 * Description: Source file containing Hardware Abstraction Layer source code
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "device.h"
#include "hal.h"
#include "cla_shared.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void HAL_GPIO_ini(const gpio_cfg_t*);
void HAL_ADC_ini(const adc_cfg_t*);
void HAL_ePWM_ini(const epwm_cfg_t*);
void HAL_eQEP_ini(const eqep_cfg_t*);
void HAL_SPI_ini(const spi_cfg_t*);
void HAL_SCI_ini(const sci_cfg_t*);
void HAL_TMR_ini(const tmr_cfg_t*);
void HAL_DMA_rst(void);
void HAL_IPC_ini(const ipc_cfg_t*);
void HAL_CLA_ini(const cla_cfg_t*);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Initialize all peripherals necessaries.
 * @param[in]   *p_hal  Pointer on the general HAL structure definition
 */
void HAL_ini(const hal_cfg_t* p_hal)
{
    if(p_hal != NULL)
    {
        // Initialize GPIO pins
        HAL_GPIO_ini(p_hal->p_gpioHandle);
        // Initialize ADC
        HAL_ADC_ini(p_hal->p_adcHandle);
        // Initialize ePWM
        HAL_ePWM_ini(p_hal->p_epwmHandle);
        // Initialize eQEP
        HAL_eQEP_ini(p_hal->p_eqepHandle);
        // Initialize IPC communication
        HAL_IPC_ini(p_hal->p_ipcHandle);
        // Initialize CPU TIMERS
        HAL_TMR_ini(p_hal->p_timerHandle);
        // Initialize SCI/UART debug
        HAL_SCI_ini(p_hal->p_sciHandle);
        // Initialize DMA
        HAL_DMA_rst();
        HAL_DMA_ini(p_hal->p_dmaHandle);
        // Initialize SPI
        HAL_SPI_ini(p_hal->p_spiHandle);
        // Initialize CLA
        HAL_CLA_ini(p_hal->p_claHandle);
    }

    return;
}

/**
 * @brief       GPIO initialization function.
 * @param[in]   *p_gpio     Pointer on the GPIO structure definition
 */
void HAL_GPIO_ini(const gpio_cfg_t* p_gpio)
{
    if(p_gpio != NULL)
    {
        uint32_t pinNum = p_gpio->pinNumber;

        while(pinNum != UINT32_MAX)
        {
            GPIO_setMasterCore(pinNum, p_gpio->coreSelect);
            if(p_gpio->analogModeEnable == GPIO_ANALOG_DISABLED)
            {
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
            }
            else
                GPIO_setAnalogMode(pinNum, GPIO_ANALOG_ENABLED);
            pinNum = (++p_gpio)->pinNumber;
        }
    }

    return;
}

/**
 * @brief       ADC initialization function.
 * @param[in]   *p_adcCfg   Pointer on the ADC configuration structure
 */
void HAL_ADC_ini(const adc_cfg_t* p_adcCfg)
{
    if((p_adcCfg != NULL) && (p_adcCfg->p_adcIni != NULL) && (p_adcCfg->p_adcAcq != NULL))
    {
        uint32_t adcBase    = p_adcCfg->adcBase;
        while(adcBase != UINT32_MAX)
        {
            const adc_ini_t* p_adcIni   = p_adcCfg->p_adcIni;
            const adc_acq_t* p_adcAcq   = p_adcCfg->p_adcAcq;
            const adc_int_t* p_adcInt   = p_adcCfg->p_adcInt;
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

            while(p_adcIni->adcBase == adcBase)
            {
                /* Set priorities according round-robin
                 * Set the input channel, SOC number, trigger source & S/H window
                 * SOC source will be PWM, no IT.
                 */
                ADC_setSOCPriority(adcBase, ADC_SOC_PRIORITY);
                ADC_setupSOC(adcBase, p_adcIni->adcSOCNumber, p_adcIni->adcTriggerSrc, p_adcIni->adcChannel, ADC_SAMPLING_WINDOW);
                ADC_setInterruptSOCTrigger(adcBase, p_adcIni->adcSOCNumber, ADC_INT_SOC_TRIGGER);
                p_adcIni++;
            }

            while(p_adcAcq->adcBase == adcBase)
            {
                /* Post processing block initialization, associate with ADC_SOC_NUMBER0
                 * Initialize PPB offset to zero
                 */
                ADC_setupPPB(adcBase, p_adcAcq->adcPPBNumber, p_adcAcq->adcSOCNumber);
                p_adcAcq++;
            }

            if(p_adcCfg->p_adcInt != NULL)
            {
                while(p_adcInt->adcBase == adcBase)
                {
                    ADC_setInterruptSource(adcBase, p_adcInt->adcIntNumber, p_adcInt->adcEOCNumber);
                    ADC_enableContinuousMode(adcBase, p_adcInt->adcIntNumber);
                    ADC_enableInterrupt(adcBase, p_adcInt->adcIntNumber);
                    ADC_clearInterruptStatus(adcBase, p_adcInt->adcIntNumber);
                    p_adcInt++;
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
void HAL_ADC_offsetCalib(const hal_motor_cfg_t* p_halMotorCfg)
{
    if(p_halMotorCfg != NULL)
    {
        const epwm_cc_t*    p_pwmCC     = NULL;
        const adc_int_t*    p_ITConv    = NULL;
        lpf_t               fltCoef     = {.a = 0.998f, .one_minus_a = 0.002f};
        float32_t           i[3]        = {0.0f, 0.0f, 0.0f};
        uint16_t            calibCnt    = 0;
        uint8_t             loop        = 0;

        // force no switching with low side transistors closed
        for(loop = 0; loop < 3; loop++)
        {
            p_pwmCC = p_halMotorCfg->p_pwmCntCmp[loop];
            EPWM_setCounterCompareValue(p_pwmCC->epwmBase, p_pwmCC->compModule, p_pwmCC->compCount);
        }

        for(calibCnt = 0; calibCnt < 10000; calibCnt++)
        {
            /* Wait data acquisition for Ia, Ib, Ic . Blocking. */
            p_ITConv = p_halMotorCfg->p_intAcq;
            // Wait IT flag event
            while(ADC_getInterruptStatus(p_ITConv->adcBase, p_ITConv->adcIntNumber) == false);
            if(calibCnt > 1000)
                for(loop = 0; loop < 3; loop++)
                    UOMODRI_HAL_ADC_CALIB(fltCoef, i[loop], (float32_t) *(p_halMotorCfg->p_iAcq[loop]->adcResultReg));
            else
                for(loop = 0; loop < 3; loop++)
                    i[loop] = (float32_t) *(p_halMotorCfg->p_iAcq[loop]->adcResultReg);

            // clear flags
            for(loop = 0; loop < 3; loop++)
            {
                // Clear the interrupt overflow flag
                ADC_clearInterruptOverflowStatus(p_halMotorCfg->p_iAcq[loop]->adcBase, p_ITConv->adcIntNumber);
                // Clear the interrupt flag
                ADC_clearInterruptStatus(p_halMotorCfg->p_iAcq[loop]->adcBase, p_ITConv->adcIntNumber);
            }
        }

        // Set offsets in PPBs
        for(loop = 0; loop < 3; loop++)
            // Sets the post processing block reference offset for tensions
            ADC_setPPBReferenceOffset(p_halMotorCfg->p_iAcq[loop]->adcBase,
                                      p_halMotorCfg->p_iAcq[loop]->adcPPBNumber,
                                      (uint16_t)i[loop]);
    }

    return;
}

/**
 * @brief       ePWM global peripheral initialization function.
 * @param[in]   *p_pwmCfg   Pointer on the PWM configuration structure
 */
void HAL_ePWM_ini(const epwm_cfg_t* p_pwmCfg)
{
    if((p_pwmCfg != NULL) && (p_pwmCfg->p_epwmTimeBase != NULL) && (p_pwmCfg->p_epwmCounterCompare) &&
            (p_pwmCfg->p_epwmActionQualifier != NULL) && (p_pwmCfg->p_epwmDeadband != NULL))
    {
        uint32_t pwmBase    = p_pwmCfg->epwmBase;
        /* Disable sync(Freeze clock to PWM as well). GTBCLKSYNC is applicable
         * only for multiple core devices.
         */
        SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
        // Set correct frequency for EPWMCLK (max = PLLSYSCLK / 2)
        SysCtl_setEPWMClockDivider(SYSCTL_EPWMCLK_DIV_2);

        while(pwmBase != UINT32_MAX)
        {
            const epwm_tb_t* p_pwmTimeBase          = p_pwmCfg->p_epwmTimeBase;
            const epwm_cc_t* p_pwmCounterCompare    = p_pwmCfg->p_epwmCounterCompare;
            const epwm_aq_t* p_pwmActionQualifier   = p_pwmCfg->p_epwmActionQualifier;
            const epwm_db_t* p_pwmDeadBand          = p_pwmCfg->p_epwmDeadband;
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
//                EPWM_setOneShotSyncOutTrigger(pwmBase, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
//                EPWM_enableOneShotSync(pwmBase);
//                EPWM_startOneShotSync(pwmBase);
                EPWM_setSyncInPulseSource(pwmBase, p_pwmTimeBase->syncInputSrc);
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
            if(p_pwmCfg->p_epwmProgChopper != NULL)
            {
                const epwm_pc_t* p_pwmProgChopper = p_pwmCfg->p_epwmProgChopper;

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
            if(p_pwmCfg->p_epwmAdcSocEventTrigger != NULL)
            {
                const epwm_et_t* p_pwmAdcSocEvtTrigger = p_pwmCfg->p_epwmAdcSocEventTrigger;

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
            if(p_pwmCfg->p_epwmInterruptsEvent != NULL)
            {
                const epwm_it_t* p_pwmInterruptsEvent = p_pwmCfg->p_epwmInterruptsEvent;

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
                        EPWM_enableInterrupt(pwmBase); // TODO : perhaps move out of HAL
                        /*
                         *     EALLOW;
                         *     EPwm1Regs.ETCLR.bit.INT = 1;
                         *     EPwm1Regs.ETSEL.bit.INTEN = 1;
                         *     EDIS;
                         */
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
//        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
        EPWM_forceSyncPulse(MOTOR1_PWM1_BASE);
    }

    return;
}

/**
 * @brief       EQEP (Quadrature encoder reading) initialization function.
 * @param[in]   *p_qepCfg   Pointer on the QEP configuration structure
 */
void HAL_eQEP_ini(const eqep_cfg_t* p_qepCfg)
{
    if(p_qepCfg != NULL)
    {
        uint32_t qepBase = p_qepCfg->eqepBase;
        while(qepBase != UINT32_MAX)
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
            // Disable Compare, watchdog , QMA and IT units.
            EQEP_disableCompare(qepBase);
            EQEP_disableWatchdog(qepBase);
            EQEP_setQMAModuleMode(qepBase, EQEP_QMA_MODE_BYPASS);
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
void HAL_SPI_ini(const spi_cfg_t* p_spiCfg)
{
    if(p_spiCfg != NULL)
    {
        uint32_t spiBase = p_spiCfg->spiBase;
        while(spiBase != UINT32_MAX)
        {
            /* Disable SPI before configuration */
            SPI_disableModule(spiBase);
            /* Configure FIFO if required. */
            SPI_disableFIFO(spiBase);
            if((p_spiCfg->rxLevel != SPI_FIFO_RXEMPTY) || (p_spiCfg->rxLevel != SPI_FIFO_RX0) ||
                    (p_spiCfg->txLevel != SPI_FIFO_TXEMPTY) || (p_spiCfg->txLevel != SPI_FIFO_TX0))
            {
                SPI_enableFIFO(spiBase);
                SPI_resetTxFIFO(spiBase);
                SPI_resetRxFIFO(spiBase);
                SPI_setFIFOInterruptLevel(spiBase, p_spiCfg->txLevel, p_spiCfg->rxLevel);
            }
            /* Set SPI configuration, disable loopback, enable free run (debug mode) */
            SPI_setConfig(spiBase, DEVICE_LSPCLK_FREQ, p_spiCfg->protocol, p_spiCfg->mode, p_spiCfg->bitRate, p_spiCfg->dataWidth);
            SPI_setEmulationMode(spiBase, SPI_EMULATION_FREE_RUN);
            SPI_disableLoopback(spiBase);
            SPI_disableHighSpeedMode(spiBase);
            SPI_disableTriWire(spiBase);
            if(p_spiCfg->mode == SPI_MODE_SLAVE)
                SPI_setSTESignalPolarity(spiBase, SPI_STE_ACTIVE_LOW);
            /* Clear flags & enable or disable interrupts */
            SPI_clearInterruptStatus(spiBase, SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RX_OVERRUN |
                                     SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            SPI_disableInterrupt(spiBase, SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RX_OVERRUN |
                                 SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            if(p_spiCfg->intEnable)
                SPI_enableInterrupt(spiBase, p_spiCfg->intSrc);
            /* Enable SPI module */
            SPI_enableModule(spiBase);
            spiBase = (++p_spiCfg)->spiBase;
        }
    }

    return;
}

/**
 * @brief       UART bus initialization function.
 * @param[in]   *p_uartCfg   Pointer on the UART configuration structure
 */
void HAL_SCI_ini(const sci_cfg_t* p_sciCfg)
{
    if(p_sciCfg != NULL)
    {
        uint32_t sciBase = p_sciCfg->sciBase;
        while(sciBase != UINT32_MAX)
        {
            /* Set SPI configuration, disable loopback */
            SCI_setConfig(sciBase, DEVICE_LSPCLK_FREQ, p_sciCfg->bitRate, p_sciCfg->sciMode);
            /* Stop the SCI during configuration */
//            SCI_disableModule(sciBase);
            SCI_disableLoopback(sciBase);
            /* Multiprocessor mode */
            if(p_sciCfg->multiProcMode == SCI_MULTI_PROC_ADDR)
                SCI_setAddrMultiProcessorMode(sciBase);
            else if(p_sciCfg->multiProcMode == SCI_MULTI_PROC_IDLE)
                SCI_setIdleMultiProcessorMode(sciBase);
            /* Configure FIFO if required. */
            SCI_disableFIFO(sciBase);
            if((p_sciCfg->rxLevel != SCI_FIFO_RX0) || (p_sciCfg->txLevel != SCI_FIFO_TX0))
            {
                SCI_enableFIFO(sciBase);
                SCI_resetTxFIFO(sciBase);
                SCI_resetRxFIFO(sciBase);
                SCI_setFIFOInterruptLevel(sciBase, p_sciCfg->txLevel, p_sciCfg->rxLevel);
            }
            /* Clear flags & enable or disable interrupts */
            SCI_clearInterruptStatus(sciBase, SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_TXFF | SCI_INT_RXFF);
            SCI_disableInterrupt(sciBase, SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY | SCI_INT_TXFF | SCI_INT_RXFF);
            if(p_sciCfg->intEnable)
                SCI_enableInterrupt(sciBase, p_sciCfg->intSrc);
            /* Restart SCI after configuration */
            SCI_enableModule(sciBase);
            sciBase = (++p_sciCfg)->sciBase;
        }
    }

    return;
}

/**
 * @brief       Force a hard reset of all DMA peripherals
 */
void HAL_DMA_rst(void)
{
    /* Global hardware reset of the DMA controller. */
    DMA_initController();
    DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

    return;
}

/**
 * @brief       DMA initialization function.
 * @param[in]   *p_dmaCfg   Pointer on the DMA configuration structure
 */
void HAL_DMA_ini(const dma_cfg_t* p_dmaCfg)
{
    if(p_dmaCfg != NULL)
    {
        uint32_t dmaChBase = p_dmaCfg->dmaChBase;
        while(dmaChBase != UINT32_MAX)
        {
            /* DMA channel initialization
             * Configure direction, words per burst
             */
            DMA_configAddresses(dmaChBase, p_dmaCfg->p_dstAddr, p_dmaCfg->p_srcAddr);
            DMA_configBurst(dmaChBase, p_dmaCfg->burstSize, p_dmaCfg->srcStep, p_dmaCfg->dstStep);
            DMA_configTransfer(dmaChBase, p_dmaCfg->transferSize, p_dmaCfg->srcStep, p_dmaCfg->dstStep);
            DMA_configWrap(dmaChBase, p_dmaCfg->srcWrapSize, p_dmaCfg->srcWrapStep, p_dmaCfg->dstWrapSize, p_dmaCfg->dstWrapStep);
            DMA_configMode(dmaChBase, p_dmaCfg->trigger, p_dmaCfg->config);
            /* Clear flags & set or disable interrupts */
            DMA_disableOverrunInterrupt(dmaChBase);
            DMA_setInterruptMode(dmaChBase, p_dmaCfg->intMode);
            (p_dmaCfg->intEnable) ? (DMA_enableInterrupt(dmaChBase)) : (DMA_disableInterrupt(dmaChBase));
            /* Enable Trigger source & start DMA channel */
            DMA_clearTriggerFlag(dmaChBase);
            DMA_triggerSoftReset(dmaChBase);
            asm(" nop");
            DMA_enableTrigger(dmaChBase);
            DMA_startChannel(dmaChBase);
            dmaChBase = (++p_dmaCfg)->dmaChBase;
        }
    }

    return;
}

/**
 * @brief       IPC initialization function.
 * @param[in]   *p_ipcCfg   Pointer on the IPC configuration structure
 */
void HAL_IPC_ini(const ipc_cfg_t* p_ipcCfg)
{
    if(p_ipcCfg != NULL)
    {
        IPC_Type_t  ipcType     = p_ipcCfg->ipcType;
        /* (0) - C_CPU1_L_CPU2_R, (1) - IPC_CPU1_L_CM_R, (2) - IPC_CPU2_L_CPU1_R, (3) - IPC_CPU2_L_CM_R*/
        uint8_t     ipcSyncDone = 0;
        while(ipcType != IPC_TOTAL_NUM)
        {
            // Enable IPC interrupts
            if(p_ipcCfg->ipcIntEnable == true)
                IPC_registerInterrupt(ipcType, p_ipcCfg->ipcInt, p_ipcCfg->p_ipcIntHandler);
            // Initialize message queue
            if(p_ipcCfg->ipcMsgQEnable == true)
                IPC_initMessageQueue(ipcType, p_ipcCfg->p_ipcMsgQ, p_ipcCfg->ipcIntMsgQL, p_ipcCfg->ipcIntMsgQR);
            // Synchronize both the cores.
            if(!(ipcSyncDone & (1 << ipcType)))
            {
                // Clear any IPC flags if set already
                IPC_clearFlagLtoR(ipcType, IPC_FLAG_ALL);
                // Synchronize both the cores.
                IPC_sync(ipcType, IPC_SYNC_FLAG);
                ipcSyncDone |= (1 << ipcType);
            }
            ipcType = (++p_ipcCfg)->ipcType;
        }
    }

    return;
}

/**
 * @brief       CLA initialization function.
 * @param[in]   *p_claCfg   Pointer on the CLA configuration structure
 */
void HAL_CLA_ini(const cla_cfg_t* p_clacCfg)
{
    if(p_clacCfg != NULL)
    {
        EALLOW;

#ifdef _FLASH
        extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
        extern uint32_t Cla1ConstRunStart, Cla1ConstLoadStart, Cla1ConstLoadSize;
        // Copy over code from FLASH to RAM
        memcpy((uint32_t *)&Cla1funcsRunStart,  (uint32_t *)&Cla1funcsLoadStart,    (uint32_t)&Cla1funcsLoadSize);
        memcpy((uint32_t *)&Cla1ConstRunStart,  (uint32_t *)&Cla1ConstLoadStart,    (uint32_t)&Cla1ConstLoadSize);
#endif //_FLASH

        // Initialize and wait for CLA1ToCPUMsgRAM
        MemCfg_initSections(MEMCFG_SECT_MSGCLA1TOCPU);
        while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCLA1TOCPU));
        // Initialize and wait for CPUToCLA1MsgRAM
        MemCfg_initSections(MEMCFG_SECT_MSGCPUTOCLA1);
        while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCPUTOCLA1));

        uint8_t i;

        for(i = 0; i < 8; i++)
        {
            uint32_t memcfg_sect_lsx = MEMCFG_SECT_TYPE_LS | (1 << i);
            /* Select LSxRAM to be the programming space for the CLA
             * First configure the CLA to be the master for LSx and then
             * set the space to be a program block */
            if((p_clacCfg->progRAM & memcfg_sect_lsx) == memcfg_sect_lsx)
            {
                MemCfg_setLSRAMMasterSel(memcfg_sect_lsx, MEMCFG_LSRAMMASTER_CPU_CLA1);
                MemCfg_setCLAMemType(memcfg_sect_lsx, MEMCFG_CLA_MEM_PROGRAM);
            }
            /* Configure LSxRAM as data spaces for the CLA
             * First configure the CLA to be the master for LSx and then
             * set the spaces to be code blocks */
            if((p_clacCfg->dataRAM & memcfg_sect_lsx) == memcfg_sect_lsx)
            {
                MemCfg_setLSRAMMasterSel(memcfg_sect_lsx, MEMCFG_LSRAMMASTER_CPU_CLA1);
                MemCfg_setCLAMemType(memcfg_sect_lsx, MEMCFG_CLA_MEM_DATA);
            }
        }

    /* Suppressing #770-D conversion from pointer to smaller integer
     * The CLA address range is 16 bits so the addresses passed to the MVECT
     * registers will be in the lower 64KW address space. Turn the warning
     * back on after the MVECTs are assigned addresses */
        EDIS;
        EALLOW;

#pragma diag_suppress=770
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_1, (uint16_t)&(p_clacCfg->fun_ptr));
#pragma diag_warning=770
    // Enable Tasks 1 and 8
    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, (1 << p_clacCfg->claTaskNum));
    CLA_forceTasks(CLA1_BASE, (1 << p_clacCfg->claTaskNum));
    }

    return;
}

/**
 * @brief       Interrupts initialization function.
 */
void HAL_INT_ini(const int_cfg_t* p_intCfg)
{
    if(p_intCfg != NULL)
    {
        uint32_t intNum = p_intCfg->intNum;
        // Clear & set IT flags & enable IT bits in a known state.
        Interrupt_initModule();
        // Setup Interrupts
        while(intNum != UINT32_MAX)
        {
            // Attach ISR to interrupt
            Interrupt_register(intNum, p_intCfg->p_intHandler);
            // Enable Interrupts
            Interrupt_enable(intNum);
            intNum = (++p_intCfg)->intNum;
        }
        EINT;   // enable global interrupt (INTM)
        ERTM;   // Enable realtime interrupt (DGBM)
    }

    return;
}

/**
 * @brief       CPU Timer initialization function.
 * @param[in]   *p_TimerCfg Pointer on the CPU timer configuration structure
 */
void HAL_TMR_ini(const tmr_cfg_t* p_TimerCfg)
{
    if(p_TimerCfg != NULL)
    {
        uint32_t cpuTimerBase = p_TimerCfg->cpuTimerBase;
        while(cpuTimerBase != UINT32_MAX)
        {
            /* Make sure timer is stopped */
            CPUTimer_stopTimer(cpuTimerBase);
            /* Debug mode of the CPU Timers. */
            CPUTimer_setEmulationMode(cpuTimerBase, CPUTIMER_EMULATIONMODE_RUNFREE);
            /* Initialize pre-scale & period counters. Reload counters. */
            CPUTimer_setPreScaler(cpuTimerBase, p_TimerCfg->prescaler);
            CPUTimer_setPeriod(cpuTimerBase, p_TimerCfg->periodCount);
            /* clear flags & set or clear associated interrupts */
            CPUTimer_clearOverflowFlag(cpuTimerBase);
            (p_TimerCfg->intEnable) ? (CPUTimer_enableInterrupt(cpuTimerBase)) : (CPUTimer_disableInterrupt(cpuTimerBase));
            /* Reload & start CPU-Timer */
            CPUTimer_reloadTimerCounter(cpuTimerBase);
            CPUTimer_startTimer(cpuTimerBase);
            cpuTimerBase = (++p_TimerCfg)->cpuTimerBase;
        }
    }
}
