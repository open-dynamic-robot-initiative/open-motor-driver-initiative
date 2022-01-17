/*
 * File name: hal.c
 * Description: Source file containing Hardware Abstraction Layer source code
 */

#include "hal.h"
#include "communication.h"
#include "uomodri_hal_config_handlers.h"

//extern rx_uomodri_t* com_rx_cmd;
//extern tx_msg_t* com_tx_msg;

/* Static function signatures */
// Device HAL
void HAL_GPIO_init(const GPIO_TypeDef*);
void HAL_ADC_init(const ADC_TypeDef*);
void HAL_ePWM_init(const ePWM_TypeDef*);
void HAL_eQEP_init(const eQEP_TypeDef*);
void HAL_SPI_init(const SPI_TypeDef*);
void HAL_DMA_init(const DMA_TypeDef*);
void HAL_IPC_init(const IPC_TypeDef*);
void* HAL_IPC_addr_init(const IPC_TypeDef*, uint32_t);
#if (CLA == 1)
void HAL_CLA_init(const CLA_TypeDef*);
#endif

//const void* p_msgComInter;

// Function definitions
/**
 * @brief       Initialize all peripherals necessaries.
 * @param[in]   *p_hal  Pointer on the general HAL structure definition
 */
void HAL_init(HAL* p_hal)
{
//    uint32_t    command, data, addr;
    if(p_hal != NULL)
    {
        // Initialize GPIO pins
        HAL_GPIO_init(p_hal->p_gpioHandler);
        // Initialize ADC
        HAL_ADC_init(p_hal->p_adcHandler);
        // Initialize ePWM
        HAL_ePWM_init(p_hal->p_epwmHandler);
        // Initialize eQEP
        HAL_eQEP_init(p_hal->p_eqepHandler);
        // Initialize IPC communication
//        HAL_IPC_init(p_hal->p_ipcHandler);

//        // Memory address sharing CM to CPU1
//        while(p_hal->p_ipcHandler->p_ipcIntHandler != NULL)
//            p_hal->p_ipcHandler++;
//        com_rx_cmd = (void *)HAL_IPC_addr_init(p_hal->p_ipcHandler, 0x1000);
//        com_tx_msg = (void *)HAL_IPC_addr_init(p_hal->p_ipcHandler, 0x1001);
        // Initialize DMA
        HAL_DMA_init(p_hal->p_dmaHandler);
        // Initialize SPIA device
        HAL_SPI_init(p_hal->p_spiHandler);
#if (CLA == 1)
        // Initialize CLA
        HAL_CLA_init(p_hal->p_claHandler);
#endif
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
void HAL_ADC_init(const ADC_TypeDef* p_adcCfg)
{
    if((p_adcCfg != NULL) && (p_adcCfg->p_adcInit != NULL) && (p_adcCfg->p_adcAcq != NULL))
    {
        const ADC_INIT_TypeDef* p_adcInit   = p_adcCfg->p_adcInit;
        const ADC_ACQ_TypeDef*  p_adcAcq    = p_adcCfg->p_adcAcq;
        const ADC_IT_TypeDef*   p_adcIT     = p_adcCfg->p_adcIt;
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

            if(p_adcCfg->p_adcIt != NULL)
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
            p_ITConv = p_halMotorCfg->p_ITAcq;
            // Wait IT flag event
            while(ADC_getInterruptStatus(p_ITConv->adcBase, p_ITConv->adcIntNumber) == false);
            if (calibCnt > 1000)
                for(loop = 0; loop < 3; loop++)
                {
                    i[loop] = 0.998f * i[loop] + 0.002f * (*p_halMotorCfg->p_iAcq[loop]->adcResultReg);
                    v[loop] = 0.998f * v[loop] + 0.002f * (*p_halMotorCfg->p_vAcq[loop]->adcResultReg);
                }
            else
                for(loop = 0; loop < 3; loop++)
                {
                    i[loop] = *(p_halMotorCfg->p_iAcq[loop]->adcResultReg);
                    v[loop] = *(p_halMotorCfg->p_vAcq[loop]->adcResultReg);
                }

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
        {
            // Sets the post processing block reference offset for tensions
            ADC_setPPBReferenceOffset(p_halMotorCfg->p_vAcq[loop]->adcBase, p_halMotorCfg->p_vAcq[loop]->adcPPBNumber, (uint16_t)v[loop]);
            // Sets the post processing block reference offset for tensions
            ADC_setPPBReferenceOffset(p_halMotorCfg->p_iAcq[loop]->adcBase, p_halMotorCfg->p_iAcq[loop]->adcPPBNumber, (uint16_t)i[loop]);
            // Enables two's complement capability
//            ADC_enablePPBTwosComplement(p_halMotorCfg->p_iAcq[loop]->adcBase, p_halMotorCfg->p_iAcq[loop]->adcPPBNumber);
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
    if((p_pwmCfg != NULL) && ( p_pwmCfg->p_epwmTimeBase != NULL) && (p_pwmCfg->p_epwmCaptureCompare) &&
            (p_pwmCfg->p_epwmActionQualifier != NULL) && (p_pwmCfg->p_epwmDeadband != NULL))
    {
        const ePWM_TB_TypeDef* p_pwmTimeBase        = p_pwmCfg->p_epwmTimeBase;
        const ePWM_CC_TypeDef* p_pwmCounterCompare  = p_pwmCfg->p_epwmCaptureCompare;
        const ePWM_AQ_TypeDef* p_pwmActionQualifier = p_pwmCfg->p_epwmActionQualifier;
        const ePWM_DB_TypeDef* p_pwmDeadBand        = p_pwmCfg->p_epwmDeadband;
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
            if(p_pwmCfg->p_epwmProgChopper != NULL)
            {
                const ePWM_PC_TypeDef* p_pwmProgChopper = p_pwmCfg->p_epwmProgChopper;

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
                const ePWM_ET_TypeDef* p_pwmAdcSocEvtTrigger = p_pwmCfg->p_epwmAdcSocEventTrigger;

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
                const ePWM_IT_TypeDef* p_pwmInterruptsEvent = p_pwmCfg->p_epwmInterruptsEvent;

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
        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
        EPWM_forceSyncPulse(MOTOR1_PWM1_BASE);
    }

    return;
}

/**
 * @brief       EQEP (Quadrature encoder reading) initialization function.
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
            /* Clear flags & enable or disable interrupts */
            SPI_clearInterruptStatus(spiBase, SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RX_OVERRUN | SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            SPI_disableInterrupt(spiBase, SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RX_OVERRUN | SPI_INT_TXFF | SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW);
            (p_spiCfg->intEnable == true) ? (SPI_enableInterrupt(spiBase, p_spiCfg->intSrc)) : (1);
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
#ifdef DEBUG
        DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);
#endif

        while(dmaChBase != 0xFFFFFFFF)
        {
            /* DMA channel initialization
             * Configure direction, words per burst
             */
            DMA_configAddresses(dmaChBase, p_dmaCfg->p_dstAddr, p_dmaCfg->p_srcAddr);
            DMA_configBurst(dmaChBase, p_dmaCfg->burstSize, p_dmaCfg->srcStep, p_dmaCfg->dstStep);
            DMA_configTransfer(dmaChBase, p_dmaCfg->transferSize, p_dmaCfg->srcStep, p_dmaCfg->dstStep);
            DMA_configWrap(dmaChBase, p_dmaCfg->srcWrapSize, p_dmaCfg->srcWrapStep, p_dmaCfg->dstWrapSize, p_dmaCfg->dstWrapStep);
            DMA_configMode(dmaChBase, p_dmaCfg->trigger, p_dmaCfg->config);
            /* Clear flags & disable interrupts */
            DMA_disableOverrunInterrupt(dmaChBase);
            DMA_setInterruptMode(dmaChBase, p_dmaCfg->intMode);
            DMA_enableInterrupt(dmaChBase);
//            (p_dmaCfg->intEnable) ? (DMA_enableInterrupt(dmaChBase)) : (DMA_disableInterrupt(dmaChBase));
            /* Enable Trigger source & DMA channel */
            DMA_clearTriggerFlag(dmaChBase);
            DMA_enableTrigger(dmaChBase);
//            DMA_startChannel(dmaChBase);
            dmaChBase = (++p_dmaCfg)->dmaChBase;
        }
    }

    return;
}

/**
 * @brief       DMA start function.
 * @param[in]   *p_dmaCfg   Pointer on the DMA configuration structure to be started
 */
void HAL_DMA_start(const DMA_TypeDef* p_dmaCfg)
{
    if(p_dmaCfg != NULL)
    {
        uint32_t dmaChBase = p_dmaCfg->dmaChBase;
        while(dmaChBase != 0xFFFFFFFF)
        {
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
void HAL_IPC_init(const IPC_TypeDef* p_ipcCfg)
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

void* HAL_IPC_addr_init(const IPC_TypeDef* p_ipcCfg, uint32_t cmd_valid)
{
    uint32_t cmd, data, addr;
    bool addrCorr = ((p_ipcCfg->ipcType == IPC_CPU1_L_CM_R) || (p_ipcCfg->ipcType == IPC_CPU2_L_CM_R)) ?
            (IPC_ADDR_CORRECTION_ENABLE) : (IPC_ADDR_CORRECTION_DISABLE);
    // Memory address sharing CM to CPU1
    IPC_waitForFlag(p_ipcCfg->ipcType, p_ipcCfg->ipcFlag);
    IPC_readCommand(p_ipcCfg->ipcType, p_ipcCfg->ipcFlag, addrCorr, &cmd, &addr, &data);
    IPC_ackFlagRtoL(p_ipcCfg->ipcType, p_ipcCfg->ipcFlag);

    return((cmd == cmd_valid) ? ((void *)addr) : (NULL));
}

#if (CLA == 1)
/**
 * @brief       CLA initialization function.
 * @param[in]   *p_claCfg   Pointer on the CLA configuration structure
 */
void HAL_CLA_init(const CLA_TypeDef* p_clacCfg)
{
    if(p_clacCfg != NULL)
    {
#ifdef _FLASH
        extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
        extern uint32_t Cla1ConstRunStart, Cla1ConstLoadStart, Cla1ConstLoadSize;
        // Copy over code from FLASH to RAM
        memcpy((uint32_t *)&Cla1ProgRunStart,   (uint32_t *)&Cla1ProgLoadStart,     (uint32_t)&Cla1ProgLoadSize);
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
            if(p_clacCfg->progRAM & memcfg_sect_lsx)
            {
                MemCfg_setLSRAMMasterSel(memcfg_sect_lsx, MEMCFG_LSRAMMASTER_CPU_CLA1);
                MemCfg_setCLAMemType(memcfg_sect_lsx, MEMCFG_CLA_MEM_PROGRAM);
            }
            /* Configure LSxRAM as data spaces for the CLA
             * First configure the CLA to be the master for LSx and then
             * set the spaces to be code blocks */
            if(p_clacCfg->dataRAM & memcfg_sect_lsx)
            {
                MemCfg_setLSRAMMasterSel(memcfg_sect_lsx,MEMCFG_LSRAMMASTER_CPU_CLA1);
                MemCfg_setCLAMemType(memcfg_sect_lsx, MEMCFG_CLA_MEM_DATA);
            }
        }
        // TODO : CLA Task config
//        void CLA_initCpu1Cla1(void)
//        {
//            /* Compute all CLA task vectors
//             * On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
//             * opposed to offsets used on older Type-0 CLAs */
//
//        #pragma diag_suppress=770
//            CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_1, (uint16_t)(&Cla1Task1));
//            CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_8, (uint16_t)(&Cla1Task8));
//        #pragma diag_warning=770
//
//            /* Enable the IACK instruction to start a task on CLA in software
//            * for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
//            * subset of tasks) by writing to their respective bits in the
//            * MIER register */
//        //    CLA_enableIACK(CLA1_BASE);
            CLA_setTriggerSource(CLA_TASK_1, CLA_TRIGGER_EPWM1INT);//CLA_TRIGGER_ADCA3);
            CLA_setTriggerSource(CLA_TASK_8, CLA_TRIGGER_SOFTWARE);
            CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_1 | CLA_TASKFLAG_8);//CLA_TASKFLAG_ALL);
//
//            /* <Step 6>: Enable software forcing and run task 8 once
//             *    - The MCTL register allows for a task to be triggered by software
//             *      see TRM 5.7.2.9
//             *    - Run any one-time tasks by using the software force. "F2806x_Cla_defines.h"
//             *      has some useful macro functions to force each task */
//            CLA_enableIACK(CLA1_BASE);//    Cla1Regs.MCTL.bit.IACKE    = 1;
//        //    Cla1ForceTask8andWait();
//        //    EALLOW;
//        //    Cla1Regs.MCTL.bit.IACKE    = 1;
//        //    Cla1Regs.MIFRC.bit.INT8 = 1;
//        //    EDIS;
//            Cla1ForceTask8();
//            /* Configure the vectors for the end-of-task interrupt for all 8 tasks */
//        //    Interrupt_register(INT_CLA1_1, &cla1_isr1);
//        //    Interrupt_register(INT_CLA1_2, &cla1Isr2);
//        //    Interrupt_register(INT_CLA1_3, &cla1Isr3);
//        //    Interrupt_register(INT_CLA1_4, &cla1Isr4);
//        //    Interrupt_register(INT_CLA1_5, &cla1Isr5);
//        //    Interrupt_register(INT_CLA1_6, &cla1Isr6);
//        //    Interrupt_register(INT_CLA1_7, &cla1Isr7);
//        //    Interrupt_register(INT_CLA1_8, &cla1Isr8);
//            /* Enable CLA interrupts at the group and subgroup levels */
//        //    Interrupt_enable(INT_CLA1_1);
//        //    Interrupt_enable(INT_CLA1_2);
//        //    Interrupt_enable(INT_CLA1_3);
//        //    Interrupt_enable(INT_CLA1_4);
//        //    Interrupt_enable(INT_CLA1_5);
//        //    Interrupt_enable(INT_CLA1_6);
//        //    Interrupt_enable(INT_CLA1_7);
//        //    Interrupt_enable(INT_CLA1_8);
//        }

    }

    return;
}
#endif

/**
 * @brief       Interrupts initialization function.
 */
void HAL_IT_init(const IT_TypeDef* p_intCfg)
{
    if(p_intCfg != NULL)
    {
        uint32_t intNum = p_intCfg->intNum;
        // Clear & set IT flags & enable IT bits in a known state.
        Interrupt_initModule();
        // Setup Interrupts
        while(intNum != 0xFFFFFFFF)
        {
            // Attach ISR to interrupt
            Interrupt_register(intNum, p_intCfg->p_intHandler);
            // Enable Interrupts
            Interrupt_enable(intNum);
            intNum = (++p_intCfg)->intNum;
        }
        // enable global interrupt (INTM)
        EINT;
        // Enable realtime interrupt (DGBM)
        ERTM;
    }

    return;
}
