#include "main.h"

const float32_t Sinus1kHz[200] = {0.5, 0.510993765677345, 0.521976681835260, 0.532937909661480, 0.543866631747507, 0.554752062764081, \
    0.565583460105004, 0.576350134488790, 0.587041460507699, 0.597646887113730, 0.608155948031232, 0.618558272085852, \
    0.628843593439637, 0.639001761722173, 0.649022752047775, 0.658896674908841, 0.668613785935600, 0.678164495512630, \
    0.687539378242649, 0.696729182248246, 0.705724838302366, 0.714517468778542, 0.723098396412041, 0.731459152863278, \
    0.739591487075041, 0.747487373415292, 0.755139019597494, 0.762538874370661, 0.769679634971526, 0.776554254331492, \
    0.783155948031232, 0.789478200996097, 0.795514773925705, 0.801259709451380, 0.806707338015352, 0.811852283465929, \
    0.816689468363107, 0.821214118989393, 0.825421770060888, 0.829308269133979, 0.832869780703304, 0.836102789986930, \
    0.839004106395021, 0.841570866678562, 0.843800537755041, 0.845690919208298, 0.847240145460067, 0.848446687611078, \
    0.849309354949895, 0.849827296128006, 0.85, 0.849827296128006, 0.849309354949895, 0.848446687611078, \
    0.847240145460067, 0.845690919208298, 0.843800537755041, 0.841570866678562, 0.839004106395021, 0.836102789986930, \
    0.832869780703304, 0.829308269133979, 0.825421770060888, 0.821214118989393, 0.816689468363107, 0.811852283465929, \
    0.806707338015352, 0.801259709451380, 0.795514773925705, 0.789478200996097, 0.783155948031232, 0.776554254331492, \
    0.769679634971526, 0.762538874370661, 0.755139019597494, 0.747487373415292, 0.739591487075041, 0.731459152863278, \
    0.723098396412041, 0.714517468778542, 0.705724838302366, 0.696729182248246, 0.687539378242649, 0.678164495512630, \
    0.668613785935601, 0.658896674908841, 0.649022752047775, 0.639001761722173, 0.628843593439637, 0.618558272085852, \
    0.608155948031232, 0.597646887113730, 0.587041460507699, 0.576350134488790, 0.565583460105004, 0.554752062764081, \
    0.543866631747506, 0.532937909661480, 0.521976681835260, 0.510993765677345, 0.5, 0.489006234322655, \
    0.478023318164741, 0.467062090338520, 0.456133368252494, 0.445247937235919, 0.434416539894997, 0.423649865511210, \
    0.412958539492301, 0.402353112886270, 0.391844051968769, 0.381441727914148, 0.371156406560363, 0.360998238277827, \
    0.350977247952225, 0.341103325091159, 0.331386214064400, 0.321835504487370, 0.312460621757351, 0.303270817751754, \
    0.294275161697634, 0.285482531221458, 0.276901603587959, 0.268540847136722, 0.260408512924959, 0.252512626584708, \
    0.244860980402506, 0.237461125629339, 0.230320365028474, 0.223445745668509, 0.216844051968769, 0.210521799003903, \
    0.204485226074295, 0.198740290548620, 0.193292661984648, 0.188147716534071, 0.183310531636893, 0.178785881010607, \
    0.174578229939112, 0.170691730866021, 0.167130219296696, 0.163897210013070, 0.160995893604979, 0.158429133321439, \
    0.156199462244959, 0.154309080791702, 0.152759854539933, 0.151553312388922, 0.150690645050105, 0.150172703871994, \
    0.15, 0.150172703871994, 0.150690645050105, 0.151553312388922, 0.152759854539933, 0.154309080791702, \
    0.156199462244959, 0.158429133321438, 0.160995893604979, 0.163897210013070, 0.167130219296696, 0.170691730866021, \
    0.174578229939112, 0.178785881010607, 0.183310531636893, 0.188147716534071, 0.193292661984648, 0.198740290548620, \
    0.204485226074295, 0.210521799003903, 0.216844051968768, 0.223445745668508, 0.230320365028474, 0.237461125629339, \
    0.244860980402506, 0.252512626584708, 0.260408512924959, 0.268540847136722, 0.276901603587959, 0.285482531221458, \
    0.294275161697634, 0.303270817751754, 0.312460621757351, 0.321835504487370, 0.331386214064400, 0.341103325091159, \
    0.350977247952224, 0.360998238277827, 0.371156406560363, 0.381441727914148, 0.391844051968768, 0.402353112886270, \
    0.412958539492301, 0.423649865511210, 0.434416539894996, 0.445247937235919, 0.456133368252493, 0.467062090338520, \
    0.478023318164740, 0.489006234322655};

// FOC structure declarations
//extern HAL hal;
//extern FOC foc[];

/**
 * main.c
 */
int main(void)
{
    uint8_t i[3] = {0, 65, 132}, j[3] = {0, 65, 132}, ch = 0;

    // Control card devices initializations
    Mcu_init();
    // Hardware Abstraction Layer initialization
    HAL_init(&hal);
    // DRV8323 initialization
    DRV_init(motor[MOTOR_1].motorDRV);
    DRV_init(motor[MOTOR_2].motorDRV);
    // ADC offset calib for debug
    HAL_ADC_offsetCalib(motor[MOTOR_1].motorHalCfg);
    HAL_ADC_offsetCalib(motor[MOTOR_2].motorHalCfg);
    // Interrupts initialization
    HAL_IT_init(InterruptList);

    // Reset controllers
//    CL_resetFoc(&foc[MOTOR_1]);
//    CL_resetFoc(&foc[MOTOR_2]);
    // Initialize FOC structures
//    CL_initFoc(&motor_foc[MOTOR_1], MOTOR_1);
//    CL_initFoc(&motor_foc[MOTOR_2], MOTOR_2);

    // PWM test section, replace 0 by an integer between 0x0000 and 0xFFFF
//    EPWM_setCounterCompareValue(MOTOR1_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR1_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR1_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR2_PWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR2_PWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
//    EPWM_setCounterCompareValue(MOTOR2_PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);

//    (*((volatile uint16_t *)((uintptr_t)(base + registerOffset + 0x1U))))

//    MOTOR_SetPwmCmd(&motor[MOTOR_1], 3 * PWM_TBPRD / 4, 2 * PWM_TBPRD / 4, 1 * PWM_TBPRD / 4);
//    MOTOR_SetPwmCmd(&motor[MOTOR_2], 3 * PWM_TBPRD / 4, 2 * PWM_TBPRD / 4, 1 * PWM_TBPRD / 4);

    MOTOR_SetPwmCmd(&motor[MOTOR_1], Sinus1kHz[i[0]], Sinus1kHz[i[1]], Sinus1kHz[i[2]]);
    MOTOR_SetPwmCmd(&motor[MOTOR_2], Sinus1kHz[j[0]], Sinus1kHz[j[1]], Sinus1kHz[j[2]]);

//    *(motor[MOTOR_1].pwmChAReg) = 3 * PWM_TBPRD / 4;
//    *(motor[MOTOR_1].pwmChBReg) = 2 * PWM_TBPRD / 4;
//    *(motor[MOTOR_1].pwmChCReg) = 1 * PWM_TBPRD / 4;
//    *(motor[MOTOR_2].pwmChAReg) = 3 * PWM_TBPRD / 4;
//    *(motor[MOTOR_2].pwmChBReg) = 2 * PWM_TBPRD / 4;
//    *(motor[MOTOR_2].pwmChCReg) = 1 * PWM_TBPRD / 4;

    // Infinite loop
    while(1)
    {
        if(motor[MOTOR_1].motorEncoder->posNew == true)
        {
            // Compute sine cosine
            FOC_sineCosine(motor[MOTOR_1].motorFOC);
            motor[MOTOR_1].motorEncoder->posNew = false;
        }

        if(motor[MOTOR_2].motorEncoder->posNew == true)
        {
            // Compute sine cosine
            FOC_sineCosine(motor[MOTOR_2].motorFOC);
            motor[MOTOR_2].motorEncoder->posNew = false;
        }

        if(motor[MOTOR_1].motorMeas->MeasNew == true)
        {
            for (ch = 0; ch < 3; ch++)
                i[ch] = (i[ch] < 199) ? (i[ch] + 1) : (0);

            MOTOR_SetPwmCmd(&motor[MOTOR_1], Sinus1kHz[i[0]], Sinus1kHz[i[1]], Sinus1kHz[i[2]]);
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_1].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_1].motorFOC);
//            // Voltage transforms
//            FOC_invParkClarke(motor[MOTOR_1].motorFOC);
//            // Update PWM compare registers
//            FOC_svm(motor[MOTOR_1].motorFOC);
            motor[MOTOR_1].motorMeas->MeasNew = false;
        }

        if(motor[MOTOR_2].motorMeas->MeasNew == true)
        {
            for (ch = 0; ch < 3; ch++)
                j[ch] = (j[ch] < 199) ? (j[ch] + 1) : (0);

            MOTOR_SetPwmCmd(&motor[MOTOR_2], Sinus1kHz[j[0]], Sinus1kHz[j[1]], Sinus1kHz[j[2]]);
//            // Current transforms
//            FOC_parkClarke(motor[MOTOR_2].motorFOC);
//            // Run PI controllers
//            FOC_runPi(motor[MOTOR_2].motorFOC);
//            // Voltage transforms
//            FOC_invParkClarke(motor[MOTOR_2].motorFOC);
//            // Update PWM compare registers
//            FOC_svm(motor[MOTOR_2].motorFOC);
            motor[MOTOR_2].motorMeas->MeasNew = false;
        }
    }
}

/**
 * @brief   Microcontroller initialization
 *          - Configure master clock to 200MHz
 *          - Enable GPIO clocking
 *          - Configure the interruption vector table
 */
static void Mcu_init(void)
{
    // Initialize clock and peripherals
    Device_init();
    // Initialize GPIO
    Device_initGPIO();
    // Initialize PIE control registers and flags
    Interrupt_initModule();
    // Initialize vector table
    Interrupt_initVectorTable();
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_1
 */
__interrupt void adc_isrMotor1(void)
{
    GPIO_writePin(DEBUG_INT_PIN1, 1);
    MOTOR_getVoltage(&motor[MOTOR_1]);
    MOTOR_getCurrent(&motor[MOTOR_1], &InterruptList[ADC_IT_MOTOR_1]);
    GPIO_writePin(DEBUG_INT_PIN1, 0);
}

/**
 * @brief   ADC interrupt. Read Va, Vb, Vc, Vbus, Ia, Ib \& Ic of MOTOR_2
 */
__interrupt void adc_isrMotor2(void)
{
    GPIO_writePin(DEBUG_INT_PIN2, 1);
    MOTOR_getVoltage(&motor[MOTOR_2]);
    MOTOR_getCurrent(&motor[MOTOR_2], &InterruptList[ADC_IT_MOTOR_2]);
    GPIO_writePin(DEBUG_INT_PIN2, 0);
}


////    if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4))
////    {
////        uint8_t loop = 0;
//
//        GPIO_writePin(DEBUG_INT_PIN2, 1);
////        motor_hal_cfg[MOTOR_2].dataValid[1] = true;
//
//        //        FOC_getPhaseVoltage(&foc[MOTOR_2], &motor_hal_cfg[MOTOR_2]);
////        for(loop = 0; loop < 3; loop++)
////            motor_hal_cfg[MOTOR_2].i[loop] = (float32_t)HWSREGH(motor_hal_cfg[MOTOR_2].p_iConv[loop]->adcPPBResultBase) * ADC_CURRENT_SCALE;
////        motor[MOTOR_2].ia = (float32_t)HWSREGH(motor[MOTOR_2].motorHalCfg->p_iConv[0]->adcPPBResultBase) * ADC_CURRENT_SCALE;
////        motor[MOTOR_2].ib = (float32_t)HWSREGH(motor[MOTOR_2].motorHalCfg->p_iConv[1]->adcPPBResultBase) * ADC_CURRENT_SCALE;
////        motor[MOTOR_2].ic = (float32_t)HWSREGH(motor[MOTOR_2].motorHalCfg->p_iConv[2]->adcPPBResultBase) * ADC_CURRENT_SCALE;
//
//        motor[MOTOR_2].motorMeas->ia    = (float32_t)(*motor[MOTOR_2].motorMeas->iaMeasReg) * ADC_CURRENT_SCALE;
//        motor[MOTOR_2].motorMeas->ib    = (float32_t)(*motor[MOTOR_2].motorMeas->ibMeasReg) * ADC_CURRENT_SCALE;
//        motor[MOTOR_2].motorMeas->ic    = (float32_t)(*motor[MOTOR_2].motorMeas->icMeasReg) * ADC_CURRENT_SCALE;
//        // Clear the interrupt overflow flag
//        HWREGH(ADCA_BASE + ADC_O_INTOVFCLR) = (1U << ADC_INT_NUMBER4);
//        //        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER4);
//        // Clear the interrupt flag
//        HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = (1U << ADC_INT_NUMBER4);
//        //        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER4);
//        //HWREG(PIECTRL_BASE + PIE_O_IFR10) = (1U << ADC_INT_NUMBER4);
//        // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
//        HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP10;
//        //        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
//
//        GPIO_writePin(DEBUG_INT_PIN2, 0);
////}
//}

__interrupt void pwm_isrMotor1(void)
{
    GPIO_writePin(DEBUG_INT_PIN1, 1);
    if (motor[MOTOR_1].motorFOC->enabled)
    {
        /* FOC update */
        // Read encoder position
        encoder_GetPosition(motor[MOTOR_1].motorEncoder);
        // Compute sine cosine
        FOC_sineCosine(motor[MOTOR_1].motorFOC);


        //encoder_GetSpeed(motor[MOTOR_1].motorEncoder);

        //bwhile((motor[MOTOR_1].motorFOC->adcIntFlag->all & (1<<MOTOR_1)) == 0);
    }
    else
    {
        // Don't send current to motors
        //        FOC_noVoltage(0);
        //
        //        // Reset controllers
        //        CL_resetFoc(&foc[0]);
    }

    //-------------------------------------------------------------------//
    //                          FOC update                               //
    //-------------------------------------------------------------------//
    // Read encoder position

    //    foc->encoder.qepHandle = (struct EQEP_REGS *)MOTOR1_QEP_BASE;
    //    foc->encoder.mechScaler = 0.25 / QEP_MOTOR1_ENCODER_LINES_NB;
    //    foc->encoder.polePairs = M1_POLES_PAIRS;
    //
    //
    //    ENC_read(&foc[motorIndex].encoder);


    //    if(EPWM_getEventTriggerInterruptStatus(MOTOR1_PWM1_BASE))
    //    EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);

    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(InterruptList[PWM_IT_MOTOR_1].intPeriphBase);
//    HWREGH(EPWM1_BASE + EPWM_O_ETCLR) |= EPWM_ETCLR_INT; // EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    //HWREG(PIECTRL_BASE + PIE_O_IFR3) = (1U << ADC_INT_NUMBER4);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    Interrupt_clearACKGroup(InterruptList[PWM_IT_MOTOR_1].intAckGroup);
//    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP3; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
    GPIO_writePin(DEBUG_INT_PIN1, 0);
}

__interrupt void pwm_isrMotor2(void)
{
    GPIO_writePin(DEBUG_INT_PIN2, 1);
    if (motor[MOTOR_2].motorFOC->enabled)
    {
        /* FOC update */
        // Read encoder position
        encoder_GetPosition(motor[MOTOR_2].motorEncoder);
        // Compute sine cosine
        FOC_sineCosine(motor[MOTOR_2].motorFOC);


        //encoder_GetSpeed(motor[MOTOR_1].motorEncoder);

        //bwhile((motor[MOTOR_1].motorFOC->adcIntFlag->all & (1<<MOTOR_1)) == 0);
    }
    else
    {
        // Don't send current to motors
        //        FOC_noVoltage(0);
        //
        //        // Reset controllers
        //        CL_resetFoc(&foc[0]);
    }

    //-------------------------------------------------------------------//
    //                          FOC update                               //
    //-------------------------------------------------------------------//
    // Read encoder position

    //    foc->encoder.qepHandle = (struct EQEP_REGS *)MOTOR1_QEP_BASE;
    //    foc->encoder.mechScaler = 0.25 / QEP_MOTOR1_ENCODER_LINES_NB;
    //    foc->encoder.polePairs = M1_POLES_PAIRS;
    //
    //
    //    ENC_read(&foc[motorIndex].encoder);


    //    if(EPWM_getEventTriggerInterruptStatus(MOTOR1_PWM1_BASE))
    //    EPWM_clearEventTriggerInterruptFlag(MOTOR1_PWM1_BASE);

    // Clear the interrupt flag
    EPWM_clearEventTriggerInterruptFlag(InterruptList[PWM_IT_MOTOR_2].intPeriphBase);
//    HWREGH(EPWM4_BASE + EPWM_O_ETCLR) |= EPWM_ETCLR_INT; // EPWM_clearEventTriggerInterruptFlag(EPWM4_BASE);
    // Acknowledge the interrupt (Acknowledge PIE Interrupt Group 10)
    Interrupt_clearACKGroup(InterruptList[PWM_IT_MOTOR_2].intAckGroup);
//    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP3; // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    GPIO_writePin(DEBUG_INT_PIN2, 0);
}
