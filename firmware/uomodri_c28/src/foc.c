/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "driverlib.h"
#include "device.h"
#include "foc.h"
#include "communication.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void FOC_resetMeasuresStruct(acq_t*);
inline void FOC_resistanceEstimation(foc_t*);
inline void FOC_resetPI(pi_t*);
inline float32_t FOC_runPI(pi_t*);
inline void FOC_resetPD(pd_t*);
inline void FOC_resetCmdStruct(cmd_t*);
void limit_norm(float32_t*, float32_t*, float32_t);
void FOC_svpwm1(foc_t*);
void FOC_svpwm2(foc_t*);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief           Read the phases current and phases \& bus voltages
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_getMeasures(acq_t* p_acq)
{
    // Get the latest current values in Amps
    p_acq->ia       = (float32_t)(*p_acq->p_iaMeasReg) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ib       = (float32_t)(*p_acq->p_ibMeasReg) * ADC_MOTOR_CURRENT_SCALE;
    p_acq->ic       = (float32_t)(*p_acq->p_icMeasReg) * ADC_MOTOR_CURRENT_SCALE;
    // Get the latest Voltage supply value in Volt (minimum is one to avoid dividing by zero later)
    float32_t vbus  = __fmax(((float32_t)(*p_acq->p_vBusMeasReg) * ADC_VBUS_VOLTAGE_SCALE), 1.0f);
    UOMODRI_VBUS_FLT(p_acq->vBusFlt, p_acq->vbus, vbus);
    float32_t vext  = (float32_t)(*p_acq->p_vExtMeasReg) * ADC_VEXT12_VOLTAGE_SCALE;
    UOMODRI_VEXT_FLT(p_acq->vExtFlt, p_acq->vExt, vext);

    return;
}

/**
 * @brief           Reset the acquisition structure - Only I & Vext
 * @param[inout]    *p_acq  Pointer on the motor acquisition structure.
 */
inline void FOC_resetMeasuresStruct(acq_t* p_acq)
{
    // Get the latest current values in Amps
    p_acq->ia   = 0.0f;
    p_acq->ib   = 0.0f;
    p_acq->ic   = 0.0f;
    p_acq->vExt = 0.0f;

    return;
}

/**
 * @brief       Compute sine and cosine from the electrical angle
 * @param[in]   *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_resistanceEstimation(foc_t* p_foc)
{
    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
        UOMODRI_RES_ESTIM_FLT(p_foc->resEstFlt, p_foc->resEst, (p_foc->uq - p_foc->motor_enc.speed.speedElec * p_foc->motor_cfg.ke) / p_foc->iq);

    return;
}

/**
 * @brief           Compute the Field Oriented Control (FOC) algorithm
 * @param[inout]    *p_foc  Pointer on the FOC motor control structure.
 */
inline void FOC_runControl(foc_t* p_foc)
{
    acq_t*          p_acq   = &p_foc->motor_acq;
    encoder_t*      p_enc   = &p_foc->motor_enc;
    /* Compute sine and cosine from the electrical angle */
    p_foc->sinTheta         = __sin(p_enc->thetaElec);
    p_foc->cosTheta         = __cos(p_enc->thetaElec);
    // Clarke transform
    p_foc->ialpha           = ((2.0f * p_acq->ia) - p_acq->ib - p_acq->ic) * FM_1DIV3;
    p_foc->ibeta            = (p_acq->ib - p_acq->ic) * FM_1DIVSQRT3;
    // Park transform + filtering
    float32_t id            = (p_foc->sinTheta * p_foc->ibeta) + (p_foc->cosTheta * p_foc->ialpha);
    UOMODRI_IDQ_FLT(p_foc->iParkFlt, p_foc->id, id);
    float32_t iq            = (p_foc->cosTheta * p_foc->ibeta) - (p_foc->sinTheta * p_foc->ialpha);
    UOMODRI_IDQ_FLT(p_foc->iParkFlt, p_foc->iq, iq);
    // Compute decoupling feed-forward terms
#if (UOMODRI_FEED_FORWARD_ENABLE)
    const params_t* p_cfg   = &p_foc->motor_cfg;
    speed_t*        p_speed = &p_enc->speed;
    p_foc->piId.ff          = -1.0f * p_speed->speedMech[0] * p_cfg->Ls * p_foc->iq;
    p_foc->piIq.ff          = (p_speed->speedElec * p_cfg->Ls * p_foc->id) + (p_speed->speedMech[0] * p_cfg->ke);
#else
    p_foc->piId.ff          = 0.0f;
    p_foc->piIq.ff          = 0.0f;
#endif
    // Compute Id(Flux Current) and Iq (Torque Current) PI controllers
    p_foc->ud               = FOC_runPI(&p_foc->piId);
    p_foc->uq               = FOC_runPI(&p_foc->piIq);
    p_foc->vmax             = p_acq->vbus * FM_1DIVSQRT3;
//    p_foc->udMax            = p_foc->vmax;
//    p_foc->uqMax            = __sqrt((p_foc->vmax * p_foc->vmax) - (p_foc->ud * p_foc->ud));
//    float32_t v_ref         = __sqrt((p_foc->ud * p_foc->ud) + (p_foc->uq * p_foc->uq));
//    limit_norm(&p_foc->ud, &p_foc->uq, p_foc->vmax);
    // Inverse Park transform
    p_foc->ualpha           = (p_foc->cosTheta * p_foc->ud) - (p_foc->sinTheta * p_foc->uq);
    p_foc->ubeta            = (p_foc->sinTheta * p_foc->ud) + (p_foc->cosTheta * p_foc->uq);

    FOC_svpwm1(p_foc);

    return;
}

/**
 * @brief           Reset the Proportional Integral (PI) Controller structure
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 */
inline void FOC_resetPI(pi_t* p_pi)
{
    p_pi->err       = 0.0;
    p_pi->integral  = 0.0;
    p_pi->ff        = 0.0;
    p_pi->out       = 0.0;

    return;
}

/**
 * @brief           Run the Proportional Integral Controller (PI)
 * @param[inout]    *p_pi   Pointer on the PI control structure.
 * @return          out     Command of PI controller
 */
inline float32_t FOC_runPI(pi_t* p_pi)
{
    float32_t sat_max   = 0.5f  * (*p_pi->p_sat);
    float32_t sat_min   = -1.0f * sat_max;
    p_pi->err           = (*p_pi->p_set) - (*p_pi->p_fb);
    p_pi->integral     += (p_pi->ki * p_pi->err * (float32_t)PWM_PERIOD);
    p_pi->integral      = __fsat(p_pi->integral, sat_max, sat_min);
    p_pi->out           = (p_pi->kp * p_pi->err) + p_pi->integral + p_pi->ff;
    p_pi->out           = __fsat(p_pi->out, sat_max, sat_min);

    return(p_pi->out);
}

/**
 * @brief           Reset the Proportional Derivative (PD) Controller structure
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 */
inline void FOC_resetPD(pd_t* p_pd)
{
    p_pd->errTheta      = 0.0f;
    p_pd->errSpeed      = 0.0f;
    p_pd->derivative    = 0.0f;
    p_pd->out           = 0.0f;

    return;
}

/**
 * @brief           Run the Proportional Derivative (PD) Controller
 * @param[inout]    *p_pd   Pointer on the PD control structure.
 * @return          out     Command of PD controller
 */
inline float32_t FOC_runPD(pd_t* p_pd)
{
    float32_t sat_max   = 1.0f  * (*p_pd->p_sat);
    float32_t sat_min   = -1.0f * sat_max;
    p_pd->errTheta      = (*p_pd->p_setTheta) - (*p_pd->p_fbTheta);
    p_pd->errSpeed      = (*p_pd->p_setSpeed) - (*p_pd->p_fbSpeed);
    p_pd->derivative    = (*p_pd->p_kd) * p_pd->errSpeed;
    // Compute PID and clamp output.
    p_pd->out           = ((*p_pd->p_kp) * p_pd->errTheta) + p_pd->derivative + (*p_pd->p_ff);
    p_pd->out           = __fsat(p_pd->out, sat_max, sat_min);

    return(p_pd->out);
}

/**
 * @brief           Reset the command structure associated to motor control
 * @param[inout]    p_cmd   Pointer on the local command associated to motor control
 */
inline void FOC_resetCmdStruct(cmd_t* p_cmd)
{
    p_cmd->posRef               = 0.0f;
    p_cmd->velRef               = 0.0f;
    p_cmd->iqff                 = 0.0f;
    p_cmd->kpCoeff              = 0.0f;
    p_cmd->kdCoeff              = 0.0f;
    p_cmd->iSat                 = 0.0f;//(p_cmd->motor_id == MOTOR_1) ? (MOTOR1_CURRENT_REGULATOR_SAT_MAX) : (MOTOR2_CURRENT_REGULATOR_SAT_MAX);
    p_cmd->timeoutRef           = 0;
    p_cmd->cptTimeout           = 0;
    p_cmd->enableReg.all        = 0;
//    p_cmd->encOffsetCompEnable  = false;
//    p_cmd->rollOverEnable       = false;
//    p_cmd->motorEnable          = false;
//    p_cmd->systemEnable         = false;
//    p_cmd->cmdAccesValid        = false;
}

/**
 * @brief           Reset the FOC structure
 * @param[inout]    *p_foc   Pointer on the FOC structure to reset.
 */
void FOC_resetStruct(foc_t* p_foc)
{
    // Reset Id & Iq currents PI controllers
    FOC_resetPI(&p_foc->piId);
    FOC_resetPI(&p_foc->piIq);
    // Reset position PD controller
    FOC_resetPD(&p_foc->pdPosVel);
//    FOC_resetPD(&p_foc->pdPosVel[EVEN]);
    // Reset encoder structure
    ENC_resetStruct(&p_foc->motor_enc);
    // Reset analog measures
    FOC_resetMeasuresStruct(&p_foc->motor_acq);
    // Reset the commands associated to motor control
    FOC_resetCmdStruct(&p_foc->motor_cmd);
    // Reset currents and voltages
    p_foc->sinTheta         = 0.0f;
    p_foc->cosTheta         = 0.0f;
    p_foc->ialpha           = 0.0f;
    p_foc->ibeta            = 0.0f;
    p_foc->id               = 0.0f;
    p_foc->iq               = 0.0f;
    p_foc->ud               = 0.0f;
    p_foc->uq               = 0.0f;
    p_foc->ualpha           = 0.0f;
    p_foc->ubeta            = 0.0f;
    p_foc->ua               = 0.0f;
    p_foc->ub               = 0.0f;
    p_foc->uc               = 0.0f;
    p_foc->vmax             = 0.0f;
    p_foc->dtc_u            = 0.0f;
    p_foc->dtc_v            = 0.0f;
    p_foc->dtc_w            = 0.0f;
//    p_foc->idRef            = 0.0f;
//    p_foc->iqRef            = 0.0f;
    p_foc->resEst           = 0.0f;
//    p_foc->cmd_cr_active    = ODD;
//    p_foc->cmd_nx_active    = ODD;

    return;
}

/**
 * @brief           Scale the length of vector (x, y) to be <= limit
 * @param[out]      *p_x    Pointer on the first element to normalize.
 * @param[out]      *p_y    Pointer on the second element to normalize.
 * @param[in]       limit   Limit value.
 */
void limit_norm(float32_t *p_x, float32_t *p_y, float32_t limit)
{
    float32_t norm = __sqrtf((*p_x) * (*p_x) + (*p_y) * (*p_y));
    if(norm > limit)
    {
        *p_x *= limit / norm;
        *p_y *= limit / norm;
    }
}

/**
 * @brief           First implementation of the Space Vector Modulation (SVM)
 * @param[inout]    *p_foc   Pointer on the FOC structure.
 */
void FOC_svpwm1(foc_t* p_foc)
{
    // Inverse Clarke transform
    p_foc->ua               = p_foc->ualpha;
    p_foc->ub               = (-0.5f * p_foc->ualpha) + (FM_SQRT3DIV2 * p_foc->ubeta);
    p_foc->uc               = (-0.5f * p_foc->ualpha) - (FM_SQRT3DIV2 * p_foc->ubeta);
    // Compute middle voltage to center PWM duty cycles around it
    float32_t   uOffset     = 0.5f * (FMIN3(p_foc->ua, p_foc->ub, p_foc->uc) + FMAX3(p_foc->ua, p_foc->ub, p_foc->uc));
    float32_t   uMidPoint   = 0.5f * (p_foc->dtcMax + p_foc->dtcMin);
    // Compute duty cycles
    p_foc->dtc_u            = __fsat((((p_foc->ua - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_v            = __fsat((((p_foc->ub - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
    p_foc->dtc_w            = __fsat((((p_foc->uc - uOffset) / p_foc->motor_acq.vbus) + uMidPoint), p_foc->dtcMax, p_foc->dtcMin);
}

/**
 * @brief           Second implementation of the Space Vector Modulation (SVM)
 * @param[inout]    *p_foc   Pointer on the FOC structure.
 */
void FOC_svpwm2(foc_t* p_foc)
{
    p_foc->ualpha          /= p_foc->vmax;
    p_foc->ubeta           /= p_foc->vmax;

    p_foc->ua               =  p_foc->ubeta;
    p_foc->ub               = (-0.5f * p_foc->ubeta) + (FM_SQRT3DIV2 * p_foc->ualpha);
    p_foc->uc               = (-0.5f * p_foc->ubeta) - (FM_SQRT3DIV2 * p_foc->ualpha);

//    svm_test_reg_u  svm_test;
//    svm_test.bit.A_TEST     = (p_foc->ua > 0.0f) ? (1) : (0);   /* Ubeta > 0.0 */
//    svm_test.bit.B_TEST     = (p_foc->ub > 0.0f) ? (1) : (0);   /* (- 1 / 2 x Ubeta) + (sqrt(3) / 2 x Ualpha) > 0.0 */
//    svm_test.bit.C_TEST     = (p_foc->uc > 0.0f) ? (1) : (0);   /* (- 1 / 2 x Ubeta) - (sqrt(3) / 2 x Ualpha) > 0.0 */
//    svm_test.bit.TEST_RSV1  = 0;
    svm_sector_e svm_test   = (svm_sector_e)(((p_foc->ua > 0.0f) * 4) + ((p_foc->ub > 0.0f) * 2) + (p_foc->uc > 0.0f));

    switch(svm_test)
    {
    case MOTOR_SVM_SECTOR_0:
    case MOTOR_SVM_SECTOR_7:
        // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
        p_foc->dtc_u        = 0.5f;
        p_foc->dtc_v        = 0.5f;
        p_foc->dtc_w        = 0.5f;
        break;

    case MOTOR_SVM_SECTOR_1:
    case MOTOR_SVM_SECTOR_4:
        // Sector 1: t1=y   and t2=x    (abc ---> Ta,Tb,Tc)
        // Sector 4: t1=-x  and t2=-y   (abc ---> Tc,Tb,Ta)
        p_foc->dtc_u        = 0.5f * (1.0f + p_foc->uc);
        p_foc->dtc_v        = 0.5f * (1.0f + p_foc->ub - p_foc->ua);
        p_foc->dtc_w        = 0.5f * (1.0f - p_foc->uc);
        break;

    case MOTOR_SVM_SECTOR_2:
    case MOTOR_SVM_SECTOR_5:
        // Sector 2: t1=-y  and t2=-z   (abc ---> Tb,Ta,Tc)
        // Sector 5: t1=z   and t2=y    (abc ---> Tb,Tc,Ta)
        p_foc->dtc_u        = 0.5f * (1.0f + p_foc->uc - p_foc->ub);
        p_foc->dtc_v        = 0.5f * (1.0f - p_foc->ua);
        p_foc->dtc_w        = 0.5f * (1.0f + p_foc->ua);
        break;

    case MOTOR_SVM_SECTOR_3:
    case MOTOR_SVM_SECTOR_6:
        // Sector 3: t1=x   and t2=z    (abc ---> Tc,Ta,Tb)
        // Sector 6: t1=-z   and t2=-x  (abc ---> Ta,Tc,Tb)
        p_foc->dtc_u        = 0.5f * (1.0f - p_foc->ub);
        p_foc->dtc_v        = 0.5f * (1.0f + p_foc->ub);
        p_foc->dtc_w        = 0.5f * (1.0f + p_foc->ua - p_foc->uc);
        break;
    }

    return;
}
