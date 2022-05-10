/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "motor.h"
#include "encoder.h"

/***********************************************************************
 * LOCAL VARIABLES
 ***********************************************************************/

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       	Command for the 3 phases of the ePWM
 * @param[out]      p_motor Pointer on the associated motor structure
 * @param[in]       cmd_a   Normalized command on PWM channel A (between 0 and 1)
 * @param[in]       cmd_b   Normalized command on PWM channel B (between 0 and 1)
 * @param[in]       cmd_c   Normalized command on PWM channel C (between 0 and 1)
 */
inline void MOT_runCommand(motor_t* p_motor, float32_t cmd_a, float32_t cmd_b, float32_t cmd_c)
{
    *(p_motor->p_motorChAReg) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorChBReg) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT);
    *(p_motor->p_motorChCReg) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT);

    return;
}

/**
 * @brief           Force a hard stop on the 3 phases of the ePWM (low side active)
 * @param[out]      p_motor Pointer on the associated motor structure
 */
inline void MOT_stopCommand(motor_t* p_motor)
{
    *(p_motor->p_motorChAReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChBReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChCReg) = (uint16_t)PWM_TIMEBASE_CNT;

    return;
}

/**
 * @brief           Control the motor states
 * @param[inout]    p_motor Pointer on the associated motor structure
 * @param[in]       p_cmd   Pointer on the command structure driving motor
 */
inline bool_t MOT_runControl(motor_t* p_motor)
{
    foc_t*      p_foc           = p_motor->p_motorFOC;
    encoder_t*  p_enc           = &p_foc->motor_enc;
    cmd_t*      p_cmd           = &p_foc->motor_cmd;
    p_enc->indexCompensated     = p_cmd->encOffsetCompEnable;

    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    bool_t motor_rollover_flag  = p_enc->rollOverFlag && p_cmd->rollOverEnable;
    bool_t motor_drv_fault_flag = !GPIO_readPin(p_motor->p_motorDRV->p_drvCfgHandler->gpioNumber_FAULT);
    bool_t com_timeout_flag     = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef;
    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->p_motorHalCfg->p_pwmCntCmp[0]->epwmBase);
    p_motor->motor_error       |= (motor_rollover_flag)     ? (MOTOR_ERROR_POS_ROLLOVER): (p_motor->motor_error);
    p_motor->motor_error       |= (motor_drv_fault_flag)    ? (MOTOR_ERROR_DRV_NFAULT)  : (p_motor->motor_error);
    p_motor->motor_error       |= (com_timeout_flag)        ? (MOTOR_ERROR_COM_TIMEOUT) : (p_motor->motor_error);
    p_motor->motor_state        = (p_motor->motor_error)    ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);
//    p_foc->motor_state          = (p_foc->motor_error || p_cmd->mode.bit.RST)   ? (MOTOR_STATE_ERROR)       : (p_foc->motor_state);

    switch(p_motor->motor_state)
    {
    case MOTOR_STATE_INIT:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (p_cmd->systemEnable && p_cmd->motorEnable)   ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += 1e-4f; // Increment for alignment current procedure
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (p_foc->idRef < p_foc->iAlignMax)             ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_ALIGN_FIX);
        p_motor->motor_state    = (p_cmd->systemEnable && p_cmd->motorEnable)   ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ALIGN_FIX:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = p_foc->iAlignMax; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (p_motor->itCnt < (2 * PWM_FREQ))             ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY); // wait 2s
        p_motor->motor_state    = (p_cmd->systemEnable && p_cmd->motorEnable)   ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPos);
        p_motor->motor_state    = (p_cmd->motorEnable)                          ? (MOTOR_STATE_READY)       : (MOTOR_STATE_STOP);
        p_motor->motor_state    = (p_cmd->systemEnable)                         ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (!p_cmd->motorEnable)                         ? (MOTOR_STATE_STOP)        : (MOTOR_STATE_READY);
        p_motor->motor_state    = (p_cmd->systemEnable)                         ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = MOTOR_STATE_ERROR;
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;
    }

    return(true);
}

inline void MOT_resistorEstimation(motor_t* p_motor)
{

    foc_t*      p_foc   = p_motor->p_motorFOC;
    float32_t   resEst  = (p_foc->uq - (p_foc->motor_enc.speed.speedElec * p_foc->motor_cfg.ke)) / p_foc->iq;

    // Compute resistance
    // Only compute if measured current is over a threshold
    if (FM_ABS(p_foc->iq) > FOC_RE_CURRENT_THRESHOLD)
        UOMODRI_RES_ESTIM_FLT(p_foc->resEstFlt, p_foc->resEst, resEst);

    return;
}

/**
 * @brief       Process the space vector pulse width modulation.
 * @param[inout] p_foc  Pointer on a \a FocTypeDef structure pointing the current \a FOC structure.
 * @param   halfperiod  Timer auto reload value.
 * @return  error       Function Status.
 */
//void MOT_svpwm(motor_t* p_motor, foc_t* p_foc)
//{
//#if 1
//  float32_t tmp = FM_1DIVSQRT3 * p_foc->ubeta;
////  int32_t t6, t5, t4, t3, t2, t1, t0;
//  float32_t t6, t5, t4, t3, t2, t1, t0;
//  float32_t ta, tb, tc;
//  motor_svm_sector_e sector = SECTOR_0;
//
//  svm_test_reg_u svm_test;
////  svm_test.bit.UBETA_SIGN_TEST            = (p_foc->ubeta > 0.0f)                                         ? (1) : (0);
////  svm_test.bit.UALPHA_UBETA_SIGN_TEST     = (((FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta) > 0.0f)          ? (1) : (0);
////  svm_test.bit.NEG_UALPHA_UBETA_SIGN_TEST = (((-1.0f * FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta) > 0.0f)  ? (1) : (0);
////  svm_test.bit.SVM_TEST_RSV1              = 0;
////
//
//  float32_t u1                              = p_foc->ubeta;
//  float32_t u2                              = (FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta;
//  float32_t u3                              = (-1.0f * FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta;
//  svm_test.bit.UBETA_SIGN_TEST              = (u1 > 0.0f) ? (1) : (0);
//  svm_test.bit.UALPHA_UBETA_SIGN_TEST       = (u2 > 0.0f) ? (1) : (0);
//  svm_test.bit.NEG_UALPHA_UBETA_SIGN_TEST   = (u3 > 0.0f) ? (1) : (0);
//  svm_test.bit.SVM_TEST_RSV1                = 0;
//
//  float32_t tmp_x = p_foc->ubeta;
//  float32_t tmp_y = 0.5f * p_foc->ubeta + FM_SQRT3DIV2 * p_foc->ualpha;
//  float32_t tmp_z = tmp_y - tmp_x;// (-0.5f * p_foc->ubeta + FM_SQRT3DIV2 * p_foc->ualpha)
//
//  // PWM timings
//    switch (svm_test.all)
//    {
//      case SECTOR_1 : // sector 1-2
//      // Vector on-times
//          ta            = tmp_y;
//          tb            = tmp_x - tmp_z;
//          tc            = tmp_y;
//
//          t2            = (int32_t)(2.0f * FM_1DIVSQRT3 * p_foc->ubeta * PWM_TIMEBASE_CNT);
//          t1            = (int32_t)((p_foc->ualpha - (FM_1DIVSQRT3 * p_foc->ubeta)) * PWM_TIMEBASE_CNT);
//          t0            = PWM_TIMEBASE_CNT - t1 - t2;
//          // PWM timings
//          p_foc->dtc_u  = t0 / 2;
//          p_foc->dtc_v  = p_foc->dtc_u + t1;
//          p_foc->dtc_w  = p_foc->dtc_v + t2;
//      break;
//      case SECTOR_2:// sector 2-3
//      // Vector on-times
//          t0 = (-1.0f * p_foc->ualpha) + (p_foc->ubeta / FM_SQRT3);//u2 * FM_SQRT3;
//          t1 = p_foc->ualpha + (p_foc->ubeta / FM_SQRT3);// * 2.0 / FM_SQRT3;
//          t2 = 0.0f;
////      t3              = (int32_t) (-p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
////      t2              = (int32_t) (p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
////      t0              = PWM_TIMEBASE_CNT - t2 - t3;
////      // PWM timings
////      p_foc->dtc_v    = t0 / 2;
////      p_foc->dtc_u    = p_foc->dtc_v + t3;
////      p_foc->dtc_w    = p_foc->dtc_u + t2;
//      break;
//      case SECTOR_3:// sector 3-4
//      // Vector on-times
//          t0 = p_foc->ubeta * 2.0f / FM_SQRT3;////u2 * FM_SQRT3;
//          t1 = (-1.0f * p_foc->ualpha) + (p_foc->ubeta / FM_SQRT3);// * 2.0 / FM_SQRT3;
//          t2 = 0.0f;
////      t4              = (int32_t) (-p_foc->ualpha - tmp) * PWM_TIMEBASE_CNT;
////      t3              = (int32_t) (2.0 * tmp) * PWM_TIMEBASE_CNT;
////      t0              = PWM_TIMEBASE_CNT - t3 - t4;
////      // PWM timings
////      p_foc->dtc_v    = t0 / 2;
////      p_foc->dtc_w    = p_foc->dtc_v + t3;
////      p_foc->dtc_u    = p_foc->dtc_w + t4;
//      break;
//      case SECTOR_4:// sector 4-5
//      // Vector on-times
//          t0 = -1.0f * p_foc->ubeta * 2.0f / FM_SQRT3;////u2 * FM_SQRT3;
//          t1 = (-1.0f * p_foc->ualpha) + (p_foc->ubeta / FM_SQRT3);// * 2.0 / FM_SQRT3;
//          t2 = 0.0f;
////      t5              = (int32_t) (-2.0 * tmp) * PWM_TIMEBASE_CNT;
////      t4              = (int32_t) (-p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
////      t0              = PWM_TIMEBASE_CNT - t4 - t5;
////      // PWM timings
////      p_foc->dtc_w    = t0 / 2;
////      p_foc->dtc_v    = p_foc->dtc_w + t5;
////      p_foc->dtc_u    = p_foc->dtc_v + t4;
//      break;
//      case SECTOR_5:// sector 5-6
//      // Vector on-times
//          t0 = (-1.0f * p_foc->ualpha) - (p_foc->ubeta / FM_SQRT3)////u2 * FM_SQRT3;
//          t1 = p_foc->ualpha - (p_foc->ubeta / FM_SQRT3);// * 2.0 / FM_SQRT3;
//          t2 = 0.0f;
////      t6              = (int32_t) (p_foc->ualpha - tmp) * PWM_TIMEBASE_CNT;
////      t5              = (int32_t) (-p_foc->ualpha - tmp) * PWM_TIMEBASE_CNT;
////      t0              = PWM_TIMEBASE_CNT - t5 - t6;
////      // PWM timings
////      p_foc->dtc_w    = t0 / 2;
////      p_foc->dtc_u    = p_foc->dtc_w + t5;
////      p_foc->dtc_v    = p_foc->dtc_u + t6;
//      break;
//      case SECTOR_6:// sector 6-1
//      // Vector on-times
//      t6              = (int32_t) (-2.0 * tmp) * PWM_TIMEBASE_CNT;
//      t1              = (int32_t) (p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
//      t0              = PWM_TIMEBASE_CNT - t6 - t1;
//      // PWM timings
//      p_foc->dtc_u    = t0 / 2;
//      p_foc->dtc_w    = p_foc->dtc_u + t1;
//      p_foc->dtc_v    = p_foc->dtc_w + t6;
//      break;
//    }
//
//
//
//
//  if (p_foc->ubeta >= 0.0)
//      if (p_foc->ualpha >= 0.0)
//          sector = (tmp > p_foc->ualpha) ? (SECTOR_2) : (SECTOR_1); //quadrant I
//      else
//          sector = (-tmp > p_foc->ualpha) ? (SECTOR_3) : (SECTOR_2);//quadrant II
//  else if (p_foc->ualpha >= 0.0)
//      sector = (-tmp > p_foc->ualpha) ? (SECTOR_5) : (SECTOR_6);//quadrant IV
//  else
//      sector = (tmp > p_foc->ualpha) ? (SECTOR_4) : (SECTOR_5);//quadrant III
//
////  // PWM timings
////  switch (svm_test.all)
////  {
////    case SECTOR_1 : // sector 1-2
////    // Vector on-times
////    t2              = (int32_t) (2.0 * tmp) * PWM_TIMEBASE_CNT;
////    t1              = (int32_t) (p_foc->ualpha - tmp) * PWM_TIMEBASE_CNT;
////    t0              = PWM_TIMEBASE_CNT - t1 - t2;
////    // PWM timings
////    p_foc->dtc_u    = t0 / 2;
////    p_foc->dtc_v    = p_foc->dtc_u + t1;
////    p_foc->dtc_w    = p_foc->dtc_v + t2;
////    break;
////    case SECTOR_2:// sector 2-3
////    // Vector on-times
////    t3              = (int32_t) (-p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
////    t2              = (int32_t) (p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
////    t0              = PWM_TIMEBASE_CNT - t2 - t3;
////    // PWM timings
////    p_foc->dtc_v    = t0 / 2;
////    p_foc->dtc_u    = p_foc->dtc_v + t3;
////    p_foc->dtc_w    = p_foc->dtc_u + t2;
////    break;
////    case SECTOR_3:// sector 3-4
////    // Vector on-times
////    t4              = (int32_t) (-p_foc->ualpha - tmp) * PWM_TIMEBASE_CNT;
////    t3              = (int32_t) (2.0 * tmp) * PWM_TIMEBASE_CNT;
////    t0              = PWM_TIMEBASE_CNT - t3 - t4;
////    // PWM timings
////    p_foc->dtc_v    = t0 / 2;
////    p_foc->dtc_w    = p_foc->dtc_v + t3;
////    p_foc->dtc_u    = p_foc->dtc_w + t4;
////    break;
////    case SECTOR_4:// sector 4-5
////    // Vector on-times
////    t5              = (int32_t) (-2.0 * tmp) * PWM_TIMEBASE_CNT;
////    t4              = (int32_t) (-p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
////    t0              = PWM_TIMEBASE_CNT - t4 - t5;
////    // PWM timings
////    p_foc->dtc_w    = t0 / 2;
////    p_foc->dtc_v    = p_foc->dtc_w + t5;
////    p_foc->dtc_u    = p_foc->dtc_v + t4;
////    break;
////    case SECTOR_5:// sector 5-6
////    // Vector on-times
////    t6              = (int32_t) (p_foc->ualpha - tmp) * PWM_TIMEBASE_CNT;
////    t5              = (int32_t) (-p_foc->ualpha - tmp) * PWM_TIMEBASE_CNT;
////    t0              = PWM_TIMEBASE_CNT - t5 - t6;
////    // PWM timings
////    p_foc->dtc_w    = t0 / 2;
////    p_foc->dtc_u    = p_foc->dtc_w + t5;
////    p_foc->dtc_v    = p_foc->dtc_u + t6;
////    break;
////    case SECTOR_6:// sector 6-1
////    // Vector on-times
////    t6              = (int32_t) (-2.0 * tmp) * PWM_TIMEBASE_CNT;
////    t1              = (int32_t) (p_foc->ualpha + tmp) * PWM_TIMEBASE_CNT;
////    t0              = PWM_TIMEBASE_CNT - t6 - t1;
////    // PWM timings
////    p_foc->dtc_u    = t0 / 2;
////    p_foc->dtc_w    = p_foc->dtc_u + t1;
////    p_foc->dtc_v    = p_foc->dtc_w + t6;
////    break;
////  }
//}
//#else
//  /* Determine the space vector sector position :
//   * see paper : A New Sector Positioning Method of the SVPWM Reference
//   *  Voltage Space Vector - Gaoming He & al
//   * ---------------------------------------------------------------------
//   * Tests result (sectors)                          - True    - False   -
//   * ---------------------------------------------------------------------
//   * va = Ubeta >= 0.0                               - 1, 2, 3 - 4, 5, 6 -
//   * vb = (sqrt(3)/2 x valpha) - 1/2 x vbeta >= 0.0  - 1, 5    - 2, 4    -
//   * vc = -(sqrt(3)/2 x valpha) - 1/2 x vbeta >= 0.0 - 3, 5    - 2, 6    -
//   * ---------------------------------------------------------------------
//   */
////#if 1
////
////    svm_test_reg_u svm_test;
////    svm_test.bit.UBETA_SIGN             = (p_foc->ubeta > 0.0f)                                         ? (1) : (0);
////    svm_test.bit.UALPHA_UBETA_SIGN      = (((FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta) > 0.0f)          ? (1) : (0);
////    svm_test.bit.NEG_UALPHA_UBETA_SIGN  = (((-1.0f * FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta) > 0.0f)  ? (1) : (0);
////    svm_test.bit.SVM_TEST_RSV1          = 0;
////
//    svm_test_reg_u svm_test;
//    svm_test.bit.UBETA_SIGN_TEST            = (p_foc->ubeta > 0.0f)                                         ? (1) : (0);
//    svm_test.bit.UALPHA_UBETA_SIGN_TEST     = (((FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta) > 0.0f)          ? (1) : (0);
//    svm_test.bit.NEG_UALPHA_UBETA_SIGN_TEST = (((-1.0f * FM_SQRT3 * p_foc->ualpha) - p_foc->ubeta) > 0.0f)  ? (1) : (0);
//    svm_test.bit.SVM_TEST_RSV1              = 0;
////
////  sector = ((p_foc->ua >= 0.0) ? 4 : 0) + ((p_foc->ub >= 0.0) ? 2 : 0) + ((p_foc->uc >= 0.0) ? 1 : 0);
//////  sector = 4 * (p_foc->va >= 0.0) + 2 * (p_foc->vb >= 0.0) + (p_foc->vc >= 0.0);
////#elif 0
////  sector = (uint8_t)ceil(p_foc->theta / (M_PI / 3.0)); // PI/3 expressed in counts
////#else
////  float32_t tmp = ONE_DIV_SQRT3 * p_foc->ubeta;
////  if (p_foc->ubeta >= 0.0)
////  if (p_foc->ualpha >= 0.0)
////  sector = (tmp > p_foc->ualpha) ? (2) : (1); //quadrant I
////  else
////  sector = (-tmp > p_foc->ualpha) ? (3) : (2);//quadrant II
////  else if (p_foc->ualpha >= 0.0)
////  sector = (-tmp > p_foc->ualpha) ? (5) : (6);//quadrant IV
////  else
////  sector = (tmp > p_foc->ualpha) ? (4) : (5);//quadrant III
////#endif
//  /* Check sector value */
//////  if((sector < SECTOR_1) || (sector > SECTOR_6))
//////    return(FOC_ERROR);
//
//    /* Determine t1 & t2 according space vector sector
//     * see paper : Analysis and Realization of a Pulsewidth Modulator
//     * based on Voltage Space Vectors - HW van der Broeck
//     * A = sqrt(3) x Tz x abs(Vref) / Vdc
//     * t1 = A x sin(PI/3 - phi + ((n-1)/3 x PI))
//     * t2 = A x sin(phi - ((n-1)/3 x PI))
//     * using :
//     * Valpha = A x cos(phi) & Vbeta = A x sin(phi)
//     * then :
//     * t1 = A x (Valpha x sin((n x PI)/3) - Vbeta x cos((n x PI)/3))
//     * t2 = A x (-Valpha x sin(((n-1) x PI)/3) - Vbeta x cos(((n-1) x PI)/3))
//     */
////  float32_t t2, t1;
////  motor_svm_sector_e sector = SECTOR_0;
////
////  switch (sector) {
////    case SECTOR_1: // sector 1-2
////      t1 = p_foc->ub * 0.5f * PWM_PERIOD;
////      t2 = p_foc->ua * 0.5f * PWM_PERIOD;
////      break;
////    case SECTOR_2: // sector 2-3
////      t1 = -p_foc->uc * 0.5f * PWM_PERIOD;
////      t2 = -p_foc->ub * 0.5f * PWM_PERIOD;
////      break;
////    case SECTOR_3:  // sector 3-4
////      t1 = p_foc->ua * 0.5f * PWM_PERIOD;
////      t2 = p_foc->uc * 0.5f * PWM_PERIOD;
////      break;
////    case SECTOR_4:  // sector 4-5
////      t1 = -p_foc->ub * 0.5f * PWM_PERIOD;
////      t2 = -p_foc->ua * 0.5f * PWM_PERIOD;
////      break;
////    case SECTOR_5:  // sector 5-6
////      t1 = p_foc->uc * 0.5f * PWM_PERIOD;
////      t2 = p_foc->ub * 0.5f * PWM_PERIOD;
////      break;
////    case SECTOR_6:  // sector 6-1
////      t1 = -p_foc->ua * 0.5f * PWM_PERIOD;
////      t2 = -p_foc->uc * 0.5f * PWM_PERIOD;
////      break;
////    default:
////      t1 = 0.0;
////      t2 = 0.0;
////  }
//
//  /* Calculate Periods :
//   * PWM Timer using PWM mode 2 (see ImpactVSA_bsp.c):
//   *  upcounting, channel 1 is inactive when 0 < TIMx_CNT < TIMx_CCR1 else active.
//   *  downcounting, channel 1 is active when TIMx_ARR > TIMx_CNT > TIMx_CCR1 else inactive.
//   * T1 (shortest time) < T2 < T3 => But T1 longest ON active transistors.
//   * T1 = 1/2 sector 0(---) + 1/2 sector 7 (+++) - t1 - t2,
//   * T2 = 1/2 sector 0(---) + 1/2 sector 7 (+++) + t1,
//   * T3 = 1/2 sector 0(---) + 1/2 sector 7 (+++) + t1 + t2
//   */
////  uint32_t T1 = (uint32_t) ((0.5f * PWM_PERIOD) - t1 - t2) / 2;
////  uint32_t T2 = (uint32_t) (T1 + t1);
////  uint32_t T3 = (uint32_t) (T2 + t2);
//
//  /* PWM timings
//   * sector 1 : V1(+--) - V2(++-)
//   * sector 2 : V2(++-) - V3(-+-)
//   * sector 3 : V3(-+-) - V4(-++)
//   * sector 4 : V4(-++) - V5(--+)
//   * sector 5 : V5(--+) - V6(+-+)
//   * sector 6 : V6(+-+) - V1(+--)
//   */
////  switch (sector) {
////    case SECTOR_1: // sector 1 : V1(+--) - V2(++-)
////        *(p_motor->p_motorChAReg) = (uint16_t)T1;
////        *(p_motor->p_motorChBReg) = (uint16_t)T2;
////        *(p_motor->p_motorChCReg) = (uint16_t)T3;
////      break;
////    case SECTOR_2: // sector 2 : V2(++-) - V3(-+-)
////        *(p_motor->p_motorChAReg) = (uint16_t)T2;
////         *(p_motor->p_motorChBReg) = (uint16_t)T1;
////         *(p_motor->p_motorChCReg) = (uint16_t)T3;
////      break;
////    case SECTOR_3: // sector 3 : V3(-+-) - V4(-++)
////        *(p_motor->p_motorChAReg) = (uint16_t)T3;
////         *(p_motor->p_motorChBReg) = (uint16_t)T1;
////         *(p_motor->p_motorChCReg) = (uint16_t)T2;
////      break;
////    case SECTOR_4: // sector 4 : V4(-++) - V5(--+)
////        *(p_motor->p_motorChAReg) = (uint16_t)T3;
////         *(p_motor->p_motorChBReg) = (uint16_t)T2;
////         *(p_motor->p_motorChCReg) = (uint16_t)T1;
////      break;
////    case SECTOR_5: // sector 5 : V5(--+) - V6(+-+)
////        *(p_motor->p_motorChAReg) = (uint16_t)T2;
////         *(p_motor->p_motorChBReg) = (uint16_t)T3;
////         *(p_motor->p_motorChCReg) = (uint16_t)T1;
////      break;
////    case SECTOR_6: // sector 6 : V6(+-+) - V1(+--)
////        *(p_motor->p_motorChAReg) = (uint16_t)T1;
////         *(p_motor->p_motorChBReg) = (uint16_t)T3;
////         *(p_motor->p_motorChCReg) = (uint16_t)T2;
////      break;
////    default:
////        *(p_motor->p_motorChAReg) = (uint16_t)T1;
////         *(p_motor->p_motorChBReg) = (uint16_t)T2;
////         *(p_motor->p_motorChCReg) = (uint16_t)T3;
////  }
////  return;
//
//
//}
//#endif


//     static inline void SVGEN_calc(SVGEN* v)
//     {
//         v->tmp1= v->Ubeta;
//         v->tmp2= _IQdiv2(v->Ubeta) + (_IQmpy(_IQ(0.866),v->Ualpha));
//         v->tmp3= v->tmp2 - v->tmp1;
//
//         v->VecSector=3;
//         v->VecSector=(v->tmp2> 0)?( v->VecSector-1):v->VecSector;
//         v->VecSector=(v->tmp3> 0)?( v->VecSector-1):v->VecSector;
//         v->VecSector=(v->tmp1< 0)?(7-v->VecSector) :v->VecSector;
//
//         if(v->VecSector==1 || v->VecSector==4)
//         {
//             v->Ta= v->tmp2;
//             v->Tb= v->tmp1-v->tmp3;
//             v->Tc=-v->tmp2;
//         }
//         else if(v->VecSector==2 || v->VecSector==5)
//         {
//             v->Ta= v->tmp3+v->tmp2;
//             v->Tb= v->tmp1;
//             v->Tc=-v->tmp1;
//         }
//         else
//         {
//             v->Ta= v->tmp3;
//             v->Tb=-v->tmp3;
//             v->Tc=-(v->tmp1+v->tmp2);
//         }
//     }
