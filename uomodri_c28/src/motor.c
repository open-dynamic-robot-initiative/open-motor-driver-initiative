/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "motor.h"
#include "encoder.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
void MOT_initStruct(motor_t* p_motor, uint16_t motor_id)
{
    if(motor_id == MOTOR_1)
    {
        //--- MOTOR_1 -----------------------------------------------------------
        p_motor->id                                     = MOTOR_1;
        //--- FOC structure -----------------------------------------------------
        //--- MOTOR PARAMS STRUCTURE --------------------------------------------
        p_motor->foc.cfg.Rs                             = MOTOR1_RS;
        p_motor->foc.cfg.Ls                             = MOTOR1_LS;
        p_motor->foc.cfg.kv                             = MOTOR1_KV;
        p_motor->foc.cfg.ke                             = MOTOR1_KE;
        p_motor->foc.cfg.ki                             = MOTOR1_KI;
        p_motor->foc.cfg.polePairs                      = MOTOR1_POLES_PAIRS;
        //--- MOTOR ACQUISITION STRUCTURE ---------------------------------------
        p_motor->foc.acq.iabcResultBase[CHANNEL_1]      = MOTOR1_IA_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_2]      = MOTOR1_IB_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_3]      = MOTOR1_IC_ADC_RESULT_ADDR;
        p_motor->foc.acq.vExtResultBase                 = VEXT1_ADC_RESULT_ADDR;
        p_motor->foc.acq.ppbNum.ia                      = MOTOR1_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ib                      = MOTOR1_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ic                      = MOTOR1_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.vext                    = VEXT1_PPB_NUM;
        //--- MOTOR ENCORDER STRUCTURE ------------------------------------------
        p_motor->foc.enc.eqepBase                       = MOTOR1_QEP_BASE;
        p_motor->foc.enc.polePairs                      = MOTOR1_POLES_PAIRS;
        p_motor->foc.enc.thetaMechScaler                = (float32_t)MOTOR1_ENC_RESOLUTION_SCALE;
        p_motor->foc.enc.speed.speedHighScaler          = (float32_t)MOTOR1_ENC_SPEED_HIGH_SCALE;
        p_motor->foc.enc.speed.speedLowScaler           = (float32_t)MOTOR1_ENC_SPEED_LOW_SCALE;
        p_motor->foc.enc.speed.speedFlt[0].a            = (float32_t)MOTOR1_ENC_SPEED_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[0].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LPF_ONE_M_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].a            = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].one_minus_a  = (float32_t)MOTOR1_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA;
        //--- MOTOR COMMAND STRUCTURE -------------------------------------------
        p_motor->foc.cmd.iSat                           = MOTOR1_CURRENT_CMD_SAT_MAX;
        //-----------------------------------------------------------------------
        p_motor->foc.iParkFlt.a                         = MOTOR1_CURRENT_LPF_ALPHA;
        p_motor->foc.iParkFlt.one_minus_a               = MOTOR1_CURRENT_LPF_ONE_M_ALPHA;
        //--- FOC - Initialize duty cycle saturation ----------------------------
        p_motor->foc.dtcMax                             = MOTOR1_DTC_MAX;
        p_motor->foc.dtcMin                             = MOTOR1_DTC_MIN;
        //--- FOC - Encoder test ------------------------------------------------
        p_motor->foc.initPosCnt                         = MOTOR1_POSITION_INIT_EVT;
        p_motor->foc.initFixCnt                         = MOTOR1_FIX_INIT_EVT;
        p_motor->foc.initErr[MIN]                       = MOTOR1_POSITION_INIT_ERR_MIN;
        p_motor->foc.initErr[MAX]                       = MOTOR1_POSITION_INIT_ERR_MAX;
        //--- FOC - Initialize PI controller (Id) -------------------------------
        p_motor->foc.piId.kp                            = (float32_t)MOTOR1_PI_ID_KP_COEF;
        p_motor->foc.piId.ki                            = (float32_t)MOTOR1_PI_ID_KI_COEF;
        //--- FOC - Initialize PI controller (Iq) -------------------------------
        p_motor->foc.piIq.kp                            = (float32_t)MOTOR1_PI_IQ_KP_COEF;
        p_motor->foc.piIq.ki                            = (float32_t)MOTOR1_PI_IQ_KI_COEF;
        //--- DRV IO Config -----------------------------------------------------
        p_motor->drvFaultNum                            = MOTOR1_DRV_FAULT_NUM;
        p_motor->drvEnNum                               = MOTOR1_DRV_EN_NUM;
        //---- PWM config address -----------------------------------------------
        p_motor->epwmBase[CHANNEL_1]                    = MOTOR1_PWM1_BASE;
        p_motor->epwmBase[CHANNEL_2]                    = MOTOR1_PWM2_BASE;
        p_motor->epwmBase[CHANNEL_3]                    = MOTOR1_PWM3_BASE;
    }
    else
    {
        //--- MOTOR_2 -----------------------------------------------------------
        p_motor->id                                     = MOTOR_2;
        //--- FOC structure -----------------------------------------------------
        //--- MOTOR PARAMS STRUCTURE --------------------------------------------
        p_motor->foc.cfg.Rs                             = MOTOR2_RS;
        p_motor->foc.cfg.Ls                             = MOTOR2_LS;
        p_motor->foc.cfg.kv                             = MOTOR2_KV;
        p_motor->foc.cfg.ke                             = MOTOR2_KE;
        p_motor->foc.cfg.ki                             = MOTOR2_KI;
        p_motor->foc.cfg.polePairs                      = MOTOR2_POLES_PAIRS;
        //--- MOTOR ACQUISITION STRUCTURE ---------------------------------------
        p_motor->foc.acq.iabcResultBase[CHANNEL_1]      = MOTOR2_IA_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_2]      = MOTOR2_IB_ADC_RESULT_ADDR;
        p_motor->foc.acq.iabcResultBase[CHANNEL_3]      = MOTOR2_IC_ADC_RESULT_ADDR;
        p_motor->foc.acq.vExtResultBase                 = VEXT2_ADC_RESULT_ADDR;
        p_motor->foc.acq.ppbNum.ia                      = MOTOR2_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ib                      = MOTOR2_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.ic                      = MOTOR2_IABC_PPB_NUM;
        p_motor->foc.acq.ppbNum.vext                    = VEXT2_PPB_NUM;
        //--- MOTOR ENCORDER STRUCTURE ------------------------------------------
        p_motor->foc.enc.eqepBase                       = MOTOR2_QEP_BASE;
        p_motor->foc.enc.polePairs                      = MOTOR2_POLES_PAIRS;
        p_motor->foc.enc.thetaMechScaler                = (float32_t)MOTOR2_ENC_RESOLUTION_SCALE;
        p_motor->foc.enc.speed.speedHighScaler          = (float32_t)MOTOR2_ENC_SPEED_HIGH_SCALE;
        p_motor->foc.enc.speed.speedLowScaler           = (float32_t)MOTOR2_ENC_SPEED_LOW_SCALE;
        p_motor->foc.enc.speed.speedFlt[0].a            = (float32_t)MOTOR2_ENC_SPEED_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[0].one_minus_a  = (float32_t)MOTOR2_ENC_SPEED_LPF_ONE_M_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].a            = (float32_t)MOTOR2_ENC_SPEED_LOCAL_LPF_ALPHA;
        p_motor->foc.enc.speed.speedFlt[1].one_minus_a  = (float32_t)MOTOR2_ENC_SPEED_LOCAL_LPF_ONE_M_ALPHA;
        //--- MOTOR COMMAND STRUCTURE -------------------------------------------
        p_motor->foc.cmd.iSat                           = MOTOR2_CURRENT_CMD_SAT_MAX;
        //-----------------------------------------------------------------------
        p_motor->foc.iParkFlt.a                         = MOTOR2_CURRENT_LPF_ALPHA;
        p_motor->foc.iParkFlt.one_minus_a               = MOTOR2_CURRENT_LPF_ONE_M_ALPHA;
        //--- FOC - Initialize duty cycle saturation ----------------------------
        p_motor->foc.dtcMax                             = MOTOR2_DTC_MAX;
        p_motor->foc.dtcMin                             = MOTOR2_DTC_MIN;
        //--- FOC - Encoder test ------------------------------------------------
        p_motor->foc.initPosCnt                         = MOTOR2_POSITION_INIT_EVT;
        p_motor->foc.initFixCnt                         = MOTOR2_FIX_INIT_EVT;
        p_motor->foc.initErr[MIN]                       = MOTOR2_POSITION_INIT_ERR_MIN;
        p_motor->foc.initErr[MAX]                       = MOTOR2_POSITION_INIT_ERR_MAX;
        //--- FOC - Initialize PI controller (Id) -------------------------------
        p_motor->foc.piId.kp                            = (float32_t)MOTOR2_PI_ID_KP_COEF;
        p_motor->foc.piId.ki                            = (float32_t)MOTOR2_PI_ID_KI_COEF;
        //--- FOC - Initialize PI controller (Iq) -------------------------------
        p_motor->foc.piIq.kp                            = (float32_t)MOTOR2_PI_IQ_KP_COEF;
        p_motor->foc.piIq.ki                            = (float32_t)MOTOR2_PI_IQ_KI_COEF;
        //--- DRV IO Config -----------------------------------------------------
        p_motor->drvFaultNum                            = MOTOR2_DRV_FAULT_NUM;
        p_motor->drvEnNum                               = MOTOR2_DRV_EN_NUM;
        //---- PWM config address -----------------------------------------------
        p_motor->epwmBase[CHANNEL_1]                    = MOTOR2_PWM1_BASE;
        p_motor->epwmBase[CHANNEL_2]                    = MOTOR2_PWM2_BASE;
        p_motor->epwmBase[CHANNEL_3]                    = MOTOR2_PWM3_BASE;
    }
    //--- MOTOR ACQUISITION STRUCTURE -------------------------------------------
    p_motor->foc.acq.vBusResultBase                 = VBUS_ADC_RESULT_ADDR;
    p_motor->foc.acq.ppbNum.vbus                    = VBUS_PPB_NUM;
    p_motor->foc.acq.vBusFlt.a                      = VBUS_LPF_ALPHA;
    p_motor->foc.acq.vBusFlt.one_minus_a            = VBUS_LPF_ONE_M_ALPHA;
    p_motor->foc.acq.vExtFlt.a                      = VEXT_LPF_ALPHA;
    p_motor->foc.acq.vExtFlt.one_minus_a            = VEXT_LPF_ONE_M_ALPHA;
    //--- MOTOR ENCORDER STRUCTURE ----------------------------------------------
    p_motor->foc.enc.thetaMech[NEW]                 = 0.0f;
    p_motor->foc.enc.thetaMech[OLD]                 = 0.0f;
    p_motor->foc.enc.thetaIndexRelative             = 0.0f;
    p_motor->foc.enc.thetaIndexAbsolute             = 0.0f;
    p_motor->foc.enc.turnNb                         = 0;
    p_motor->foc.enc.flags.all                      = 0U;
    p_motor->foc.enc.speed.speedMech[0]             = 0.0f;
    p_motor->foc.enc.speed.speedMech[1]             = 0.0f;
    p_motor->foc.enc.speed.theta[NEW]               = 0.0f;
    p_motor->foc.enc.speed.theta[OLD]               = 0.0f;
    //--- MOTOR COMMAND STRUCTURE -----------------------------------------------
    p_motor->foc.cmd.posRef                         = 0.0f;
    p_motor->foc.cmd.velRef                         = 0.0f;
    p_motor->foc.cmd.iqff                           = 0.0f;
    p_motor->foc.cmd.kpCoeff                        = 0.0f;
    p_motor->foc.cmd.kdCoeff                        = 0.0f;
    p_motor->foc.cmd.timeoutRef                     = 0U;
    p_motor->foc.cmd.cptTimeout                     = 0U;
    p_motor->foc.cmd.index                          = 0U;
    p_motor->foc.cmd.cmdBits.all                    = 0U;
    //--- FOC - Park transform results ------------------------------------------
    p_motor->foc.id                                 = 0.0f;
    p_motor->foc.iq                                 = 0.0f;
    //--- FOC - Encoder test ----------------------------------------------------
    p_motor->foc.initPosStep                        = 0.0f;
    p_motor->foc.initPosEnc[FW]                     = 0.0f;
    p_motor->foc.initPosEnc[REV]                    = 0.0f;
    //--- FOC - Initialize PI controller (Id) -----------------------------------
    p_motor->foc.piId.p_fb.ptr                      = (float32_t *)(&p_motor->foc.id);
    p_motor->foc.piId.p_set.ptr                     = (float32_t *)(&p_motor->foc.idRef);
    p_motor->foc.piId.p_sat.ptr                     = (float32_t *)(&p_motor->foc.acq.vbus);
    p_motor->foc.piId.err                           = 0.0f;
    p_motor->foc.piId.integral                      = 0.0f;
    p_motor->foc.piId.ff                            = 0.0f;
    p_motor->foc.piId.out                           = 0.0f;
    //--- FOC - Initialize PI controller (Iq) -----------------------------------
    p_motor->foc.piIq.p_fb.ptr                      = (float32_t *)(&p_motor->foc.iq);
    p_motor->foc.piIq.p_set.ptr                     = (float32_t *)(&p_motor->foc.iqRef);
    p_motor->foc.piIq.p_sat.ptr                     = (float32_t *)(&p_motor->foc.acq.vbus);
    p_motor->foc.piIq.err                           = 0.0f;
    p_motor->foc.piIq.integral                      = 0.0f;
    p_motor->foc.piIq.ff                            = 0.0f;
    p_motor->foc.piIq.out                           = 0.0f;
    //--- FOC - Initialize PD controller (Speed & position) ---------------------
    p_motor->foc.pdPosVel.p_fbTheta.ptr             = (float32_t *)(&p_motor->foc.enc.thetaAbsolute);
    p_motor->foc.pdPosVel.p_setTheta.ptr            = (float32_t *)(&p_motor->foc.cmd.posRef);
    p_motor->foc.pdPosVel.errTheta                  = 0.0f;
    p_motor->foc.pdPosVel.p_fbSpeed.ptr             = (float32_t *)(&p_motor->foc.enc.speed.speedMech[0]);
    p_motor->foc.pdPosVel.p_setSpeed.ptr            = (float32_t *)(&p_motor->foc.cmd.velRef);
    p_motor->foc.pdPosVel.errSpeed                  = 0.0f;
    p_motor->foc.pdPosVel.p_kp.ptr                  = (float32_t *)(&p_motor->foc.cmd.kpCoeff);
    p_motor->foc.pdPosVel.p_kd.ptr                  = (float32_t *)(&p_motor->foc.cmd.kdCoeff);
    p_motor->foc.pdPosVel.derivative                = 0.0f;
    p_motor->foc.pdPosVel.p_ff.ptr                  = (float32_t *)(&p_motor->foc.cmd.iqff);
    p_motor->foc.pdPosVel.p_sat.ptr                 = (float32_t *)(&p_motor->foc.cmd.iSat);
    p_motor->foc.pdPosVel.out                       = 0.0f;
    //--- FSM motor initial state -----------------------------------------------
    p_motor->state                                  = MOTOR_STATE_IDLE;
    //--- Error message ---------------------------------------------------------
    p_motor->error.all                              = 0U;
    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------
}
