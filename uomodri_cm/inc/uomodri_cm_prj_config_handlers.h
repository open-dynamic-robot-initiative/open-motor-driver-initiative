#ifndef __UOMODRI_CM_PRJ_CONFIG_HANDLERS_H__
#define __UOMODRI_CM_PRJ_CONFIG_HANDLERS_H__

// drivers
#include "driverlib_cm.h"
#include "cm.h"

#include "hal_cm.h"
#include "communication_cm.h"
#include "uomodri_cm_user_defines.h"

//https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_µDriver_SPI_interface.md
//https://github.com/open-dynamic-robot-initiative/master-board/blob/master/firmware/main/defines.h

/** @var    command_msg uomodri_cmd
 *  @brief  List all configuration registers for each DRVs.
 */
//#pragma DATA_SECTION(com_rx_cmd, "MSGRAM_CM_TO_CPU1");
//uomodri_cmd uomodri_cmd[] =
//{
// {
//  .positionRef                  = 0.0,
//  .velocityRef                  = 0.0,
//  .iqRef                        = 0.0,
//  .kpCoeff                      = 1.0,
//  .kdCoeff                      = 1.0,
//  .iSat                         = 4.0,
//  .encoderOffsetCompEnable      = false,
//  .rolloverPositionErrorEnable  = false,
//  .motorEnable                  = false;
// },
// {
//  .positionRef                  = 0.0,
//  .velocityRef                  = 0.0,
//  .iqRef                        = 0.0,
//  .kpCoeff                      = 1.0,
//  .kdCoeff                      = 1.0,
//  .iSat                         = 4.0,
//  .encoderOffsetCompEnable      = false,
//  .rolloverPositionErrorEnable  = false,
//  .motorEnable                  = false;
// }
//};
extern rx_uomodri_t __attribute__((section (".ComRxCmdSection"))) com_rx_cmd =
{
 .modeReg.bit.TIMEOUT   = 0xFFFF,
 .modeReg.bit.MODE_RSV1 = 0,
 .modeReg.bit.EI2OC     = false,
 .modeReg.bit.EI1OC     = false,
 .modeReg.bit.EPRE      = false,
 .modeReg.bit.EM2       = false,
 .modeReg.bit.EM1       = false,
 .modeReg.bit.ES        = false,
 .positionRef[MOTOR_1]  = 0.0,
 .positionRef[MOTOR_2]  = 0.0,
 .velocityRef[MOTOR_1]  = 0.0,
 .velocityRef[MOTOR_2]  = 0.0,
 .iqRef[MOTOR_1]        = 0.0,
 .iqRef[MOTOR_2]        = 0.0,
 .kpCoeff[MOTOR_1]      = 1.0,
 .kpCoeff[MOTOR_2]      = 1.0,
 .kdCoeff[MOTOR_1]      = 1.0,
 .kdCoeff[MOTOR_2]      = 1.0,
 .iSat[MOTOR_1]         = 4.0,
 .iSat[MOTOR_1]         = 4.0,
};

//#pragma DATA_SECTION(com_tx_msg, "MSGRAM_CM_TO_CPU1");
extern tx_msg_t __attribute__((section (".ComTxMsgSection"))) com_tx_msg =
{
 .payload.statusReg.bit.ERROR_CODE  = 0x00,
 .payload.statusReg.bit.STATUS_RSV  = 0,
 .payload.statusReg.bit.IDX2T       = false,
 .payload.statusReg.bit.IDX1T       = false,
 .payload.statusReg.bit.IDX2D       = false,
 .payload.statusReg.bit.IDX1D       = false,
 .payload.statusReg.bit.M2R         = false,
 .payload.statusReg.bit.M2E         = false,
 .payload.statusReg.bit.M1R         = false,
 .payload.statusReg.bit.M1E         = false,
 .payload.statusReg.bit.SE          = false,
 .payload.timeStamp                 = 0,
 .payload.position[MOTOR_1]         = 0,
 .payload.position[MOTOR_2]         = 0,
 .payload.velocity[MOTOR_1]         = 0,
 .payload.velocity[MOTOR_2]         = 0,
 .payload.iq[MOTOR_1]               = 0,
 .payload.iq[MOTOR_2]               = 0,
 .payload.coilRes[MOTOR_1]          = 0,
 .payload.coilRes[MOTOR_2]          = 0,
 .payload.adcSamples[MOTOR_1]       = 0,
 .payload.adcSamples[MOTOR_2]       = 0,
 .payload.lastCmdIndex              = 0,
 .crc                               = 0,
 .time[MOTOR_1]                     = 0,
 .time[MOTOR_2]                     = 0,
};

#endif  /*__UOMODRI_CM_PRJ_CONFIG_HANDLERS_H__*/
