#ifndef __UOMODRI_PRJ_CONFIG_HANDLERS_H__
#define __UOMODRI_PRJ_CONFIG_HANDLERS_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
//#include "f2838x_device.h"
//#include "f2838x_examples.h"
#include <stdint.h>
#include <driverlib.h>

#include "hal.h"
#include "drv8353.h"
#include "foc.h"
#include "motor.h"
#include "communication.h"
#include "config/uomodri_3_1/uomodri_3_1_handlers.h"
#include "uomodri_user_defines.h"
//#include "uomodri_hal_config_handlers.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void cla_ClearTask(void);
__attribute__((interrupt)) void cla_isrMotor1(void);
__attribute__((interrupt)) void cla_isrMotor2(void);
__attribute__((interrupt)) void tmr_isrLedCmd(void);
__attribute__((interrupt)) void dma_isrSpiComRx(void);
__attribute__((interrupt)) void sci_isrRx_RS485(void);
__attribute__((interrupt)) void sci_isrTx_RS485(void);
__attribute__((interrupt)) void clb_isrTxRx_RS485(void);

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   InterruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t InterruptList[] =
{
 {
  .intNum           = INT_CLA_ADC_CALIB,
  .intAckGroup      = INT_CLA_ADC_CALIB_GROUP,
  .p_intHandler     = INT_CLA_ADC_HANDLER_HANDLER
 },
 {
  .intNum           = INT_CLA_MOTOR1,
  .intAckGroup      = INT_CLA_MOTOR1_GROUP,
  .p_intHandler     = INT_CLA_MOTOR1_HANDLER
 },
 {
  .intNum           = INT_CLA_MOTOR2,
  .intAckGroup      = INT_CLA_MOTOR2_GROUP,
  .p_intHandler     = INT_CLA_MOTOR2_HANDLER
 },
 {
  .intNum           = INT_TIMER_LED,
  .intAckGroup      = INT_TIMER_LED_CMD_GROUP,
  .p_intHandler     = INT_TIMER_LED_CMD_HANDLER
 },
 {
  .intNum           = INT_DMA_RX_MSG,
  .intAckGroup      = INT_DMA_RX_MSG_GROUP,
  .p_intHandler     = INT_DMA_RX_MSG_HANDLER
 },
#if (RS485_BUS_ENABLE) && (!UOMODRI_V1_0_ENABLE)
 {
  .intNum           = INT_RS485_TX_MSG,
  .intAckGroup      = INT_RS485_TX_MSG_GROUP,
  .p_intHandler     = INT_RS485_TX_MSG_HANDLER
 },
 {
  .intNum           = INT_RS485_RX_MSG,
  .intAckGroup      = INT_RS485_RX_MSG_GROUP,
  .p_intHandler     = INT_RS485_RX_MSG_HANDLER
 },
 {
  .intNum           = INT_RS485_Rx_Tx_CLB,
  .intAckGroup      = INT_RS485_RX_Tx_CLB_GROUP,
  .p_intHandler     = INT_RS485_RX_Tx_CLB_HANDLER
 },
#endif
 // END OF ARRAY
 {.intNum           = UINT32_MAX},
};

#endif  /*__UOMODRI_PRJ_CONFIG_HANDLERS_H__*/
