#ifndef __UOMODRI_INTERRUPTS_H__
#define __UOMODRI_INTERRUPTS_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
//#include "/home/jmanhes/ti/c2000/C2000Ware_5_02_00_00/device_support/f2838x/common/include/f2838x_cla_typedefs.h"  // F2838x CLA Type definitions
//#include "/home/jmanhes/ti/c2000/C2000Ware_5_02_00_00/device_support/f2838x/headers/include/f2838x_device.h"        // F2838x Headerfile Include File
//#include "/home/jmanhes/ti/c2000/C2000Ware_5_02_00_00/driverlib/f2838x/driverlib/include/f2838x_device.h"        // F2838x Headerfile Include File
//#include "/home/jmanhes/ti/c2000/C2000Ware_5_02_00_00/device_support/f2838x/common/include/f2838x_examples.h"      // F2838x Examples Include File

//#include "f2838x_cla_typedefs.h"  // F2838x CLA Type definitions
//#include "f2838x_device.h"
//#include "f2838x_examples.h"      // F2838x Examples Include File

#include "uomodri_user_defines.h"   // uOmodri Headerfile Include File
//#include "f2838x_device.h"          // F2838x Headerfile Include File
//#include "device.h"
//#include "driverlib.h"
//#include "device.h"

//#include "../main.h"
//#include "../hal.h"
//#include "../foc.h"
//#include "driverlib.h"
//#include "device.h"
//#include "board.h"
//#include "c2000ware_libraries.h"

//#include "driverlib.h"
//#include "../hal.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
__attribute__((interrupt)) void cla_ClearTask(void);
__attribute__((interrupt)) void cla_isrMotor1(void);
#if (USE_DUAL_AXIS_CONFIG == true)
__attribute__((interrupt)) void cla_isrMotor2(void);
#endif
__attribute__((interrupt)) void tmr_isrLedCmd(void);
__attribute__((interrupt)) void dma_isrSpiComRx(void);
__attribute__((interrupt)) void sci_isrRx_RS485(void);
__attribute__((interrupt)) void sci_isrTx_RS485(void);
__attribute__((interrupt)) void clb_isrTxRx_RS485(void);

/***********************************************************************
 * INTERRUPTS DEFINES
 ***********************************************************************/
#define INT_TIMER_LED                   INT_TIMER0
#define INT_TIMER_LED_CMD_GROUP         INTERRUPT_ACK_GROUP1
#define INT_TIMER_LED_CMD_HANDLER       &tmr_isrLedCmd

#define INT_RS485_Rx_Tx_CLB             INT_CLB1
#define INT_RS485_RX_Tx_CLB_GROUP       INTERRUPT_ACK_GROUP5
#define INT_RS485_RX_Tx_CLB_HANDLER     &clb_isrTxRx_RS485

#define INT_DMA_RX_MSG                  INT_DMA_CH6
#define INT_DMA_RX_MSG_GROUP            INTERRUPT_ACK_GROUP7
#define INT_DMA_RX_MSG_HANDLER          &dma_isrSpiComRx

#define INT_RS485_RX_PIE_FLAG           PIE_IFR8_INTX5
#define INT_RS485_RX_MSG                INT_SCIC_RX
#define INT_RS485_RX_MSG_GROUP          INTERRUPT_ACK_GROUP8
#define INT_RS485_RX_MSG_HANDLER        &sci_isrRx_RS485

#define INT_RS485_TX_PIE_FLAG           PIE_IFR8_INTX6
#define INT_RS485_TX_MSG                INT_SCIC_TX
#define INT_RS485_TX_MSG_GROUP          INTERRUPT_ACK_GROUP8
#define INT_RS485_TX_MSG_HANDLER        &sci_isrTx_RS485

#define INT_CLA_ADC_CALIB               INT_CLA1_1
#define INT_CLA_ADC_CALIB_GROUP         INTERRUPT_ACK_GROUP11
#define INT_CLA_ADC_HANDLER_HANDLER     &cla_ClearTask

#define INT_CLA_MOTOR1                  INT_CLA1_3
#define INT_CLA_MOTOR1_GROUP            INTERRUPT_ACK_GROUP11
#define INT_CLA_MOTOR1_HANDLER          &cla_isrMotor1

#define INT_CLA_MOTOR2                  INT_CLA1_4
#define INT_CLA_MOTOR2_GROUP            INTERRUPT_ACK_GROUP11
#define INT_CLA_MOTOR2_HANDLER          &cla_isrMotor2

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    int_cfg_t   interruptList[]
 *  @brief  List all required interrupts configurations.
 */
static const int_cfg_t interruptList[] =
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
#if (USE_DUAL_AXIS_CONFIG == true)
 {
  .intNum           = INT_CLA_MOTOR2,
  .intAckGroup      = INT_CLA_MOTOR2_GROUP,
  .p_intHandler     = INT_CLA_MOTOR2_HANDLER
 },
#endif
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
#if (RS485_BUS_ENABLE) && (USE_UOMODRI_REV != USE_UOMODRI_V1)
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

#endif  /*__UOMODRI_INTERRUPTS_H__*/
