#ifndef __UOMODRI_HAL_CONFIG_HANDLERS_H__
#define __UOMODRI_HAL_CONFIG_HANDLERS_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
//#include "f2838x_device.h"
//#include "f2838x_examples.h"
#include "driverlib.h"
//#include "device.h"

#include "hal.h"
#include "communication.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * VARIABLE DECLARATION
 ***********************************************************************/
extern mst2slv_msg_t    cmd_msg;
extern slv2mst_msg_t    status_msg;
extern uint16_t         led_msg[LED_MSG_TX_16BIT_LENGTH];

/** @var    GPIO_TypeDef GpioPinsCfgList[]
 *  @brief  List all the GPIO definitions except the analog pins.
 */
static const gpio_cfg_t GpioPinsCfgList[] =
{
#ifdef UOMODRI_V1_ENABLE
#if (defined USB_BUS_ENABLE) && (!defined UART_BUS_ENABLE)
 // USB Pins
 {.pinNumber = USB_M,                   .fctMux = USB_M_CFG,                    .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false, .analogModeEnable = GPIO_ANALOG_ENABLED},
 {.pinNumber = USB_P,                   .fctMux = USB_P_CFG,                    .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false, .analogModeEnable = GPIO_ANALOG_ENABLED},
#elif (!defined USB_BUS_ENABLE) && (defined UART_BUS_ENABLE)
 // UART Pins
 {.pinNumber = UART_ON_USB_TX,          .fctMux = UART_ON_USB_TX_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = UART_ON_USB_RX,          .fctMux = UART_ON_USB_RX_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
#endif
 // GPIO Debug pins
 {.pinNumber = DBG_PIN0,                .fctMux = DBG_PIN0_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = DBG_PIN1,                .fctMux = DBG_PIN1_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = DBG_PIN2,                .fctMux = DBG_PIN2_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = DBG_PIN3,                .fctMux = DBG_PIN3_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = DBG_PIN4,                .fctMux = DBG_PIN4_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
#if (defined CAN_BUS_ENABLE) && (!defined RS485_BUS_ENABLE)
 // CAN BUS IO pins
 {.pinNumber = CAN_RX,                  .fctMux = CAN_RX_CFG,                   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = CAN_TX,                  .fctMux = CAN_TX_CFG,                   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false},
#endif
 // Quadrature encoder #1
 {.pinNumber = MOTOR1_ENC_CHA,          .fctMux = MOTOR1_ENC_CHA_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_ENC_CHB,          .fctMux = MOTOR1_ENC_CHB_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_ENC_CHI,          .fctMux = MOTOR1_ENC_CHI_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 // Quadrature encoder #2
 {.pinNumber = MOTOR2_ENC_CHA,          .fctMux = MOTOR2_ENC_CHA_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_ENC_CHB,          .fctMux = MOTOR2_ENC_CHB_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_ENC_CHI,          .fctMux = MOTOR2_ENC_CHI_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 // DRV communication
 {.pinNumber = DRV_SPI_SIMO,            .fctMux = DRV_SPI_SIMO_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = DRV_SPI_SOMI,            .fctMux = DRV_SPI_SOMI_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false},
 {.pinNumber = DRV_SPI_CLK,             .fctMux = DRV_SPI_CLK_CFG,              .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 // OMODRI communication
 {.pinNumber = COM_SPI_SIMO,            .fctMux = COM_SPI_SIMO_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = COM_SPI_SOMI,            .fctMux = COM_SPI_SOMI_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = COM_SPI_CLK,             .fctMux = COM_SPI_CLK_CFG,              .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = COM_SPI_CS,              .fctMux = COM_SPI_CS_CFG,               .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 // IMU & DEBUG communication
 {.pinNumber = EXT_DBG_SPI_SIMO,        .fctMux = EXT_DBG_SPI_SIMO_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = EXT_DBG_SPI_SOMI,        .fctMux = EXT_DBG_SPI_SOMI_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = EXT_DBG_SPI_CLK,         .fctMux = EXT_DBG_SPI_CLK_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = EXT_DBG_SPI_CS1,         .fctMux = EXT_DBG_SPI_CS1_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1_CLA1,  .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = EXT_SPI_CS2,             .fctMux = EXT_SPI_CS2_CFG,              .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1_CLA1,  .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 // RGB LED communication
 {.pinNumber = LED_SPI_SIMO,            .fctMux = LED_SPI_SIMO_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 // DRV1 configuration pins
 {.pinNumber = MOTOR1_DRV_GPIO_EN,      .fctMux = MOTOR1_DRV_GPIO_EN_CFG,       .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM1_CHA,         .fctMux = MOTOR1_PWM1_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM1_CHB,         .fctMux = MOTOR1_PWM1_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM2_CHA,         .fctMux = MOTOR1_PWM2_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM2_CHB,         .fctMux = MOTOR1_PWM2_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM3_CHA,         .fctMux = MOTOR1_PWM3_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM3_CHB,         .fctMux = MOTOR1_PWM3_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_DRV_SPI_CS,       .fctMux = MOTOR1_DRV_SPI_CS_CFG,        .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_DRV_GPIO_NFAULT,  .fctMux = MOTOR1_DRV_GPIO_NFAULT_CFG,   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_3SAMPLE,.setValue = 0,  .intEnable = false},
 // DRV2 configuration pins
 {.pinNumber = MOTOR2_DRV_GPIO_EN,      .fctMux = MOTOR2_DRV_GPIO_EN_CFG,       .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM1_CHA,         .fctMux = MOTOR2_PWM1_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM1_CHB,         .fctMux = MOTOR2_PWM1_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM2_CHA,         .fctMux = MOTOR2_PWM2_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM2_CHB,         .fctMux = MOTOR2_PWM2_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM3_CHA,         .fctMux = MOTOR2_PWM3_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM3_CHB,         .fctMux = MOTOR2_PWM3_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_DRV_SPI_CS,       .fctMux = MOTOR2_DRV_SPI_CS_CFG,        .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_DRV_GPIO_NFAULT,  .fctMux = MOTOR2_DRV_GPIO_NFAULT_CFG,   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_3SAMPLE,.setValue = 0,  .intEnable = false},
#elif (defined UOMODRI_V2_ENABLE)
#if (defined USB_BUS_ENABLE) && (!defined UART_BUS_ENABLE)
 // USB Pins
 {.pinNumber = USB_M,                   .fctMux = USB_M_CFG,                    .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false, .analogModeEnable = GPIO_ANALOG_ENABLED},
 {.pinNumber = USB_P,                   .fctMux = USB_P_CFG,                    .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false, .analogModeEnable = GPIO_ANALOG_ENABLED},
#elif (!defined USB_BUS_ENABLE) && (defined UART_BUS_ENABLE)
 // UART Pins
 {.pinNumber = UART_ON_USB_TX,          .fctMux = UART_ON_USB_TX_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = UART_ON_USB_RX,          .fctMux = UART_ON_USB_RX_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
#endif
 {.pinNumber = USB_VBUS_DETECT,         .fctMux = USB_VBUS_DETECT_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 // GPIO Debug pins
 {.pinNumber = DBG_PIN0,                .fctMux = DBG_PIN0_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = DBG_PIN1,                .fctMux = DBG_PIN1_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = DBG_PIN2,                .fctMux = DBG_PIN2_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = DBG_PIN3,                .fctMux = DBG_PIN3_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 // GPIO Select pins
 {.pinNumber = SELECT_PIN_0,            .fctMux = SELECT_PIN_0_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = SELECT_PIN_1,            .fctMux = SELECT_PIN_1_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = SELECT_PIN_2,            .fctMux = SELECT_PIN_2_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = SELECT_PIN_3,            .fctMux = SELECT_PIN_3_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
#if (defined CAN_BUS_ENABLE) && (!defined RS485_BUS_ENABLE)
 // CAN BUS IO pins
 {.pinNumber = CAN_RX,                  .fctMux = CAN_RX_CFG,                   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = CAN_TX,                  .fctMux = CAN_TX_CFG,                   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false},
#elif (!defined CAN_BUS_ENABLE) && (defined RS485_BUS_ENABLE)
 // RS485 BUS IO pins
 {.pinNumber = RS485_RX,                .fctMux = RS485_RX_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = RS485_TX,                .fctMux = RS485_TX_CFG,                 .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false},
#endif
 // RS-485 BUS IO pins
 {.pinNumber = CAN_SHDN_EN,             .fctMux = CAN_SHDN_EN_CFG,              .pinType = GPIO_PIN_TYPE_OD,    .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = RS485_RX_EN,             .fctMux = RS485_RX_EN_CFG,              .pinType = GPIO_PIN_TYPE_OD,    .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = RS485_TX_EN,             .fctMux = RS485_TX_EN_CFG,              .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 // Quadrature encoder #1
//#if (defined UOMODRI_V2_ENABLE) && (UOMODRI_V2_ENABLE)
 {.pinNumber = MOTOR1_ENC_CHA,          .fctMux = MOTOR1_ENC_CHA_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_ENC_CHB,          .fctMux = MOTOR1_ENC_CHB_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_ENC_CHI,          .fctMux = MOTOR1_ENC_CHI_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 // Quadrature encoder #2
 {.pinNumber = MOTOR2_ENC_CHA,          .fctMux = MOTOR2_ENC_CHA_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_ENC_CHB,          .fctMux = MOTOR2_ENC_CHB_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_ENC_CHI,          .fctMux = MOTOR2_ENC_CHI_CFG,           .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_6SAMPLE,.setValue = 0,  .intEnable = false},
 // DRV communication
 {.pinNumber = DRV_SPI_SIMO,            .fctMux = DRV_SPI_SIMO_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = DRV_SPI_SOMI,            .fctMux = DRV_SPI_SOMI_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_SYNC,   .setValue = 0,  .intEnable = false},
 {.pinNumber = DRV_SPI_CLK,             .fctMux = DRV_SPI_CLK_CFG,              .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 // OMODRI communication
 {.pinNumber = COM_SPI_SIMO,            .fctMux = COM_SPI_SIMO_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = COM_SPI_SOMI,            .fctMux = COM_SPI_SOMI_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = COM_SPI_CLK,             .fctMux = COM_SPI_CLK_CFG,              .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = COM_SPI_CS,              .fctMux = COM_SPI_CS_CFG,               .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 // IMU, EXTENSION & DEBUG communication
 {.pinNumber = IMU_EXT_DBG_SPI_SIMO,    .fctMux = IMU_EXT_DBG_SPI_SIMO_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = IMU_EXT_DBG_SPI_SOMI,    .fctMux = IMU_EXT_DBG_SPI_SOMI_CFG,     .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = IMU_EXT_DBG_SPI_CLK,     .fctMux = IMU_EXT_DBG_SPI_CLK_CFG,      .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = EXT_DBG_SPI_CS1,         .fctMux = EXT_DBG_SPI_CS1_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1_CLA1,  .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = EXT_SPI_CS2,             .fctMux = EXT_SPI_CS2_CFG,              .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1_CLA1,  .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = IMU_SPI_ACCEL_CS,        .fctMux = IMU_SPI_ACCEL_CS_CFG,         .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1_CLA1,  .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = IMU_SPI_GYRO_CS,         .fctMux = IMU_SPI_GYRO_CS_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1_CLA1,  .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 // RGB LED communication
 {.pinNumber = LED_SPI_SIMO,            .fctMux = LED_SPI_SIMO_CFG,             .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 // DRV1 configuration pins
 {.pinNumber = MOTOR1_DRV_GPIO_EN,      .fctMux = MOTOR1_DRV_GPIO_EN_CFG,       .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM1_CHA,         .fctMux = MOTOR1_PWM1_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM1_CHB,         .fctMux = MOTOR1_PWM1_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM2_CHA,         .fctMux = MOTOR1_PWM2_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM2_CHB,         .fctMux = MOTOR1_PWM2_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM3_CHA,         .fctMux = MOTOR1_PWM3_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR1_PWM3_CHB,         .fctMux = MOTOR1_PWM3_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_DRV_SPI_CS,       .fctMux = MOTOR1_DRV_SPI_CS_CFG,        .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR1_DRV_GPIO_NFAULT,  .fctMux = MOTOR1_DRV_GPIO_NFAULT_CFG,   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_3SAMPLE,.setValue = 0,  .intEnable = false},
 // DRV2 configuration pins
 {.pinNumber = MOTOR2_DRV_GPIO_EN,      .fctMux = MOTOR2_DRV_GPIO_EN_CFG,       .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM1_CHA,         .fctMux = MOTOR2_PWM1_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM1_CHB,         .fctMux = MOTOR2_PWM1_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM2_CHA,         .fctMux = MOTOR2_PWM2_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM2_CHB,         .fctMux = MOTOR2_PWM2_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM3_CHA,         .fctMux = MOTOR2_PWM3_CHA_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 0,  .intEnable = false},
 {.pinNumber = MOTOR2_PWM3_CHB,         .fctMux = MOTOR2_PWM3_CHB_CFG,          .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_DRV_SPI_CS,       .fctMux = MOTOR2_DRV_SPI_CS_CFG,        .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_OUT, .samplingInMode = GPIO_QUAL_ASYNC,  .setValue = 1,  .intEnable = false},
 {.pinNumber = MOTOR2_DRV_GPIO_NFAULT,  .fctMux = MOTOR2_DRV_GPIO_NFAULT_CFG,   .pinType = GPIO_PIN_TYPE_STD,   .coreSelect = GPIO_CORE_CPU1,       .direction = GPIO_DIR_MODE_IN,  .samplingInMode = GPIO_QUAL_3SAMPLE,.setValue = 0,  .intEnable = false},
#endif
 // END OF ARRAY
 {.pinNumber = UINT32_MAX},
};

 /** @var    ADC_Init_TypeDef AdcInitList[]
  *  @brief  List all the configurations defined for ADC Start-Of-Conversion (SOC) submodule.
  */
static const adc_ini_t  AdcInitList[] =
{
#ifdef UOMODRI_V1_ENABLE
 /* ADCA - ADCINA0 : Ia Motor1, ADCINA1 : Ia Motor2, ADCINA2 : Va Motor1, ADCINA3 : Va Motor2 */
 {.adcBase = MOTOR1_VA_ADC_ADDR,    .adcChannel = MOTOR1_VA_CH, .adcSOCNumber = MOTOR1_VA_SOC_NUM,  .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT1},
 {.adcBase = MOTOR2_VA_ADC_ADDR,    .adcChannel = MOTOR2_VA_CH, .adcSOCNumber = MOTOR2_VA_SOC_NUM,  .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT2},
 {.adcBase = MOTOR1_IA_ADC_ADDR,    .adcChannel = MOTOR1_IA_CH, .adcSOCNumber = MOTOR1_IA_SOC_NUM,  .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT3},
 {.adcBase = MOTOR2_IA_ADC_ADDR,    .adcChannel = MOTOR2_IA_CH, .adcSOCNumber = MOTOR2_IA_SOC_NUM,  .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT4},
 /* ADCB - ADCINB0 : Ib Motor1, ADCINB1 : Ib Motor2, ADCINB2 : Vb Motor1, ADCINB3 : Vb Motor2 */
 {.adcBase = MOTOR1_VB_ADC_ADDR,    .adcChannel = MOTOR1_VB_CH, .adcSOCNumber = MOTOR1_VB_SOC_NUM,  .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT1},
 {.adcBase = MOTOR2_VB_ADC_ADDR,    .adcChannel = MOTOR2_VB_CH, .adcSOCNumber = MOTOR2_VB_SOC_NUM,  .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT2},
 {.adcBase = MOTOR1_IB_ADC_ADDR,    .adcChannel = MOTOR1_IB_CH, .adcSOCNumber = MOTOR1_IB_SOC_NUM,  .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT3},
 {.adcBase = MOTOR2_IB_ADC_ADDR,    .adcChannel = MOTOR2_IB_CH, .adcSOCNumber = MOTOR2_IB_SOC_NUM,  .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT4},
 /* ADCD - ADCIND0 : Ic Motor1, ADCIND1 : Ic Motor2, ADCIND2 : Vc Motor1, ADCIND3 : Vc Motor2 */
 {.adcBase = MOTOR1_VC_ADC_ADDR,    .adcChannel = MOTOR1_VC_CH, .adcSOCNumber = MOTOR1_VC_SOC_NUM,  .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT1},
 {.adcBase = MOTOR2_VC_ADC_ADDR,    .adcChannel = MOTOR2_VC_CH, .adcSOCNumber = MOTOR2_VC_SOC_NUM,  .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT2},
 {.adcBase = MOTOR1_IC_ADC_ADDR,    .adcChannel = MOTOR1_IC_CH, .adcSOCNumber = MOTOR1_IC_SOC_NUM,  .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT3},
 {.adcBase = MOTOR2_IC_ADC_ADDR,    .adcChannel = MOTOR2_IC_CH, .adcSOCNumber = MOTOR2_IC_SOC_NUM,  .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT4},
 /* ADCC - ADCINC2 : Vbus Motor1, ADCINC14 : Vin1 external analog input, ADCINC15 : Vin2 external analog input */
 {.adcBase = VBUS_ADC_ADDR,         .adcChannel = VBUS_CH,      .adcSOCNumber = VBUS_SOC_NUM,       .adcTriggerSrc = VBUS_ADC_SOC_TRIGGER_EVT},
 {.adcBase = EXTERN_V1_ADC_ADDR,    .adcChannel = EXTERN_V1_CH, .adcSOCNumber = EXTERN_V1_SOC_NUM,  .adcTriggerSrc = EXTERN_ADC_SOC_TRIGGER_EVT1},
 {.adcBase = EXTERN_V2_ADC_ADDR,    .adcChannel = EXTERN_V2_CH, .adcSOCNumber = EXTERN_V2_SOC_NUM,  .adcTriggerSrc = EXTERN_ADC_SOC_TRIGGER_EVT2},
#elif (defined UOMODRI_V2_ENABLE)
 /* ADCA - ADCINA0 : Ia Motor2, ADCINA1 : Ia Motor1 */
 {.adcBase = MOTOR1_IA_ADC_ADDR,.adcChannel = MOTOR1_IA_CH, .adcSOCNumber = MOTOR1_Iabc_SOC0_NUM,   .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT},
 {.adcBase = MOTOR2_IA_ADC_ADDR,.adcChannel = MOTOR2_IA_CH, .adcSOCNumber = MOTOR2_Iabc_SOC0_NUM,   .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT},
 /* ADCB - ADCINB0 : Ib Motor2, ADCINB1 : Ib Motor1 */
 {.adcBase = MOTOR1_IB_ADC_ADDR,.adcChannel = MOTOR1_IB_CH, .adcSOCNumber = MOTOR1_Iabc_SOC0_NUM,   .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT},
 {.adcBase = MOTOR2_IB_ADC_ADDR,.adcChannel = MOTOR2_IB_CH, .adcSOCNumber = MOTOR2_Iabc_SOC0_NUM,   .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT},
 /* ADCC - ADCINC2 : Ic Motor, ADCINC3 : Ic Motor1 */
 {.adcBase = MOTOR1_IC_ADC_ADDR,.adcChannel = MOTOR1_IC_CH, .adcSOCNumber = MOTOR1_Iabc_SOC0_NUM,   .adcTriggerSrc = MOTOR1_ADC_SOC_TRIGGER_EVT},
 {.adcBase = MOTOR2_IC_ADC_ADDR,.adcChannel = MOTOR2_IC_CH, .adcSOCNumber = MOTOR2_Iabc_SOC0_NUM,   .adcTriggerSrc = MOTOR2_ADC_SOC_TRIGGER_EVT},
 /* ADCD - ADCIND0 : Vbus, ADCINC14 : Vin1 external analog input, ADCINC15 : Vin2 external analog input */
 {.adcBase = VBUS_ADC_ADDR,     .adcChannel = VBUS_CH,      .adcSOCNumber = VBUS_SOC0_NUM,          .adcTriggerSrc = VBUS_ADC_SOC_TRIGGER_EVT},
 {.adcBase = VEXT1_ADC_ADDR,    .adcChannel = VEXT1_CH,     .adcSOCNumber = VEXT1_SOC0_NUM,         .adcTriggerSrc = VEXT1_ADC_SOC_TRIGGER_EVT},
 {.adcBase = VEXT2_ADC_ADDR,    .adcChannel = VEXT2_CH,     .adcSOCNumber = VEXT2_SOC0_NUM,         .adcTriggerSrc = VEXT2_ADC_SOC_TRIGGER_EVT},
 #endif
 // END OF ARRAY
 {.adcBase = UINT32_MAX},
};

/** @var    ADC_PPB_TypeDef AdcAcqList[]
 *  @brief  List all the configurations defined for ADC Post-Processing Block (PPB) submodule.
 */
static const adc_acq_t  AdcAcqList[] =
{
#ifdef UOMODRI_V1_ENABLE
 /* ADCA : Va_M1, Va_M2, Ia_M1, Ia_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = MOTOR1_VA_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_VA_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR1_VA_SOC_NUM,  .adcPPBNumber = MOTOR1_VA_PPB_NUM},
 {.adcBase = MOTOR2_VA_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_VA_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR2_VA_SOC_NUM,  .adcPPBNumber = MOTOR2_VA_PPB_NUM},
 {.adcBase = MOTOR1_IA_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_IA_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR1_IA_SOC_NUM,  .adcPPBNumber = MOTOR1_IA_PPB_NUM},
 {.adcBase = MOTOR2_IA_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_IA_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR2_IA_SOC_NUM,  .adcPPBNumber = MOTOR2_IA_PPB_NUM},
 /* ADCB : Vb_M1, Vb_M2, Ib_M1, Ib_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = MOTOR1_VB_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_VB_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR1_VB_SOC_NUM,  .adcPPBNumber = MOTOR1_VB_PPB_NUM},
 {.adcBase = MOTOR2_VB_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_VB_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR2_VB_SOC_NUM,  .adcPPBNumber = MOTOR2_VB_PPB_NUM},
 {.adcBase = MOTOR1_IB_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_IB_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR1_IB_SOC_NUM,  .adcPPBNumber = MOTOR1_IB_PPB_NUM},
 {.adcBase = MOTOR2_IB_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_IB_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR2_IB_SOC_NUM,  .adcPPBNumber = MOTOR2_IB_PPB_NUM},
 /* ADCD : Vc_M1, Vc_M2, Ic_M1, Ic_M2; SOC0 - 3; PPB1 - 4 */
 {.adcBase = MOTOR1_VC_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_VC_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR1_VC_SOC_NUM,  .adcPPBNumber = MOTOR1_VC_PPB_NUM},
 {.adcBase = MOTOR2_VC_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_VC_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR2_VC_SOC_NUM,  .adcPPBNumber = MOTOR2_VC_PPB_NUM},
 {.adcBase = MOTOR1_IC_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_IC_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR1_IC_SOC_NUM,  .adcPPBNumber = MOTOR1_IC_PPB_NUM},
 {.adcBase = MOTOR2_IC_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_IC_ADC_RESULT_ADDR),   .adcSOCNumber = MOTOR2_IC_SOC_NUM,  .adcPPBNumber = MOTOR2_IC_PPB_NUM},
 /* ADCC : Vbus, External_V1, External_V2; SOC0 & SOC2 - 3; PPB1 & PPB3 - 4 */
 {.adcBase = VBUS_ADC_ADDR,     .adcResultReg = (volatile uint16_t *)(VBUS_ADC_RESULT_ADDR),        .adcSOCNumber = VBUS_SOC_NUM,       .adcPPBNumber = VBUS_PPB_NUM},
 {.adcBase = EXTERN_V1_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(EXTERN_V1_ADC_RESULT_ADDR),   .adcSOCNumber = EXTERN_V1_SOC_NUM,  .adcPPBNumber = EXTERN_V1_PPB_NUM},
 {.adcBase = EXTERN_V2_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(EXTERN_V2_ADC_RESULT_ADDR),   .adcSOCNumber = EXTERN_V2_SOC_NUM,  .adcPPBNumber = EXTERN_V2_PPB_NUM},
#elif (defined UOMODRI_V2_ENABLE)
 /* ADCA : Ia_M1/SOC0/PPB1, Ia_M2/SOC4/PPB3 */
 {.adcBase = MOTOR1_IA_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_IA_ADC_RESULT0_ADDR),  .adcSOCNumber = MOTOR1_Iabc_SOC0_NUM,   .adcPPBNumber = MOTOR1_Iabc_PPB1_NUM},
 {.adcBase = MOTOR2_IA_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_IA_ADC_RESULT0_ADDR),  .adcSOCNumber = MOTOR2_Iabc_SOC0_NUM,   .adcPPBNumber = MOTOR2_Iabc_PPB1_NUM},
 /* ADCB : Ib_M1/SOC0/PPB1, Ib_M2/SOC4/PPB3 */
 {.adcBase = MOTOR1_IB_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_IB_ADC_RESULT0_ADDR),  .adcSOCNumber = MOTOR1_Iabc_SOC0_NUM,   .adcPPBNumber = MOTOR1_Iabc_PPB1_NUM},
 {.adcBase = MOTOR2_IB_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_IB_ADC_RESULT0_ADDR),  .adcSOCNumber = MOTOR2_Iabc_SOC0_NUM,   .adcPPBNumber = MOTOR2_Iabc_PPB1_NUM},
 /* ADCC : Ic_M1/SOC0/PPB1, Ic_M2/SOC4/PPB3 */
 {.adcBase = MOTOR1_IC_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR1_IC_ADC_RESULT0_ADDR),  .adcSOCNumber = MOTOR1_Iabc_SOC0_NUM,   .adcPPBNumber = MOTOR1_Iabc_PPB1_NUM},
 {.adcBase = MOTOR2_IC_ADC_ADDR,.adcResultReg = (volatile uint16_t *)(MOTOR2_IC_ADC_RESULT0_ADDR),  .adcSOCNumber = MOTOR2_Iabc_SOC0_NUM,   .adcPPBNumber = MOTOR2_Iabc_PPB1_NUM},
 /* ADCD : VBUS/SOC0/PPB1, VEXT1/SOC4/PPB3, VEXT2/SOC6/PPB4 */
 {.adcBase = VBUS_ADC_ADDR,     .adcResultReg = (volatile uint16_t *)(VBUS_ADC_RESULT0_ADDR),       .adcSOCNumber = VBUS_SOC0_NUM,          .adcPPBNumber = VBUS_PPB1_NUM},
 {.adcBase = VEXT1_ADC_ADDR,    .adcResultReg = (volatile uint16_t *)(VEXT1_ADC_RESULT0_ADDR),      .adcSOCNumber = VEXT1_SOC0_NUM,         .adcPPBNumber = VEXT1_PPB_NUM},
 {.adcBase = VEXT2_ADC_ADDR,    .adcResultReg = (volatile uint16_t *)(VEXT2_ADC_RESULT0_ADDR),      .adcSOCNumber = VEXT2_SOC0_NUM,         .adcPPBNumber = VEXT2_PPB_NUM},
#endif
 // END OF ARRAY
 {.adcBase = UINT32_MAX}
};

/** @var    ADC_IT_TypeDef AdcIntList[]
 *  @brief  List all the interrupts source generated by ADC modules.
 */
static const adc_int_t  AdcIntList[] =
{
#ifdef UOMODRI_V1_ENABLE
 {.adcBase = MOTOR12_IVA_ADC_ADDR, .adcEOCNumber = MOTOR1_IA_SOC_NUM,   .adcIntNumber = MOTOR1_IA_INT_NUM},
 {.adcBase = MOTOR12_IVA_ADC_ADDR, .adcEOCNumber = MOTOR2_IA_SOC_NUM,   .adcIntNumber = MOTOR2_IA_INT_NUM},
#elif (defined UOMODRI_V2_ENABLE)
 {.adcBase = MOTOR12_IA_ADC_ADDR,   .adcEOCNumber = MOTOR1_Iabc_SOC0_NUM,  .adcIntNumber = MOTOR1_IA_INT_NUM},
 {.adcBase = MOTOR12_IA_ADC_ADDR,   .adcEOCNumber = MOTOR2_Iabc_SOC0_NUM,  .adcIntNumber = MOTOR2_IA_INT_NUM},
#endif
 // END OF ARRAY
 {.adcBase = UINT32_MAX}
};

/** @var    ADC_TypeDef AdcCfgList[]
 *  @brief  List all the configurations defined for each ADC modules.
 */
static const adc_cfg_t  AdcCfgList[] =
{
#ifdef UOMODRI_V1_ENABLE
 {.adcBase = MOTOR12_IVA_ADC_ADDR,  .p_adcIni = &AdcInitList[0],    .p_adcAcq = &AdcAcqList[0], .p_adcInt = &AdcIntList[0]},
 {.adcBase = MOTOR12_IVB_ADC_ADDR,  .p_adcIni = &AdcInitList[4],    .p_adcAcq = &AdcAcqList[4], .p_adcInt = NULL},
 {.adcBase = MOTOR12_IVC_ADC_ADDR,  .p_adcIni = &AdcInitList[8],    .p_adcAcq = &AdcAcqList[8], .p_adcInt = NULL},
 {.adcBase = VBUS_ADC_ADDR,         .p_adcIni = &AdcInitList[12],   .p_adcAcq = &AdcAcqList[12],.p_adcInt = NULL},
#elif (defined UOMODRI_V2_ENABLE)
 {.adcBase = MOTOR12_IA_ADC_ADDR,   .p_adcIni = &AdcInitList[0],.p_adcAcq = &AdcAcqList[0], .p_adcInt = &AdcIntList[0]},
 {.adcBase = MOTOR12_IB_ADC_ADDR,   .p_adcIni = &AdcInitList[2],.p_adcAcq = &AdcAcqList[2], .p_adcInt = NULL},
 {.adcBase = MOTOR12_IC_ADC_ADDR,   .p_adcIni = &AdcInitList[4],.p_adcAcq = &AdcAcqList[4], .p_adcInt = NULL},
 {.adcBase = VBUS_VEXT12_ADC_ADDR,  .p_adcIni = &AdcInitList[6],.p_adcAcq = &AdcAcqList[6], .p_adcInt = NULL},
#endif
 // END OF ARRAY
 {.adcBase = UINT32_MAX},
};

/** @var    eQEP_TypeDef QepCfgList[]
 *  @brief  List all the configurations defined for each Quadrature Encoders modules.
 */
static const eqep_cfg_t QepCfgList[] =
{
 {.eqepBase = MOTOR1_QEP_BASE,  .eqepMaxResolution = (MOTOR1_ENC_QUADRATURE_SCALE - 1), .eqepUnitTimerPeriod = (MOTOR1_ENC_SPEED_HIGH_SAMPLING_TICKS - 1),  .eqepConfig = MOTOR1_ENC_CONFIG,    .eqepStrobeSrc = EQEP_STROBE_OR_ADCSOCA},
 {.eqepBase = MOTOR2_QEP_BASE,  .eqepMaxResolution = (MOTOR2_ENC_QUADRATURE_SCALE - 1), .eqepUnitTimerPeriod = (MOTOR2_ENC_SPEED_HIGH_SAMPLING_TICKS - 1),  .eqepConfig = MOTOR2_ENC_CONFIG,    .eqepStrobeSrc = EQEP_STROBE_OR_ADCSOCB},
 // END OF ARRAY
 {.eqepBase             = UINT32_MAX},
};

//static const sci_cfg_t sciCfgList[] =
//{
// {
//  .sciBase              = DBG_SCI_BASE,
//  .bitRate              = 3125000U,
//  .mode                 = SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | (uint16_t)SCI_CONFIG_PAR_NONE,
//  .txLevel              = SCI_FIFO_TX16,
//  .rxLevel              = SCI_FIFO_RX16,
//  .intEnable            = false,
// },
// // END OF ARRAY
// {.sciBase              = UINT32_MAX},
//};

/** @var    SPI_TypeDef spiCfgList[]
 *  @brief  List all the configurations defined for each SPI communication modules.
 */
static const spi_cfg_t  spiCfgList[] =
{
 {.spiBase = DRV_SPI_BASE,  .bitRate = 400000U,     .dataWidth = 16U,   .mode = SPI_MODE_MASTER,.protocol = SPI_PROT_POL0PHA0,  .txLevel = SPI_FIFO_TXEMPTY,.rxLevel = SPI_FIFO_RXEMPTY,.intSrc = 0,.highSpeedMode = false, .intEnable = false},
 {.spiBase = COM_SPI_BASE,  .bitRate = 6000000U,    .dataWidth = 16U,   .mode = SPI_MODE_SLAVE, .protocol = SPI_PROT_POL0PHA1,  .txLevel = SPI_FIFO_TXFULL, .rxLevel = SPI_FIFO_RX1,    .intSrc = 0,.highSpeedMode = false, .intEnable = false},
 {.spiBase = DBG_SPI_BASE,  .bitRate = 10000000U,   .dataWidth = 16U,   .mode = SPI_MODE_MASTER,.protocol = SPI_PROT_POL0PHA0,  .txLevel = SPI_FIFO_TX8,    .rxLevel = SPI_FIFO_RX8,    .intSrc = 0,.highSpeedMode = false, .intEnable = false},
 {.spiBase = LED_SPI_BASE,  .bitRate = 2380000U,    .dataWidth = 16U,   .mode = SPI_MODE_MASTER,.protocol = SPI_PROT_POL0PHA0,  .txLevel = SPI_FIFO_TX6,    .rxLevel = SPI_FIFO_RXEMPTY,.intSrc = 0,.highSpeedMode = false, .intEnable = false},
 // END OF ARRAY
 {.spiBase = UINT32_MAX},
};

static const dma_cfg_t  dmaCfgList[] =
{
 // DMA base register initialization
 // DMA for SPI communication Tx
 {
  .dmaChBase            = DMA_COM_SPI_TX_BASE_ADDR,
  .p_dstAddr            = (uint16_t *) COM_SPI_TX_BASE_REG_ADDR,
  .p_srcAddr            = (uint16_t *) &status_msg.timeStamp,
  .burstSize            = DMA_COM_SPI_TX_16BIT_BURST_EVT,
  .transferSize         = DMA_COM_SPI_TX_16BIT_TRANSFER_EVT,
  .dstWrapSize          = DMA_NO_WRAP,
  .srcWrapSize          = DMA_NO_WRAP,
  .dstStep              = 0,
  .srcStep              = DMA_COM_SPI_TX_16BIT_INC_STEP,
  .dstWrapStep          = 0,
  .srcWrapStep          = 0,
  .config               = DMA_COM_SPI_TX_16BIT_CONFIG,
  .trigger              = DMA_COM_SPI_TX_TRIGGER_SRC,
  .intMode              = DMA_INT_AT_END,
  .intEnable            = false,
 },
 // DMA for SPI communication Rx
 {
  .dmaChBase            = DMA_COM_SPI_RX_BASE_ADDR,
  .p_dstAddr            = (uint16_t *) &cmd_msg,
  .p_srcAddr            = (uint16_t *) COM_SPI_RX_BASE_REG_ADDR,
  .burstSize            = DMA_COM_SPI_RX_16BIT_BURST_EVT,
  .transferSize         = DMA_COM_SPI_RX_16BIT_TRANSFER_EVT,
  .dstWrapSize          = DMA_NO_WRAP,
  .srcWrapSize          = DMA_NO_WRAP,
  .dstStep              = DMA_COM_SPI_RX_16BIT_INC_STEP,
  .srcStep              = 0,
  .dstWrapStep          = 0,
  .srcWrapStep          = 0,
  .config               = DMA_COM_SPI_RX_16BIT_CONFIG,
  .trigger              = DMA_COM_SPI_RX_TRIGGER_SRC,
  .intMode              = DMA_INT_AT_END,
  .intEnable            = false,
 },
//#if (defined UOMODRI_USE_REV_1_0_CONFIG) && (!UOMODRI_USE_REV_1_0_CONFIG)
// {
//  .dmaChBase            = DMA_ENC_SPI_TX_BASE_ADDR,
//  .p_dstAddr            = (uint16_t *) ENC_SPI_TX_BASE_REG_ADDR,
//  .p_srcAddr            = (uint16_t *) &cmd_msg,
//  .burstSize            = 17,
//  .transferSize         = 1,
//  .dstWrapSize          = DMA_NO_WRAP,
//  .srcWrapSize          = DMA_NO_WRAP,
//  .dstStep              = 0,
//  .srcStep              = 1,
//  .dstWrapStep          = 0,
//  .srcWrapStep          = 0,
//  .config               = (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT),
//  .trigger              = DMA_ENC_SPI_TX_TRIGGER_SRC,
//  .intMode              = DMA_INT_AT_BEGINNING,
//  .intEnable            = false,
// },
//{
// .dmaChBase        = DMA_ENC_SPI_RX_BASE_ADDR,
// .burstSize        = DMA_ENC_SPI_TX_RX_16BIT_BURST_EVT, // an object of type uint32_t or float32_t
// .transferSize     = DMA_ENC_SPI_TX_RX_TRANSFER_EVT,
// .p_destAddr       = (uint16_t *) &status_Master_msg,
// .destWrapSize     = UINT32_MAX,//DMA_ENC_SPI_TX_RX_TRANSFER_EVT / 2,//
// .destStep         = 1,
// .p_srcAddr        = (uint16_t *) ENC_SPI_RX_BASE_ADDR_REG,
// .srcWrapSize      = 0,
// .srcStep          = 0,
// .config           = (DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT),
// .trigger          = DMA_TRIGGER_SPICRX,
// .ITMode           = DMA_INT_AT_END,
// .ITEnable         = false,
//},
//#endif
 // DMA for the RGB LED
 {
  .dmaChBase            = DMA_LED_SPI_TX_BASE_ADDR,
  .p_dstAddr            = (uint16_t *) LED_SPI_TX_BASE_REG_ADDR,
  .p_srcAddr            = (uint16_t *) &led_msg[0],
  .burstSize            = DMA_LED_SPI_TX_16BIT_BURST_EVT,
  .transferSize         = DMA_LED_SPI_TX_16BIT_TRANSFER_EVT,
  .dstWrapSize          = DMA_NO_WRAP,//0,//
  .srcWrapSize          = DMA_NO_WRAP,//DMA_COM_SPI_TX_RX_TRANSFER_EVT,//
  .dstStep              = 0,
  .srcStep              = DMA_COM_SPI_TX_16BIT_INC_STEP,
  .dstWrapStep          = 0,
  .srcWrapStep          = 0,//-(DMA_COM_SPI_TX_RX_TRANSFER_EVT - 1),//
  .config               = DMA_LED_SPI_TX_16BIT_CONFIG,
  .trigger              = DMA_LED_SPI_TX_TRIGGER_SRC,
  .intMode              = DMA_INT_AT_END,
  .intEnable            = false,
 },
 // END OF ARRAY
 {.dmaChBase            = UINT32_MAX},
};

static const ipc_cfg_t  ipcCfgList[] =
{
 {
  .ipcType              = IPC_CPU1_L_CM_R,
  .ipcFlag              = IPC_CPU1_TO_CM_FLAG,
  .ipcMsgQEnable        = false,
  .ipcIntEnable         = false,
 },
  // END OF ARRAY
 {.ipcType              = IPC_TOTAL_NUM},
};

static const tmr_cfg_t  timerCfgList[] =
{
 {
  // CPU timer 0
  .cpuTimerBase         = CPU_TIMER_0_BASE,
  .periodCount          = CPU_TIMER_0_PERIOD,
  .prescaler            = CPU_TIMER_0_PRESCALER,
  .intEnable            = false,
 },
 // END OF ARRAY
 {.cpuTimerBase         = UINT32_MAX},
};

//static const cla_cfg_t  claCfgList[] =
//{
// {
//  .progRAM              = MEMCFG_SECT_LS2,
//  .dataRAM              = (MEMCFG_SECT_LS0 | MEMCFG_SECT_LS1),
//  .claTaskFlg           = CLA_TASKFLAG_1,
//  .claTaskNum           = CLA_TASK_1,
////  .claIntVect           = CLA_MVECT_1,
//  .fun_ptr              = &Cla1Task1,
// },
//};

/** @var    ePWM_TB_TypeDef PwmTimeBaseCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Time Base submodule.
 */
static const epwm_tb_t  PwmTimeBaseCfgList[] =
{
#ifdef UOMODRI_V1_ENABLE
 {.epwmBase = MOTOR1_PWM1_BASE, .phaseCount = PWM_INITIAL_PHASE_0,  .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO},
 {.epwmBase = MOTOR1_PWM2_BASE, .phaseCount = PWM_INITIAL_PHASE_0,  .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR1_PWM3_BASE, .phaseCount = PWM_INITIAL_PHASE_0,  .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR2_PWM1_BASE, .phaseCount = PWM_INITIAL_PHASE_90, .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR2_PWM2_BASE, .phaseCount = PWM_INITIAL_PHASE_90, .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
 {.epwmBase = MOTOR2_PWM3_BASE, .phaseCount = PWM_INITIAL_PHASE_90, .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE},
#elif (defined UOMODRI_V2_ENABLE)
 {.epwmBase = MOTOR1_PWM1_BASE, .phaseCount = PWM_INITIAL_PHASE_180,.syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO, .syncInputSrc = EPWM_SYNC_IN_PULSE_SRC_DISABLE},
 {.epwmBase = MOTOR1_PWM2_BASE, .phaseCount = PWM_INITIAL_PHASE_180,.syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE,  .syncInputSrc = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1},
 {.epwmBase = MOTOR1_PWM3_BASE, .phaseCount = PWM_INITIAL_PHASE_180,.syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE,  .syncInputSrc = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1},
 {.epwmBase = MOTOR2_PWM1_BASE, .phaseCount = PWM_INITIAL_PHASE_0,  .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE,  .syncInputSrc = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1},
 {.epwmBase = MOTOR2_PWM2_BASE, .phaseCount = PWM_INITIAL_PHASE_0,  .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE,  .syncInputSrc = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1},
 {.epwmBase = MOTOR2_PWM3_BASE, .phaseCount = PWM_INITIAL_PHASE_0,  .syncOutputMode = EPWM_SYNC_OUT_PULSE_ON_SOFTWARE,  .syncInputSrc = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1},
#endif
 // END OF ARRAY
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_CC_TypeDef PwmCounterCompareCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Capture/Compare submodule.
 */
static const epwm_cc_t  PwmCounterCompareCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .compCount = PWM_INITIAL_COMP_VAL,  .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR1_PWM2_BASE, .compCount = PWM_INITIAL_COMP_VAL,  .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR1_PWM3_BASE, .compCount = PWM_INITIAL_COMP_VAL,  .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR2_PWM1_BASE, .compCount = PWM_INITIAL_COMP_VAL,  .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR2_PWM2_BASE, .compCount = PWM_INITIAL_COMP_VAL,  .compModule = EPWM_COUNTER_COMPARE_A},
 {.epwmBase = MOTOR2_PWM3_BASE, .compCount = PWM_INITIAL_COMP_VAL,  .compModule = EPWM_COUNTER_COMPARE_A},
 // END OF ARRAY
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_AQ_TypeDef PwmActionQualifierCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Action Qualifier submodule.
 */
static const epwm_aq_t  PwmActionQualifierCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR1_PWM2_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR1_PWM3_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR2_PWM1_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR2_PWM2_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 {.epwmBase = MOTOR2_PWM3_BASE, .actionsEvt = (EPWM_AQ_OUTPUT_HIGH_UP_CMPA | EPWM_AQ_OUTPUT_LOW_DOWN_CMPA), .epwmOutput = EPWM_AQ_OUTPUT_A},
 // END OF ARRAY
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_DB_TypeDef PwmDeadBandCfgList[]
 *  @brief  List all the configurations necessaries for ePWM DeadBand Unit submodule.
 */
static const epwm_db_t  PwmDeadBandCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .redCount = PWM_DEADBAND_RISING_EDGE_DELAY, .fedCount = PWM_DEADBAND_FALLING_EDGE_DELAY,    .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR1_PWM2_BASE, .redCount = PWM_DEADBAND_RISING_EDGE_DELAY, .fedCount = PWM_DEADBAND_FALLING_EDGE_DELAY,    .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR1_PWM3_BASE, .redCount = PWM_DEADBAND_RISING_EDGE_DELAY, .fedCount = PWM_DEADBAND_FALLING_EDGE_DELAY,    .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR2_PWM1_BASE, .redCount = PWM_DEADBAND_RISING_EDGE_DELAY, .fedCount = PWM_DEADBAND_FALLING_EDGE_DELAY,    .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR2_PWM2_BASE, .redCount = PWM_DEADBAND_RISING_EDGE_DELAY, .fedCount = PWM_DEADBAND_FALLING_EDGE_DELAY,    .DelayMode = EPWM_DELAY_ENABLE},
 {.epwmBase = MOTOR2_PWM3_BASE, .redCount = PWM_DEADBAND_RISING_EDGE_DELAY, .fedCount = PWM_DEADBAND_FALLING_EDGE_DELAY,    .DelayMode = EPWM_DELAY_ENABLE},
 // END OF ARRAY
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_ET_TypeDef PwmAdcSocEventTriggerCfgList[]
 *  @brief  List all the configurations necessaries for ePWM Event Trigger submodule.
 */
static const epwm_et_t  PwmAdcSocEventTriggerCfgList[] =
{
#ifdef UOMODRI_V1_ENABLE
 {.epwmBase = MOTOR1_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_B, .adcSOCSource = EPWM_SOC_TBCTR_PERIOD},
 {.epwmBase = MOTOR1_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_A, .adcSOCSource = EPWM_SOC_TBCTR_ZERO},
 {.epwmBase = MOTOR2_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_A, .adcSOCSource = EPWM_SOC_TBCTR_PERIOD},
 {.epwmBase = MOTOR2_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_B, .adcSOCSource = EPWM_SOC_TBCTR_ZERO},
#elif (defined UOMODRI_V2_ENABLE)
 {.epwmBase = MOTOR1_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_A, .adcSOCSource = EPWM_SOC_TBCTR_ZERO},
 {.epwmBase = MOTOR2_PWM1_BASE, .adcSOCEvtCount = 1, .adcSOCType = EPWM_SOC_B, .adcSOCSource = EPWM_SOC_TBCTR_ZERO},
#endif
 // END OF ARRAY
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_IT_TypeDef PwmInterruptsEventCfgList[]
 *  @brief  List all the configurations necessaries for ePWM IT Event submodule.
 */
static const epwm_it_t  PwmInterruptsEventCfgList[] =
{
 {.epwmBase = MOTOR1_PWM1_BASE, .intEvtCount = 1, .intSource = EPWM_INT_TBCTR_ZERO},
 {.epwmBase = MOTOR2_PWM1_BASE, .intEvtCount = 1, .intSource = EPWM_INT_TBCTR_ZERO},
 // END OF ARRAY
 {.epwmBase = UINT32_MAX},
};

/** @var    ePWM_TypeDef PwmCfgList[]
 *  @brief  List all the configurations necessaries for PWM module.
 */
static const epwm_cfg_t PwmCfgList[] =
{
#ifdef UOMODRI_V1_ENABLE
 {.epwmBase                 = MOTOR1_PWM1_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[0],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[0],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[0],
  .p_epwmDeadband           = &PwmDeadBandCfgList[0],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = &PwmAdcSocEventTriggerCfgList[0],
  .p_epwmInterruptsEvent    = NULL,//&PwmInterruptsEventCfgList[0],
  .epwmAdcSocSrcExtern      = SYSCTL_ADCSOC_SRC_PWM1SOCA,
 },
 {.epwmBase                 = MOTOR1_PWM2_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[1],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[1],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[1],
  .p_epwmDeadband           = &PwmDeadBandCfgList[1],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
 {.epwmBase                 = MOTOR1_PWM3_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[2],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[2],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[2],
  .p_epwmDeadband           = &PwmDeadBandCfgList[2],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
 {.epwmBase                 = MOTOR2_PWM1_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[3],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[3],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[3],
  .p_epwmDeadband           = &PwmDeadBandCfgList[3],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = &PwmAdcSocEventTriggerCfgList[2],
  .p_epwmInterruptsEvent    = NULL,//&PwmInterruptsEventCfgList[1],
  .epwmAdcSocSrcExtern      = SYSCTL_ADCSOC_SRC_PWM4SOCB,
 },
 {.epwmBase                 = MOTOR2_PWM2_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[4],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[4],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[4],
  .p_epwmDeadband           = &PwmDeadBandCfgList[4],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
 {.epwmBase                 = MOTOR2_PWM3_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[5],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[5],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[5],
  .p_epwmDeadband           = &PwmDeadBandCfgList[5],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
#elif (defined UOMODRI_V2_ENABLE)
 {.epwmBase                 = MOTOR1_PWM1_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[0],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[0],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[0],
  .p_epwmDeadband           = &PwmDeadBandCfgList[0],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = &PwmAdcSocEventTriggerCfgList[0],
  .p_epwmInterruptsEvent    = NULL,//&PwmInterruptsEventCfgList[0],
  .epwmAdcSocSrcExtern      = SYSCTL_ADCSOC_SRC_PWM1SOCA,
 },
 {.epwmBase                 = MOTOR1_PWM2_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[1],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[1],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[1],
  .p_epwmDeadband           = &PwmDeadBandCfgList[1],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
 {.epwmBase                 = MOTOR1_PWM3_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[2],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[2],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[2],
  .p_epwmDeadband           = &PwmDeadBandCfgList[2],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
 {.epwmBase                 = MOTOR2_PWM1_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[3],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[3],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[3],
  .p_epwmDeadband           = &PwmDeadBandCfgList[3],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = &PwmAdcSocEventTriggerCfgList[2],
  .p_epwmInterruptsEvent    = NULL,//&PwmInterruptsEventCfgList[1],
  .epwmAdcSocSrcExtern      = SYSCTL_ADCSOC_SRC_PWM4SOCB,
 },
 {.epwmBase                 = MOTOR2_PWM2_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[4],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[4],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[4],
  .p_epwmDeadband           = &PwmDeadBandCfgList[4],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
 {.epwmBase                 = MOTOR2_PWM3_BASE,
  .p_epwmTimeBase           = &PwmTimeBaseCfgList[5],
  .p_epwmCaptureCompare     = &PwmCounterCompareCfgList[5],
  .p_epwmActionQualifier    = &PwmActionQualifierCfgList[5],
  .p_epwmDeadband           = &PwmDeadBandCfgList[5],
  .p_epwmProgChopper        = NULL,
  .p_epwmAdcSocEventTrigger = NULL,
  .p_epwmInterruptsEvent    = NULL,
  .epwmAdcSocSrcExtern      = 0,
 },
#endif
 // END OF ARRAY
 {.epwmBase                 = UINT32_MAX},
};

/** @var    HAL     hal
 *  @brief  General configuration structure for the Hardware Abstraction Layer.
 */
static const hal_cfg_t hal =
{.p_gpioHandle              = GpioPinsCfgList,
 .p_adcHandle               = AdcCfgList,
 .p_epwmHandle              = PwmCfgList,
 .p_eqepHandle              = QepCfgList,
 .p_dmaHandle               = dmaCfgList,
#if (defined CM_CORE_ENABLE) && (CM_CORE_ENABLE)
 .p_ipcHandle               = ipcCfgList,
#else
 .p_ipcHandle               = NULL,
#endif
 .p_sciHandle               = NULL,
 .p_spiHandle               = spiCfgList,
 .p_timerHandle             = timerCfgList,
#if (defined CLA_CORE_ENABLE) && (CLA_CORE_ENABLE)
 .p_claHandle               = claCfgList,
#else
 .p_claHandle               = NULL,
#endif
};

#endif  /*__UOMODRI_HAL_CONFIG_HANDLERS_H__*/
