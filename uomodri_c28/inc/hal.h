/*
 * Filename: hal.h
 * Description: Header file for the Hardware Abstraction Layer
 */

#ifndef __HAL_H__
#define __HAL_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "f2838x_device.h"
#include "device.h"

#include "uomodri_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
// Specify which core is used to manage the MCAN.
#define MCAN_ALLOCATE_ON_CPU1_CORE              (0U)
#define MCAN_ALLOCATE_ON_CM_CORE                (1U)
// Enable/Disable the loopback mode
#define MCAN_LOOPBACK_MODE_CTRL_DISABLE         (0U)
#define MCAN_LOOPBACK_MODE_CTRL_ENABLE          (1U)
// Remote/data Transmission Request
#define MCAN_RTR_TX_DATA_FRAME                  (0U)
#define MCAN_RTR_TX_REMOTE_FRAME                (1U)
// Error State Indicator
#define MCAN_ESI_PASSIVE_ERROR_FLAG             (0U)
#define MCAN_ESI_TX_RECESSIVE                   (1U)

// MCAN initialization
// Enable/Disable the CAN-FD (CAN with Flexible Data-Rate) mode.
#define MCAN_FLEXIBLE_DATA_RATE_MODE_DISABLE    (0U)
#define MCAN_FLEXIBLE_DATA_RATE_MODE_ENABLE     (1U)
// Use single or dual data rate mode (after arbitration)
#define MCAN_BIT_RATE_SWITCHING_MODE_DISABLE    (0U)
#define MCAN_BIT_RATE_SWITCHING_MODE_ENABLE     (1U)
// Enable/Disable transmit pause during retry.
#define MCAN_TX_PAUSE_DISABLE                   (0U)
#define MCAN_TX_PAUSE_ENABLE                    (1U)
// Enable/Disable Edge Filtering during Bus Integration
#define MCAN_EDGE_FILTERING_4_HARD_SYNC_DISABLE (0U)
#define MCAN_EDGE_FILTERING_4_HARD_SYNC_ENABLE  (1U)
// Enable/Disable protocol exception handling.
#define MCAN_PROTOCOL_EXCEPTION_HANDLE_ENABLE   (0U)
#define MCAN_PROTOCOL_EXCEPTION_HANDLE_DISABLE  (1U)
// Enable/Disable automatic retransmission of messages not transmitted successfully.
#define MCAN_AUTOMATIC_RETRANSMIT_ENABLE        (0U)
#define MCAN_AUTOMATIC_RETRANSMIT_DISABLE       (1U)
// Enable/Disable wakeup request.
#define MCAN_WAKEUP_REQUEST_DISABLE             (0U)
#define MCAN_WAKEUP_REQUEST_ENABLE              (1U)
// Enable/Disable auto-Wakeup
#define MCAN_AUTO_WAKEUP_DISABLE                (0U)
#define MCAN_AUTO_WAKEUP_ENABLE                 (1U)
// Enable/Disable emulation/debug suspend mode
#define MCAN_DEBUG_SUSPEND_DISABLE              (0U)
#define MCAN_DEBUG_SUSPEND_ENABLE               (1U)
// Enable/Disable  Transmitter Delay Compensation
#define MCAN_TX_DELAY_COMPENSATION_DISABLE      (0U)
#define MCAN_TX_DELAY_COMPENSATION_ENABLE       (1U)

// MCAN configuration
// Enable/Disable the Bus Monitoring Mode
#define MCAN_BUS_MONITORING_MODE_DISABLE        (0U)
#define MCAN_BUS_MONITORING_MODE_ENABLE         (1U)
// Enable/Disable the Restricted Operation Mode
#define MCAN_NORMAL_OPERATION_MODE_ACTIVE       (0U)
#define MCAN_RESTRICTED_OPERATION_MODE_ACTIVE   (1U)
// Enable/Disable the Timeout Counter
#define MCAN_TIMEOUT_COUNTER_DISABLE            (0U)
#define MCAN_TIMEOUT_COUNTER_ENABLE             (1U)
// Timestamp source selection
#define MCAN_TIMESTAMP_COUNTER_SET_TO_ZERO      (0U)
#define MCAN_TIMESTAMP_COUNTER_INC_TS_PRESCALAR (1U)
#define MCAN_TIMESTAMP_COUNTER_EXTERNAL_USED    (2U)
// Accept/Reject remote Frames messages with 29-bit extended IDs
#define MCAN_ACCEPT_REMOTE_FRAMES_W_EXT_ID      (0U)
#define MCAN_REJECT_REMOTE_FRAMES_W_EXT_ID      (1U)
// Accept/Reject remote Frames messages with 11-bit standard IDs.
#define MCAN_ACCEPT_REMOTE_FRAMES_W_STD_ID      (0U)
#define MCAN_REJECT_REMOTE_FRAMES_W_STD_ID      (1U)
// Accept/Reject Non-matching Frames messages with 29-bit IDs
#define MCAN_ACCEPT_NO_MATCH_EXT_ID_IN_RX_FIFO0 (0U)
#define MCAN_ACCEPT_NO_MATCH_EXT_ID_IN_RX_FIFO1 (1U)
#define MCAN_REJECT_NO_MATCH_EXT_ID             (2U)
// Accept/Reject Non-matching Frames messages with 11-bit IDs
#define MCAN_ACCEPT_NO_MATCH_STD_ID_IN_RX_FIFO0 (0U)
#define MCAN_ACCEPT_NO_MATCH_STD_ID_IN_RX_FIFO1 (1U)
#define MCAN_REJECT_NO_MATCH_STD_ID             (2U)

// MCAN Message RAM Sections configuration
// Tx FIFO/Queue Mode
#define MCAN_TX_FIFO_OP                         (0U)
#define MCAN_TX_QUEUE_OP                        (1U)
// RX FIFO_0 full management - Blocking or overwrite
#define MCAN_RX_FIFO0_BLOCKING_MODE             (0U)
#define MCAN_RX_FIFO0_OVERWRITE_MODE            (1U)
// RX FIFO_1 full management - Blocking or overwrite
#define MCAN_RX_FIFO1_BLOCKING_MODE             (0U)
#define MCAN_RX_FIFO1_OVERWRITE_MODE            (1U)
// MCAN memory space allocation.
#define MCAN_STRUCT_SIZE(x)                     (MCAN_getMsgObjSize(x) * 4U)

#define MCAN_DLC_PAYLOAD_LENGTH_0B              (0U)
#define MCAN_DLC_PAYLOAD_LENGTH_1B              (1U)
#define MCAN_DLC_PAYLOAD_LENGTH_2B              (2U)
#define MCAN_DLC_PAYLOAD_LENGTH_3B              (3U)
#define MCAN_DLC_PAYLOAD_LENGTH_4B              (4U)
#define MCAN_DLC_PAYLOAD_LENGTH_5B              (5U)
#define MCAN_DLC_PAYLOAD_LENGTH_6B              (6U)
#define MCAN_DLC_PAYLOAD_LENGTH_7B              (7U)
#define MCAN_DLC_PAYLOAD_LENGTH_8B              (8U)
#define MCAN_DLC_PAYLOAD_LENGTH_12B             (9U)
#define MCAN_DLC_PAYLOAD_LENGTH_16B             (10U)
#define MCAN_DLC_PAYLOAD_LENGTH_20B             (11U)
#define MCAN_DLC_PAYLOAD_LENGTH_24B             (12U)
#define MCAN_DLC_PAYLOAD_LENGTH_32B             (13U)
#define MCAN_DLC_PAYLOAD_LENGTH_48B             (14U)
#define MCAN_DLC_PAYLOAD_LENGTH_64B             (15U)

//#define MCAN_PAYLOAD_LENGTH_0B                  (0U)
//#define MCAN_PAYLOAD_LENGTH_1B                  (1U)
//#define MCAN_PAYLOAD_LENGTH_2B                  (2U)
//#define MCAN_PAYLOAD_LENGTH_3B                  (3U)
//#define MCAN_PAYLOAD_LENGTH_4B                  (4U)
//#define MCAN_PAYLOAD_LENGTH_5B                  (5U)
//#define MCAN_PAYLOAD_LENGTH_6B                  (6U)
//#define MCAN_PAYLOAD_LENGTH_7B                  (7U)
//#define MCAN_PAYLOAD_LENGTH_8B                  (8U)
//#define MCAN_PAYLOAD_LENGTH_12B                 (12U)
//#define MCAN_PAYLOAD_LENGTH_16B                 (16U)
//#define MCAN_PAYLOAD_LENGTH_20B                 (20U)
//#define MCAN_PAYLOAD_LENGTH_24B                 (24U)
//#define MCAN_PAYLOAD_LENGTH_32B                 (32U)
//#define MCAN_PAYLOAD_LENGTH_48B                 (48U)
//#define MCAN_PAYLOAD_LENGTH_64B                 (64U)

#define MCAN_TX_BUF_0_INDEX                     (0U)
#define MCAN_TX_BUF_1_INDEX                     (1U)
#define MCAN_TX_BUF_2_INDEX                     (2U)
#define MCAN_TX_BUF_3_INDEX                     (3U)
#define MCAN_TX_BUF_4_INDEX                     (4U)
#define MCAN_TX_BUF_5_INDEX                     (5U)
#define MCAN_TX_BUF_6_INDEX                     (6U)
#define MCAN_TX_BUF_7_INDEX                     (7U)
#define MCAN_TX_BUF_8_INDEX                     (8U)
#define MCAN_TX_BUF_9_INDEX                     (9U)
#define MCAN_TX_BUF_10_INDEX                    (10U)
#define MCAN_TX_BUF_11_INDEX                    (11U)
#define MCAN_TX_BUF_12_INDEX                    (12U)
#define MCAN_TX_BUF_13_INDEX                    (13U)
#define MCAN_TX_BUF_14_INDEX                    (14U)
#define MCAN_TX_BUF_15_INDEX                    (15U)
#define MCAN_TX_BUF_16_INDEX                    (16U)
#define MCAN_TX_BUF_17_INDEX                    (17U)
#define MCAN_TX_BUF_18_INDEX                    (18U)
#define MCAN_TX_BUF_19_INDEX                    (19U)
#define MCAN_TX_BUF_20_INDEX                    (20U)
#define MCAN_TX_BUF_21_INDEX                    (21U)
#define MCAN_TX_BUF_22_INDEX                    (22U)
#define MCAN_TX_BUF_23_INDEX                    (23U)
#define MCAN_TX_BUF_24_INDEX                    (24U)
#define MCAN_TX_BUF_25_INDEX                    (25U)
#define MCAN_TX_BUF_26_INDEX                    (26U)
#define MCAN_TX_BUF_27_INDEX                    (27U)
#define MCAN_TX_BUF_28_INDEX                    (28U)
#define MCAN_TX_BUF_29_INDEX                    (29U)
#define MCAN_TX_BUF_30_INDEX                    (30U)
#define MCAN_TX_BUF_31_INDEX                    (31U)

#define MCAN_RX_BUF_0_INDEX                     (0U)
#define MCAN_RX_BUF_1_INDEX                     (1U)
#define MCAN_RX_BUF_2_INDEX                     (2U)
#define MCAN_RX_BUF_3_INDEX                     (3U)
#define MCAN_RX_BUF_4_INDEX                     (4U)
#define MCAN_RX_BUF_5_INDEX                     (5U)
#define MCAN_RX_BUF_6_INDEX                     (6U)
#define MCAN_RX_BUF_7_INDEX                     (7U)
#define MCAN_RX_BUF_8_INDEX                     (8U)
#define MCAN_RX_BUF_9_INDEX                     (9U)
#define MCAN_RX_BUF_10_INDEX                    (10U)
#define MCAN_RX_BUF_11_INDEX                    (11U)
#define MCAN_RX_BUF_12_INDEX                    (12U)
#define MCAN_RX_BUF_13_INDEX                    (13U)
#define MCAN_RX_BUF_14_INDEX                    (14U)
#define MCAN_RX_BUF_15_INDEX                    (15U)
#define MCAN_RX_BUF_16_INDEX                    (16U)
#define MCAN_RX_BUF_17_INDEX                    (17U)
#define MCAN_RX_BUF_18_INDEX                    (18U)
#define MCAN_RX_BUF_19_INDEX                    (19U)
#define MCAN_RX_BUF_20_INDEX                    (20U)
#define MCAN_RX_BUF_21_INDEX                    (21U)
#define MCAN_RX_BUF_22_INDEX                    (22U)
#define MCAN_RX_BUF_23_INDEX                    (23U)
#define MCAN_RX_BUF_24_INDEX                    (24U)
#define MCAN_RX_BUF_25_INDEX                    (25U)
#define MCAN_RX_BUF_26_INDEX                    (26U)
#define MCAN_RX_BUF_27_INDEX                    (27U)
#define MCAN_RX_BUF_28_INDEX                    (28U)
#define MCAN_RX_BUF_29_INDEX                    (29U)
#define MCAN_RX_BUF_30_INDEX                    (30U)
#define MCAN_RX_BUF_31_INDEX                    (31U)
#define MCAN_RX_BUF_32_INDEX                    (32U)
#define MCAN_RX_BUF_33_INDEX                    (33U)
#define MCAN_RX_BUF_34_INDEX                    (34U)
#define MCAN_RX_BUF_35_INDEX                    (35U)
#define MCAN_RX_BUF_36_INDEX                    (36U)
#define MCAN_RX_BUF_37_INDEX                    (37U)
#define MCAN_RX_BUF_38_INDEX                    (38U)
#define MCAN_RX_BUF_39_INDEX                    (39U)
#define MCAN_RX_BUF_40_INDEX                    (40U)
#define MCAN_RX_BUF_41_INDEX                    (41U)
#define MCAN_RX_BUF_42_INDEX                    (42U)
#define MCAN_RX_BUF_43_INDEX                    (43U)
#define MCAN_RX_BUF_44_INDEX                    (44U)
#define MCAN_RX_BUF_45_INDEX                    (45U)
#define MCAN_RX_BUF_46_INDEX                    (46U)
#define MCAN_RX_BUF_47_INDEX                    (47U)
#define MCAN_RX_BUF_48_INDEX                    (48U)
#define MCAN_RX_BUF_49_INDEX                    (49U)
#define MCAN_RX_BUF_50_INDEX                    (50U)
#define MCAN_RX_BUF_51_INDEX                    (51U)
#define MCAN_RX_BUF_52_INDEX                    (52U)
#define MCAN_RX_BUF_53_INDEX                    (53U)
#define MCAN_RX_BUF_54_INDEX                    (54U)
#define MCAN_RX_BUF_55_INDEX                    (55U)
#define MCAN_RX_BUF_56_INDEX                    (56U)
#define MCAN_RX_BUF_57_INDEX                    (57U)
#define MCAN_RX_BUF_58_INDEX                    (58U)
#define MCAN_RX_BUF_59_INDEX                    (59U)
#define MCAN_RX_BUF_60_INDEX                    (60U)
#define MCAN_RX_BUF_61_INDEX                    (61U)
#define MCAN_RX_BUF_62_INDEX                    (62U)
#define MCAN_RX_BUF_63_INDEX                    (63U)

#define MCAN_RX_BUF_0_POS                       (1U << MCAN_RX_BUF_0_INDEX)
#define MCAN_RX_BUF_1_POS                       (1U << MCAN_RX_BUF_1_INDEX)
#define MCAN_RX_BUF_2_POS                       (1U << MCAN_RX_BUF_2_INDEX)
#define MCAN_RX_BUF_3_POS                       (1U << MCAN_RX_BUF_3_INDEX)
#define MCAN_RX_BUF_4_POS                       (1U << MCAN_RX_BUF_4_INDEX)
#define MCAN_RX_BUF_5_POS                       (1U << MCAN_RX_BUF_5_INDEX)
#define MCAN_RX_BUF_6_POS                       (1U << MCAN_RX_BUF_6_INDEX)
#define MCAN_RX_BUF_7_POS                       (1U << MCAN_RX_BUF_7_INDEX)
#define MCAN_RX_BUF_8_POS                       (1U << MCAN_RX_BUF_8_INDEX)
#define MCAN_RX_BUF_9_POS                       (1U << MCAN_RX_BUF_9_INDEX)
#define MCAN_RX_BUF_10_POS                      (1U << MCAN_RX_BUF_10_INDEX)
#define MCAN_RX_BUF_11_POS                      (1U << MCAN_RX_BUF_11_INDEX)
#define MCAN_RX_BUF_12_POS                      (1U << MCAN_RX_BUF_12_INDEX)
#define MCAN_RX_BUF_13_POS                      (1U << MCAN_RX_BUF_13_INDEX)
#define MCAN_RX_BUF_14_POS                      (1U << MCAN_RX_BUF_14_INDEX)
#define MCAN_RX_BUF_15_POS                      (1U << MCAN_RX_BUF_15_INDEX)
#define MCAN_RX_BUF_16_POS                      (1U << MCAN_RX_BUF_16_INDEX)
#define MCAN_RX_BUF_17_POS                      (1U << MCAN_RX_BUF_17_INDEX)
#define MCAN_RX_BUF_18_POS                      (1U << MCAN_RX_BUF_18_INDEX)
#define MCAN_RX_BUF_19_POS                      (1U << MCAN_RX_BUF_19_INDEX)
#define MCAN_RX_BUF_20_POS                      (1U << MCAN_RX_BUF_20_INDEX)
#define MCAN_RX_BUF_21_POS                      (1U << MCAN_RX_BUF_21_INDEX)
#define MCAN_RX_BUF_22_POS                      (1U << MCAN_RX_BUF_22_INDEX)
#define MCAN_RX_BUF_23_POS                      (1U << MCAN_RX_BUF_23_INDEX)
#define MCAN_RX_BUF_24_POS                      (1U << MCAN_RX_BUF_24_INDEX)
#define MCAN_RX_BUF_25_POS                      (1U << MCAN_RX_BUF_25_INDEX)
#define MCAN_RX_BUF_26_POS                      (1U << MCAN_RX_BUF_26_INDEX)
#define MCAN_RX_BUF_27_POS                      (1U << MCAN_RX_BUF_27_INDEX)
#define MCAN_RX_BUF_28_POS                      (1U << MCAN_RX_BUF_28_INDEX)
#define MCAN_RX_BUF_29_POS                      (1U << MCAN_RX_BUF_29_INDEX)
#define MCAN_RX_BUF_30_POS                      (1U << MCAN_RX_BUF_30_INDEX)
#define MCAN_RX_BUF_31_POS                      (1U << MCAN_RX_BUF_31_INDEX)
#define MCAN_RX_BUF_32_POS                      (1U << MCAN_RX_BUF_32_INDEX)
#define MCAN_RX_BUF_33_POS                      (1U << MCAN_RX_BUF_33_INDEX)
#define MCAN_RX_BUF_34_POS                      (1U << MCAN_RX_BUF_34_INDEX)
#define MCAN_RX_BUF_35_POS                      (1U << MCAN_RX_BUF_35_INDEX)
#define MCAN_RX_BUF_36_POS                      (1U << MCAN_RX_BUF_36_INDEX)
#define MCAN_RX_BUF_37_POS                      (1U << MCAN_RX_BUF_37_INDEX)
#define MCAN_RX_BUF_38_POS                      (1U << MCAN_RX_BUF_38_INDEX)
#define MCAN_RX_BUF_39_POS                      (1U << MCAN_RX_BUF_39_INDEX)
#define MCAN_RX_BUF_40_POS                      (1U << MCAN_RX_BUF_40_INDEX)
#define MCAN_RX_BUF_41_POS                      (1U << MCAN_RX_BUF_41_INDEX)
#define MCAN_RX_BUF_42_POS                      (1U << MCAN_RX_BUF_42_INDEX)
#define MCAN_RX_BUF_43_POS                      (1U << MCAN_RX_BUF_43_INDEX)
#define MCAN_RX_BUF_44_POS                      (1U << MCAN_RX_BUF_44_INDEX)
#define MCAN_RX_BUF_45_POS                      (1U << MCAN_RX_BUF_45_INDEX)
#define MCAN_RX_BUF_46_POS                      (1U << MCAN_RX_BUF_46_INDEX)
#define MCAN_RX_BUF_47_POS                      (1U << MCAN_RX_BUF_47_INDEX)
#define MCAN_RX_BUF_48_POS                      (1U << MCAN_RX_BUF_48_INDEX)
#define MCAN_RX_BUF_49_POS                      (1U << MCAN_RX_BUF_49_INDEX)
#define MCAN_RX_BUF_50_POS                      (1U << MCAN_RX_BUF_50_INDEX)
#define MCAN_RX_BUF_51_POS                      (1U << MCAN_RX_BUF_51_INDEX)
#define MCAN_RX_BUF_52_POS                      (1U << MCAN_RX_BUF_52_INDEX)
#define MCAN_RX_BUF_53_POS                      (1U << MCAN_RX_BUF_53_INDEX)
#define MCAN_RX_BUF_54_POS                      (1U << MCAN_RX_BUF_54_INDEX)
#define MCAN_RX_BUF_55_POS                      (1U << MCAN_RX_BUF_55_INDEX)
#define MCAN_RX_BUF_56_POS                      (1U << MCAN_RX_BUF_56_INDEX)
#define MCAN_RX_BUF_57_POS                      (1U << MCAN_RX_BUF_57_INDEX)
#define MCAN_RX_BUF_58_POS                      (1U << MCAN_RX_BUF_58_INDEX)
#define MCAN_RX_BUF_59_POS                      (1U << MCAN_RX_BUF_59_INDEX)
#define MCAN_RX_BUF_60_POS                      (1U << MCAN_RX_BUF_60_INDEX)
#define MCAN_RX_BUF_61_POS                      (1U << MCAN_RX_BUF_61_INDEX)
#define MCAN_RX_BUF_62_POS                      (1U << MCAN_RX_BUF_62_INDEX)
#define MCAN_RX_BUF_63_POS                      (1U << MCAN_RX_BUF_63_INDEX)

#define MCAN_INTR_MASK                          (MCAN_IR_HPM_MASK   | \
                                                 MCAN_IR_TC_MASK    | \
                                                 MCAN_IR_TCF_MASK   | \
                                                 MCAN_IR_TFE_MASK   | \
                                                 MCAN_IR_DRX_MASK   | \
                                                 MCAN_IR_BEC_MASK   | \
                                                 MCAN_IR_BEU_MASK   | \
                                                 MCAN_IR_ELO_MASK   | \
                                                 MCAN_IR_EP_MASK    | \
                                                 MCAN_IR_EW_MASK    | \
                                                 MCAN_IR_BO_MASK    | \
                                                 MCAN_IR_WDI_MASK   | \
                                                 MCAN_IR_PEA_MASK   | \
                                                 MCAN_IR_PED_MASK   | \
                                                 MCAN_IR_ARA_MASK)
#define MCAN_ENABLE_IT                          (true)
#define MCAN_DISABLE_IT                         (false)
#define MCAN_ENABLE_LINE_IT                     (true)
#define MCAN_DISABLE_LINE_IT                    (false)
// Reset IT flag for MCAN
#define MCAN_IT_EXT_TS_CLEAR                    (0x00)
#define MCAN_IT_LINE_0_CLEAR                    (0x01)
#define MCAN_IT_LINE_1_CLEAR                    (0x02)

/***********************************************************************
 * ENUMARATIONS
 ***********************************************************************/
typedef enum __sci_MultiProcCtrl_enum__
{
    SCI_MULTI_PROC_NONE = 0,
    SCI_MULTI_PROC_ADDR = 1,
    SCI_MULTI_PROC_IDLE = 2
} sci_MultiProcCtrl_e;

typedef enum __clb_RS485AddrDef_enum__
{
    RS485_CLB_USE_NO_ADDRESS    = 0,
    RS485_CLB_USE_UID_ADDRESS   = 1,
    RS485_CLB_FORCE_ADDRESS     = 2
} clb_RS485AddrDef_e;

/***********************************************************************
 * HARDWARE ABSTRACTION LAYER (HAL) CONTROL STRUCTURES
 ***********************************************************************/
/**
 * @struct  epwm_tb_t
 * @brief   ePWM - Time Base submodule structure definition.
 */
typedef struct __epwm_time_base_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            phaseCount;                 /*!< Phase counter. Sets offset (min value). */
    uint16_t                            syncOutputMode;             /*!< Output sync signal generation. \see EPWM_SyncOutPulseMode */
    EPWM_SyncInPulseSource              syncInputSrc;               /*!< Select the sync input signal.*/
} epwm_tb_t;

/**
 * @struct  epwm_cc_t
 * @brief   ePWM - Counter Compare submodule structure definition.
 */
typedef struct __epwm_counter_compare_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            compCount;                  /*!< Compare reference register. */
    EPWM_CounterCompareModule           compModule;                 /*!< Compare module (A, B, C or D). \see EPWM_CounterCompareModule */
} epwm_cc_t;

/**
 * @struct  epwm_aq_t
 * @brief   ePWM - Action Qualifier submodule structure definition.
 */
typedef struct __epwm_action_qualifier_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            actionsEvt;                 /*!< Actions associated to events matching detection. \see EPWM_ActionQualifierEventAction */
    EPWM_ActionQualifierOutputModule    epwmOutput;                 /*!< Output channel output (A or B). \see EPWM_ActionQualifierOutputModule */
} epwm_aq_t;

/**
 * @struct  epwm_db_t
 * @brief   ePWM - DeadBand Unit submodule structure definition.
 */
typedef struct __epwm_deadband_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            redCount;                   /*!< Rising Edge Delay counter. */
    uint16_t                            fedCount;                   /*!< Falling Edge Delay counter. */
    bool_t                              DelayMode;                  /*!< Enable deadband unit. */
} epwm_db_t;

/**
 * @struct  epwm_pc_t
 * @brief   ePWM - Chopper submodule structure definition.
 */
typedef struct __epwm_chopper_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            dutyCycleCount;             /*!< Programmable chopping (carrier) frequency */
    uint16_t                            freqDiv;
    uint16_t                            firstPulseWidth;            /*!< Programmable duty cycle of second and subsequent pulses */
} epwm_pc_t;

/**
 * @struct  epwm_et_t
 * @brief   ePWM - ADC SOC events trigger submodule structure definition.
 */
typedef struct __epwm_adc_soc_event_trigger_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            adcSOCEvtCount;             /*!< Delays by N events before generating an ADC SOC event. */
    EPWM_ADCStartOfConversionType       adcSOCType;                 /*!< ADC Start of Conversion (SOC) target \em EPWM_SOC_A or \em EPWM_SOC_B */
    EPWM_ADCStartOfConversionSource     adcSOCSource;               /*!< PWM events source for a ADC SOC. \see EPWM_ADCStartOfConversionSource */
} epwm_et_t;

/**
 * @struct  epwm_it_t
 * @brief   ePWM - Interrupts Events submodule structure definition.
 */
typedef struct __epwm_interrupt_events_t__
{
    uint32_t                            epwmBase;                   /*!< ePWM peripheral address. */
    uint16_t                            intEvtCount;                /*!< Delays by N events before generating an IT. */
    uint16_t                            intSource;                  /*!< PWM events source IT. <em> EPWM_INT_TBCTR_ZERO, _PERIOD, _ZERO_OR_PERIOD, _U_CMPx, _D_CMPx,...</em>  */
} epwm_it_t;

/**
 * @struct  epwm_cfg_t
 * @brief   ePWM global Hardware Abstraction Layer structure
 */
typedef struct __epwm_config_t__
{
    const uint32_t                      epwmBase;                   /*!< ePWM peripheral address. */
    const epwm_tb_t*    const           p_epwmTimeBase;
    const epwm_cc_t*    const           p_epwmCounterCompare;
    const epwm_aq_t*    const           p_epwmActionQualifier;
    const epwm_db_t*    const           p_epwmDeadband;
    const epwm_pc_t*    const           p_epwmProgChopper;
    const epwm_et_t*    const           p_epwmAdcSocEventTrigger;
    const epwm_it_t*    const           p_epwmInterruptsEvent;
    const uint32_t                      epwmAdcSocSrcExtern;
} epwm_cfg_t;

/**
 * @struct  eqep_cfg_t
 * @brief   eQEP global Hardware Abstraction Layer structure
 */
typedef struct __eqep_config_t__
{
    const uint32_t                      eqepBase;                   /*!< eQEP peripheral address. */
    const uint32_t                      eqepMaxResolution;
    const uint32_t                      eqepUnitTimerPeriod;
    const uint16_t                      eqepConfig;
    const EQEP_StrobeSource             eqepStrobeSrc;
} eqep_cfg_t;

/**
 * @struct  spi_cfg_t
 * @brief   SPI global Hardware Abstraction Layer structure
 */
typedef struct __spi_config_t__
{
    const uint32_t                      spiBase;                    /*!< ADC peripheral address. */
    const uint32_t                      bitRate;
    const uint16_t                      dataWidth;
    const SPI_Mode                      mode;
    const SPI_TransferProtocol          protocol;
    const SPI_TxFIFOLevel               txLevel;
    const SPI_RxFIFOLevel               rxLevel;
    const uint16_t                      intSrc;
    const bool_t                        highSpeedMode;
    const bool_t                        intEnable;                  /*!< GPIO can be configured as IT source */
} spi_cfg_t;

/**
 * @struct  sci_cfg_t
 * @brief   SCI/UART global Hardware Abstraction Layer structure
 */
typedef struct __sci_config_t__
{
    uint32_t                            sciBase;                    /*!< ADC peripheral address. */
    uint32_t                            bitRate;
    uint32_t                            sciMode;
    SCI_TxFIFOLevel                     txLevel;
    SCI_RxFIFOLevel                     rxLevel;
    uint32_t                            intSrc;
    bool_t                              intEnable;                  /*!< GPIO can be configured as IT source */
    sci_MultiProcCtrl_e                 multiProcMode;
} sci_cfg_t;

/**
 * @struct  mcan_cfg_t
 * @brief   MCAN (CAN-FD) module configuration handler
 */
typedef struct __mcan_cfg_t__
{
    uint32_t                            mCanBase;
    // Initialize MCAN Init parameters.
    MCAN_InitParams                     initParams;
    // Initialize MCAN Config parameters.
    MCAN_ConfigParams                   configParams;
    // Initialize bit timings.
    MCAN_BitTimingParams                bitTimesParams;
    // Initialize Message RAM Sections Configuration Parameters.
    MCAN_MsgRAMConfigParams             msgRAMConfigParams;
    // Initialize Rx Buffer Configuration parameters.
    MCAN_StdMsgIDFilterElement          stdFilterParams;
} mcan_cfg_t;

/**
 * @struct  dma_cfg_t
 * @brief   DMA global Hardware Abstraction Layer structure
 */
typedef struct __dma_config_t__
{
    const uint32_t                      dmaChBase;
    const void*                         p_srcAddr;
    const void*                         p_dstAddr;
    const uint32_t                      transferSize;
    const uint32_t                      srcWrapSize;
    const uint32_t                      dstWrapSize;
    const uint32_t                      config;
    const uint16_t                      burstSize;
    const int16_t                       srcStep;
    const int16_t                       dstStep;
    const int16_t                       srcWrapStep;
    const int16_t                       dstWrapStep;
    const DMA_Trigger                   trigger;
    const DMA_InterruptMode             intMode;
    const bool_t                        intEnable;
} dma_cfg_t;

/**
 * @struct  cla_cfg_t
 * @brief   CLA global Hardware Abstraction Layer structure - To be updated
 */
typedef struct __cla_config_t__
{
    uint32_t                            progRAM;
    uint32_t                            dataRAM;
    uint16_t                            claTaskFlg;
    CLA_TaskNumber                      claTaskNum;
    CLA_MVECTNumber                     claIntVect;
    CLA_Trigger                         claTrigSrc;
    void                                (*p_claFunc)(void);
//    bool_t                              claEnEoTInt;
    bool_t                              claForceTask;
} cla_cfg_t;

/**
 * @struct  clb_cfg_t
 * @brief   CLB global Hardware Abstraction Layer structure
 */
typedef struct __clb_config_t__
{
    clb_RS485AddrDef_e                  clb_Addr1Spec;
    clb_RS485AddrDef_e                  clb_Addr2Spec;
    uint32_t                            clb_Addr1Force;
    uint32_t                            clb_Addr2Force;
} clb_cfg_t;

/**
 * @struct  gpio_cfg_t
 * @brief   GPIO structure definition for all IOs except analog ones.
 */
typedef struct __gpio_config_t__
{
    const uint32_t                      pinNum;                     /*!< GPIO pin number (GPIO0 - GPIO168). */
    const uint32_t                      fctMux;                     /*!< GPIO configuration functions (peripheral affectation matrix). \see pin_map.h */
    const uint32_t                      setValue;                   /*!< GPIO initial value if defined as output. */
    const uint32_t                      pinType;                    /*!< GPIO pad configuration :
                                                                        - \em GPIO_PIN_TYPE_STD (Push-pull output or floating input),
                                                                        - \em GPIO_PIN_TYPE_PULLUP (Pull-up enable for input),
                                                                        - \em GPIO_PIN_TYPE_INVERT (Invert polarity on input),
                                                                        - \em GPIO_PIN_TYPE_OD (Open-drain on output) */
    const GPIO_CoreSelect               coreSelect;                 /*!< GPIO core dependency.
                                                                        - \em GPIO_CORE_CPU1,
                                                                        - \em GPIO_CORE_CPU1_CLA1,
                                                                        - \em GPIO_CORE_CPU2,
                                                                        - \em GPIO_CORE_CPU2_CLA1 */
    const GPIO_Direction                direction;                  /*!< GPIO direction :
                                                                        - \em GPIO_DIR_MODE_IN,
                                                                        - \em GPIO_DIR_MODE_OUT */
    const GPIO_QualificationMode        samplingInMode;             /*!< GPIO input sampling mode :
                                                                        - \em GPIO_QUAL_SYNC,
                                                                        - \em GPIO_QUAL_3SAMPLE,
                                                                        - \em GPIO_QUAL_6SAMPLE,
                                                                        - \em GPIO_QUAL_ASYNC */
    const GPIO_AnalogMode               analogModeEnable;           /*!< GPIO is dedicated to USB */
    const GPIO_ExternalIntNum           extIntNum;                  /*!< GPIO external interrupt source number
                                                                         - \em GPIO_INT_XINT1,
                                                                         - \em GPIO_INT_XINT2,
                                                                         - \em GPIO_INT_XINT3,
                                                                         - \em GPIO_INT_XINT4,
                                                                         - \em GPIO_INT_XINT5 */
    const GPIO_IntType                  intType;                    /*!< GPIO interrupt event source
                                                                         - \em GPIO_INT_TYPE_FALLING_EDGE,
                                                                         - \em GPIO_INT_TYPE_RISING_EDGE,
                                                                         - \em GPIO_INT_TYPE_BOTH_EDGES */
    const bool_t                        intEnable;                  /*!< GPIO can be configured as IT source */
} gpio_cfg_t;

/**
 * @struct  adc_ini_t
 * @brief   ADC initial configuration (start-of-conversion) handler
 */
typedef struct __adc_initalization_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const ADC_Channel                   adcChannel;                 /*!< ADC sampling channel. */
    const ADC_SOCNumber                 adcSOCNum;                  /*!< ADC Start-Of-Conversion (SOC) number (0-15). */
    const ADC_Trigger                   adcTrigSrc;                 /*!< ADC SOC trigger source. */
} adc_ini_t;

/**
 * @struct  adc_acq_t
 * @brief   ADC acquisition results (base and Post-Processing Block) configuration handler
 */
typedef struct __adc_acquisitions_results_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const uint32_t                      adcResultBase;
    const ADC_SOCNumber                 adcSOCNum;
    const ADC_PPBNumber                 adcPPBNum;
} adc_acq_t;

/**
 * @struct  adc_int_t
 * @brief   ADC Interrupt source configuration handler
 */
typedef struct __adc_interrupts_events_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const ADC_SOCNumber                 intTrig;                    /*!< ADC End-Of-Conversion (EOC) number (0-15). */
    const ADC_IntNumber                 adcIntNum;                  /*!< IT source (0-3) associated to ADC \& SOC number. */
} adc_int_t;

/**
 * @struct  adc_cfg_t
 * @brief   ADC module configuration handler
 */
typedef struct __Aadc_config_t__
{
    const uint32_t                      adcBase;                    /*!< ADC peripheral address. */
    const adc_ini_t*    const           p_adcIni;                   /*!< Pointer on ADC start-of-conversion configuration handler */
    const adc_acq_t*    const           p_adcAcq;                   /*!< Pointer on ADC result configuration handler */
    const adc_int_t*    const           p_adcInt;                   /*!< Pointer on ADC Interrupt source configuration handler */
} adc_cfg_t;

/**
 * @struct  int_cfg_t
 * @brief   Interrupt source configuration handler
 */
typedef struct __interrupts_config_t__
{
    uint32_t                            intNum;
    uint16_t                            intAckGroup;
    void                                (*p_intHandler)(void);
} int_cfg_t;

/**
 * @struct  ipc_cfg_t
 * @brief   IPC module configuration handler
 */
typedef struct __ipc_config_t__
{
    IPC_Type_t                          ipcType;
    uint32_t                            ipcBase;
    volatile IPC_MessageQueue_t*        p_ipcMsgQ;
    uint32_t                            ipcIntMsgQL;
    uint32_t                            ipcIntMsgQR;
    uint32_t                            ipcFlag;
    uint32_t                            ipcInt;
    void                                (*p_ipcIntHandler)(void);
    bool_t                              ipcMsgQEnable;
    bool_t                              ipcIntEnable;
} ipc_cfg_t;

/**
 * @struct  tmr_cfg_t
 * @brief   Timer0/1/2 module configuration handler
 */
typedef struct __cpu_timer_config_t__
{
    uint32_t                            cpuTimerBase;
    uint32_t                            periodCount;                /*!< in uSeconds */
    uint16_t                            prescaler;
    bool_t                              intEnable;
} tmr_cfg_t;

/**
 * @struct  hal_cfg_t
 * @brief   General HAL structure handler for the project
 */
typedef struct __hal_config_t__
{
    const gpio_cfg_t*   const           p_gpioHandle;
    const adc_cfg_t*    const           p_adcHandle;
    const epwm_cfg_t*   const           p_epwmHandle;
    const eqep_cfg_t*   const           p_eqepHandle;
    const sci_cfg_t*    const           p_sciHandle;
    const spi_cfg_t*    const           p_spiHandle;
    const mcan_cfg_t*   const           p_mcanHandle;
    const ipc_cfg_t*    const           p_ipcHandle;
    const dma_cfg_t*    const           p_dmaHandle;
    const tmr_cfg_t*    const           p_timerHandle;
    const cla_cfg_t*    const           p_claHandle;
    const clb_cfg_t*    const           p_clbHandle;
} hal_cfg_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void HAL_ini(const hal_cfg_t*);
void HAL_INT_ini(const int_cfg_t*, void (*)(void));
void HAL_DMA_ini(const dma_cfg_t*);

#endif /* __HAL_H__ */
