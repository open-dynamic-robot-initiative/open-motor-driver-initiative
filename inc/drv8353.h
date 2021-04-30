#ifndef __DRV8353_H__
#define __DRV8353_H__

// drivers
#include "device.h"
#include "driverlib.h"
#include "hal.h"

// **************************************************************************
// the defines

/*******  Bit definitions for Fault Status 1 register  *******/
#define FLT_STATUS_FAULT_Pos            (10U)
#define FLT_STATUS_FAULT_Msk            (0x1U << FLT_STATUS_FAULT_Pos)
#define FLT_STATUS_FAULT                FLT_STATUS_FAULT_Msk
#define FLT_STATUS_VDS_OCP_Pos          (9U)
#define FLT_STATUS_VDS_OCP_Msk          (0x1U << FLT_STATUS_VDS_OCP_Pos)
#define FLT_STATUS_VDS_OCP              FLT_STATUS_VDS_OCP_Msk
#define FLT_STATUS_GDF_Pos              (8U)
#define FLT_STATUS_GDF_Msk              (0x1U << FLT_STATUS_GDF_Pos)
#define FLT_STATUS_GDF                  FLT_STATUS_GDF_Msk
#define FLT_STATUS_UVLO_Pos             (7U)
#define FLT_STATUS_UVLO_Msk             (0x1U << FLT_STATUS_UVLO_Pos)
#define FLT_STATUS_UVLO                 FLT_STATUS_UVLO_Msk
#define FLT_STATUS_OTSD_Pos             (6U)
#define FLT_STATUS_OTSD_Msk             (0x1U << FLT_STATUS_OTSD_Pos)
#define FLT_STATUS_OTSD                 FLT_STATUS_OTSD_Msk
#define FLT_STATUS_VDS_HA_Pos           (5U)
#define FLT_STATUS_VDS_HA_Msk           (0x1U << FLT_STATUS_VDS_HA_Pos)
#define FLT_STATUS_VDS_HA               FLT_STATUS_VDS_HA_Msk
#define FLT_STATUS_VDS_LA_Pos           (4U)
#define FLT_STATUS_VDS_LA_Msk           (0x1U << FLT_STATUS_VDS_LA_Pos)
#define FLT_STATUS_VDS_LA               FLT_STATUS_VDS_LA_Msk
#define FLT_STATUS_VDS_HB_Pos           (3U)
#define FLT_STATUS_VDS_HB_Msk           (0x1U << FLT_STATUS_VDS_HB_Pos)
#define FLT_STATUS_VDS_HB               FLT_STATUS_VDS_HB_Msk
#define FLT_STATUS_VDS_LB_Pos           (2U)
#define FLT_STATUS_VDS_LB_Msk           (0x1U << FLT_STATUS_VDS_LB_Pos)
#define FLT_STATUS_VDS_LB               FLT_STATUS_VDS_LB_Msk
#define FLT_STATUS_VDS_HC_Pos           (1U)
#define FLT_STATUS_VDS_HC_Msk           (0x1U << FLT_STATUS_VDS_HC_Pos)
#define FLT_STATUS_VDS_HC               FLT_STATUS_VDS_HC_Msk
#define FLT_STATUS_VDS_LC_Pos           (0U)
#define FLT_STATUS_VDS_LC_Msk           (0x1U << FLT_STATUS_VDS_LC_Pos)
#define FLT_STATUS_VDS_LC               FLT_STATUS_VDS_LC_Msk

/*******  Bit definitions for VGS Status 2 register  *********/
#define VGS_STATUS_SA_OC_Pos            (10U)
#define VGS_STATUS_SA_OC_Msk            (0x1U << VGS_STATUS_SA_OC_Pos)
#define VGS_STATUS_SA_OC                VGS_STATUS_SA_OC_Msk
#define VGS_STATUS_SB_OC_Pos            (9U)
#define VGS_STATUS_SB_OC_Msk            (0x1U << VGS_STATUS_SB_OC_Pos)
#define VGS_STATUS_SB_OC                VGS_STATUS_SB_OC_Msk
#define VGS_STATUS_SC_OC_Pos            (8U)
#define VGS_STATUS_SC_OC_Msk            (0x1U << VGS_STATUS_SC_OC_Pos)
#define VGS_STATUS_SC_OC                VGS_STATUS_SC_OC_Msk
#define VGS_STATUS_OTW_Pos              (7U)
#define VGS_STATUS_OTW_Msk              (0x1U << VGS_STATUS_OTW_Pos)
#define VGS_STATUS_OTW                  VGS_STATUS_OTW_Msk
#define VGS_STATUS_GDUV_Pos             (6U)
#define VGS_STATUS_GDUV_Msk             (0x1U << VGS_STATUS_GDUV_Pos)
#define VGS_STATUS_GDUV                 VGS_STATUS_GDUV_Msk
#define VGS_STATUS_VGS_HA_Pos           (5U)
#define VGS_STATUS_VGS_HA_Msk           (0x1U << VGS_STATUS_VGS_HA_Pos)
#define VGS_STATUS_VGS_HA               VGS_STATUS_VGS_HA_Msk
#define VGS_STATUS_VGS_LA_Pos           (4U)
#define VGS_STATUS_VGS_LA_Msk           (0x1U << VGS_STATUS_VGS_LA_Pos)
#define VGS_STATUS_VGS_LA               VGS_STATUS_VGS_LA_Msk
#define VGS_STATUS_VGS_HB_Pos           (3U)
#define VGS_STATUS_VGS_HB_Msk           (0x1U << VGS_STATUS_VGS_HB_Pos)
#define VGS_STATUS_VGS_HB               VGS_STATUS_VGS_HB_Msk
#define VGS_STATUS_VGS_LB_Pos           (2U)
#define VGS_STATUS_VGS_LB_Msk           (0x1U << VGS_STATUS_VGS_LB_Pos)
#define VGS_STATUS_VGS_LB               VGS_STATUS_VGS_LB_Msk
#define VGS_STATUS_VGS_HC_Pos           (1U)
#define VGS_STATUS_VGS_HC_Msk           (0x1U << VGS_STATUS_VGS_HC_Pos)
#define VGS_STATUS_VGS_HC               VGS_STATUS_VGS_HC_Msk
#define VGS_STATUS_VGS_LC_Pos           (0U)
#define VGS_STATUS_VGS_LC_Msk           (0x1U << VGS_STATUS_VGS_LC_Pos)
#define VGS_STATUS_VGS_LC               VGS_STATUS_VGS_LC_Msk

// **************************************************************************
// Typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum
{
    DRV8353_CTRLMODE_READ   = 1,        //!< Read Mode
    DRV8353_CTRLMODE_WRITE  = 0         //!< Write Mode
} DRV8353_CtrlMode_e;

//! \brief Enumeration for the driver PWM mode
//!
typedef enum
{
    DRV8353_PWMMODE_6 = 0,              //!< PWM_MODE = 6 inputs
    DRV8353_PWMMODE_3 = 1,              //!< PWM_MODE = 3 inputs
    DRV8353_PWMMODE_1 = 2               //!< PWM_MODE = 1 input
} DRV8353_CTRL02_PWMMode_e;

//! \brief Enumeration for the high side gate drive peak source current;
//!
typedef enum
{
    DRV8353_ISOUR_HS_0P050_A = 0,       //!< IDRIVEP_HS = 0.050A
    DRV8353_ISOUR_HS_0P051_A = 1,       //!< IDRIVEP_HS = 0.050A
    DRV8353_ISOUR_HS_0P100_A = 2,       //!< IDRIVEP_HS = 0.100A
    DRV8353_ISOUR_HS_0P150_A = 3,       //!< IDRIVEP_HS = 0.150A
    DRV8353_ISOUR_HS_0P300_A = 4,       //!< IDRIVEP_HS = 0.300A
    DRV8353_ISOUR_HS_0P350_A = 5,       //!< IDRIVEP_HS = 0.350A
    DRV8353_ISOUR_HS_0P400_A = 6,       //!< IDRIVEP_HS = 0.400A
    DRV8353_ISOUR_HS_0P450_A = 7,       //!< IDRIVEP_HS = 0.450A
    DRV8353_ISOUR_HS_0P550_A = 8,       //!< IDRIVEP_HS = 0.550A
    DRV8353_ISOUR_HS_0P600_A = 9,       //!< IDRIVEP_HS = 0.600A
    DRV8353_ISOUR_HS_0P650_A = 10,      //!< IDRIVEP_HS = 0.650A
    DRV8353_ISOUR_HS_0P700_A = 11,      //!< IDRIVEP_HS = 0.700A
    DRV8353_ISOUR_HS_0P850_A = 12,      //!< IDRIVEP_HS = 0.850A
    DRV8353_ISOUR_HS_0P900_A = 13,      //!< IDRIVEP_HS = 0.900A
    DRV8353_ISOUR_HS_0P950_A = 14,      //!< IDRIVEP_HS = 0.950A
    DRV8353_ISOUR_HS_1P000_A = 15       //!< IDRIVEP_HS = 1.000A
} DRV8353_CTRL03_PeakSourCurHS_e;

//! \brief Enumeration for the high side gate drive peak sink current;
//!
typedef enum
{
    DRV8353_ISINK_HS_0P100_A = 0,       //!< IDRIVEN_HS = 0.100A
    DRV8353_ISINK_HS_0P101_A = 1,       //!< IDRIVEN_HS = 0.100A
    DRV8353_ISINK_HS_0P200_A = 2,       //!< IDRIVEN_HS = 0.200A
    DRV8353_ISINK_HS_0P300_A = 3,       //!< IDRIVEN_HS = 0.300A
    DRV8353_ISINK_HS_0P600_A = 4,       //!< IDRIVEN_HS = 0.600A
    DRV8353_ISINK_HS_0P700_A = 5,       //!< IDRIVEN_HS = 0.700A
    DRV8353_ISINK_HS_0P800_A = 6,       //!< IDRIVEN_HS = 0.800A
    DRV8353_ISINK_HS_0P900_A = 7,       //!< IDRIVEN_HS = 0.900A
    DRV8353_ISINK_HS_1P100_A = 8,       //!< IDRIVEN_HS = 1.100A
    DRV8353_ISINK_HS_1P200_A = 9,       //!< IDRIVEN_HS = 1.200A
    DRV8353_ISINK_HS_1P300_A = 10,      //!< IDRIVEN_HS = 1.300A
    DRV8353_ISINK_HS_1P400_A = 11,      //!< IDRIVEN_HS = 1.400A
    DRV8353_ISINK_HS_1P700_A = 12,      //!< IDRIVEN_HS = 1.700A
    DRV8353_ISINK_HS_1P800_A = 13,      //!< IDRIVEN_HS = 1.800A
    DRV8353_ISINK_HS_1P900_A = 14,      //!< IDRIVEN_HS = 1.900A
    DRV8353_ISINK_HS_2P000_A = 15,      //!< IDRIVEN_HS = 2.000A
} DRV8353_CTRL03_PeakSinkCurHS_e;

//! \brief Enumeration for LOCK and UNLOCK write access sequence commands
//!
typedef enum
{
    DRV8353_REG_LOCK    = 6,            //!< Lock settings
    DRV8353_REG_UNLOCK  = 3             //!< Unlock settings
} DRV8353_CTRL03_Lock_e;

//! \brief Enumeration for the high side and low side gate drive peak source
//! time; adapt timings to DRV8353
//!
typedef enum
{
    DRV8353_TSOUR_500_NS  = 0,          //!< TDRIVE = 500ns
    DRV8353_TSOUR_1000_NS = 1,          //!< TDRIVE = 1000ns
    DRV8353_TSOUR_2000_NS = 2,          //!< TDRIVE = 2000ns
    DRV8353_TSOUR_4000_NS = 3           //!< TDRIVE = 4000ns
} DRV8353_CTRL04_PeakTime_e;

//! \brief Enumeration for the low side gate drive peak source current;
//!  adapt current ratings
//!
typedef enum
{
    DRV8353_ISOUR_LS_0P050_A = 0,       //!< IDRIVEP_LS = 0.050A
    DRV8353_ISOUR_LS_0P051_A = 1,       //!< IDRIVEP_LS = 0.050A
    DRV8353_ISOUR_LS_0P100_A = 2,       //!< IDRIVEP_LS = 0.100A
    DRV8353_ISOUR_LS_0P150_A = 3,       //!< IDRIVEP_LS = 0.150A
    DRV8353_ISOUR_LS_0P300_A = 4,       //!< IDRIVEP_LS = 0.300A
    DRV8353_ISOUR_LS_0P350_A = 5,       //!< IDRIVEP_LS = 0.350A
    DRV8353_ISOUR_LS_0P400_A = 6,       //!< IDRIVEP_LS = 0.400A
    DRV8353_ISOUR_LS_0P450_A = 7,       //!< IDRIVEP_LS = 0.450A
    DRV8353_ISOUR_LS_0P550_A = 8,       //!< IDRIVEP_LS = 0.550A
    DRV8353_ISOUR_LS_0P600_A = 9,       //!< IDRIVEP_LS = 0.600A
    DRV8353_ISOUR_LS_0P650_A = 10,      //!< IDRIVEP_LS = 0.650A
    DRV8353_ISOUR_LS_0P700_A = 11,      //!< IDRIVEP_LS = 0.700A
    DRV8353_ISOUR_LS_0P850_A = 12,      //!< IDRIVEP_LS = 0.850A
    DRV8353_ISOUR_LS_0P900_A = 13,      //!< IDRIVEP_LS = 0.900A
    DRV8353_ISOUR_LS_0P950_A = 14,      //!< IDRIVEP_LS = 0.950A
    DRV8353_ISOUR_LS_1P000_A = 15       //!< IDRIVEP_LS = 1.000A
} DRV8353_CTRL04_PeakSourCurLS_e;

//! \brief Enumeration for the low side gate drive peak sink current;
//!  adapt current ratings
//!
typedef enum
{
    DRV8353_ISINK_LS_0P100_A = 0,       //!< IDRIVEN_LS = 0.100A
    DRV8353_ISINK_LS_0P101_A = 1,       //!< IDRIVEN_LS = 0.100A
    DRV8353_ISINK_LS_0P200_A = 2,       //!< IDRIVEN_LS = 0.200A
    DRV8353_ISINK_LS_0P300_A = 3,       //!< IDRIVEN_LS = 0.300A
    DRV8353_ISINK_LS_0P600_A = 4,       //!< IDRIVEN_LS = 0.600A
    DRV8353_ISINK_LS_0P700_A = 5,       //!< IDRIVEN_LS = 0.700A
    DRV8353_ISINK_LS_0P800_A = 6,       //!< IDRIVEN_LS = 0.800A
    DRV8353_ISINK_LS_0P900_A = 7,       //!< IDRIVEN_LS = 0.900A
    DRV8353_ISINK_LS_1P100_A = 8,       //!< IDRIVEN_LS = 1.100A
    DRV8353_ISINK_LS_1P200_A = 9,       //!< IDRIVEN_LS = 1.200A
    DRV8353_ISINK_LS_1P300_A = 10,      //!< IDRIVEN_LS = 1.300A
    DRV8353_ISINK_LS_1P400_A = 11,      //!< IDRIVEN_LS = 1.400A
    DRV8353_ISINK_LS_1P700_A = 12,      //!< IDRIVEN_LS = 1.700A
    DRV8353_ISINK_LS_1P800_A = 13,      //!< IDRIVEN_LS = 1.800A
    DRV8353_ISINK_LS_1P900_A = 14,      //!< IDRIVEN_LS = 1.900A
    DRV8353_ISINK_LS_2P000_A = 15       //!< IDRIVEN_LS = 2.000A
} DRV8353_CTRL04_PeakSinkCurLS_e;

//! \brief Enumeration for the VDS comparator threshold
//!
typedef enum
{
    DRV8353_VDS_LVL_0P060_V = 0,        //!< VDS_LEVEL = 0.060V
    DRV8353_VDS_LVL_0P070_V = 1,        //!< VDS_LEVEL = 0.070V
    DRV8353_VDS_LVL_0P080_V = 2,        //!< VDS_LEVEL = 0.080V
    DRV8353_VDS_LVL_0P090_V = 3,        //!< VDS_LEVEL = 0.090V
    DRV8353_VDS_LVL_0P100_V = 4,        //!< VDS_LEVEL = 0.100V
    DRV8353_VDS_LVL_0P200_V = 5,        //!< VDS_LEVEL = 0.200V
    DRV8353_VDS_LVL_0P300_V = 6,        //!< VDS_LEVEL = 0.300V
    DRV8353_VDS_LVL_0P400_V = 7,        //!< VDS_LEVEL = 0.400V
    DRV8353_VDS_LVL_0P500_V = 8,        //!< VDS_LEVEL = 0.500V
    DRV8353_VDS_LVL_0P600_V = 9,        //!< VDS_LEVEL = 0.600V
    DRV8353_VDS_LVL_0P700_V = 10,       //!< VDS_LEVEL = 0.700V
    DRV8353_VDS_LVL_0P800_V = 11,       //!< VDS_LEVEL = 0.800V
    DRV8353_VDS_LVL_0P900_V = 12,       //!< VDS_LEVEL = 0.900V
    DRV8353_VDS_LVL_1P000_V = 13,       //!< VDS_LEVEL = 1.000V
    DRV8353_VDS_LVL_1P500_V = 14,       //!< VDS_LEVEL = 1.500V
    DRV8353_VDS_LVL_2P000_V = 15        //!< VDS_LEVEL = 2.000V
} DRV8353_CTRL05_VDSLVL_e;

//! \brief Enumeration for the OCP/VDS sense deglitch time;
//!  adapt deglitch time comments
//!
typedef enum
{
    DRV8353_OCPDEG_1_US = 0,            //!< OCP_DEG = 1us
    DRV8353_OCPDEG_2_US = 1,            //!< OCP_DEG = 2us
    DRV8353_OCPDEG_4_US = 2,            //!< OCP_DEG = 4us
    DRV8353_OCPDEG_8_US = 3             //!< OCP_DEG = 8us
} DRV8353_CTRL05_OcpDeg_e;

//! \brief Enumeration for the OCP report mode
//!
typedef enum
{
    DRV8353_LATCHED_SHUTDOWN = 0,       //!< OCP_MODE = Latched fault
    DRV8353_AUTOMATIC_RETRY  = 1,       //!< OCP_MODE = Automatic Retry
    DRV8353_REPORT_ONLY      = 2,       //!< OCP_MODE = Report only
    DRV8353_DISABLE_OCP      = 3        //!< OCP_MODE = Disabled
} DRV8353_CTRL05_OcpMode_e;

//! \brief Enumeration for the driver dead time
//!
typedef enum
{
    DRV8353_DEADTIME_50_NS  = 0,        //!< DEAD_TIME = 50ns
    DRV8353_DEADTIME_100_NS = 1,        //!< DEAD_TIME = 100ns
    DRV8353_DEADTIME_200_NS = 2,        //!< DEAD_TIME = 200ns
    DRV8353_DEADTIME_400_NS = 3         //!< DEAD_TIME = 400ns
} DRV8353_CTRL05_DeadTime_e;


//! \brief Enumeration for the OCP sense levels threshold
//!
typedef enum
{
    DRV8353_SENLVL_0P25_V = 0,          //!< Sense OCP = 0.25V
    DRV8353_SENLVL_0P50_V = 1,          //!< Sense OCP = 0.50V
    DRV8353_SENLVL_0P75_V = 2,          //!< Sense OCP = 0.75V
    DRV8353_SENLVL_1P00_V = 3           //!< Sense OCP = 1.00V
} DRV8353_CTRL06_SensingLevel_e;

//! \brief Enumeration for the CSA gain
//!
typedef enum
{
    DRV8353_CSA_GAIN_05_V_V = 0,        //!< Sense OCP = 0.25V
    DRV8353_CSA_GAIN_10_V_V = 1,        //!< Sense OCP = 0.50V
    DRV8353_CSA_GAIN_20_V_V = 2,        //!< Sense OCP = 0.75V
    DRV8353_CSA_GAIN_40_V_V = 3         //!< Sense OCP = 1.00V
} DRV8353_CTRL06_CSAGain_e;

//! \brief Enumeration for the register addresses
//!
typedef enum
{
    DRV8353_ADDRESS_STATUS_0  = 0,      //!< Status Register 0
    DRV8353_ADDRESS_STATUS_1  = 1,      //!< Status Register 1
    DRV8353_ADDRESS_CONTROL_2 = 2,      //!< Control Register 2
    DRV8353_ADDRESS_CONTROL_3 = 3,      //!< Control Register 3
    DRV8353_ADDRESS_CONTROL_4 = 4,      //!< Control Register 4
    DRV8353_ADDRESS_CONTROL_5 = 5,      //!< Control Register 5
    DRV8353_ADDRESS_CONTROL_6 = 6,      //!< Control Register 6
    DRV8353_ADDRESS_CONTROL_7 = 7       //!< Control Register 7
} DRV8353_Address_e;

/**
 * @typedef _DRV8353_Stat00_t_
 * @brief   Bit fields for the DRV8353 STATUS00 register
 */
typedef struct _DRV8353_Stat00_t_
{
    uint16_t    VDS_LC      : 1;    /*!< Bits 0 : R/0b - Indicates VDS overcurrent fault on the C low-side MOSFET   */
    uint16_t    VDS_HC      : 1;    /*!< Bits 1 : R/0b - Indicates VDS overcurrent fault on the C high-side MOSFET  */
    uint16_t    VDS_LB      : 1;    /*!< Bits 2 : R/0b - Indicates VDS overcurrent fault on the B low-side MOSFET   */
    uint16_t    VDS_HB      : 1;    /*!< Bits 3 : R/0b - Indicates VDS overcurrent fault on the B high-side MOSFET  */
    uint16_t    VDS_LA      : 1;    /*!< Bits 4 : R/0b - Indicates VDS overcurrent fault on the A low-side MOSFET   */
    uint16_t    VDS_HA      : 1;    /*!< Bits 5 : R/0b - Indicates VDS overcurrent fault on the A high-side MOSFET  */
    uint16_t    OTSD        : 1;    /*!< Bits 6 : R/0b - Indicates overtemperature shutdown                         */
    uint16_t    UVLO        : 1;    /*!< Bits 7 : R/0b - Indicates undervoltage lockout fault condition             */
    uint16_t    GDF         : 1;    /*!< Bits 8 : R/0b - Indicates gate drive fault condition                       */
    uint16_t    VDS_OCP     : 1;    /*!< Bits 9 : R/0b - Indicates VDS monitor overcurrent fault condition          */
    uint16_t    FAULT       : 1;    /*!< Bits 10 : R/0b - Logic OR of FAULT status registers. Mirrors nFAULT pin.   */
    uint16_t    STAT00_RSV1 : 5;    /*!< Bits 15-11 :  R/00000b - Reserved                                          */
} DRV8353_Stat00_t;

/**
 * @typedef DRV8353_Stat01_t
 * @brief   Bit fields for the DRV8353 STATUS01 register
 */
typedef struct _DRV8353_Stat01_t_
{
    uint16_t    VGS_LC      : 1;    /*!< Bits 0 : R/0b - Indicates gate drive fault on the C low-side MOSFET                */
    uint16_t    VGS_HC      : 1;    /*!< Bits 1 : R/0b - Indicates gate drive fault on the C high-side MOSFET               */
    uint16_t    VGS_LB      : 1;    /*!< Bits 2 : R/0b - Indicates gate drive fault on the B low-side MOSFET                */
    uint16_t    VGS_HB      : 1;    /*!< Bits 3 : R/0b - Indicates gate drive fault on the B high-side MOSFET               */
    uint16_t    VGS_LA      : 1;    /*!< Bits 4 : R/0b - Indicates gate drive fault on the A low-side MOSFET                */
    uint16_t    VGS_HA      : 1;    /*!< Bits 5 : R/0b - Indicates gate drive fault on the A high-side MOSFET               */
    uint16_t    GDUV        : 1;    /*!< Bits 6 : R/0b - Indicates VCP charge pump and/or VGLS undervoltage fault condition */
    uint16_t    OTW         : 1;    /*!< Bits 7 : R/0b - Indicates overtemperature warning                                  */
    uint16_t    SC_OC       : 1;    /*!< Bits 8 : R/0b - Indicates overcurrent on phase C sense amplifier (DRV8353xS)       */
    uint16_t    SB_OC       : 1;    /*!< Bits 9 : R/0b - Indicates overcurrent on phase B sense amplifier (DRV8353xS)       */
    uint16_t    SA_OC       : 1;    /*!< Bits 10 : R/0b - Indicates overcurrent on phase A sense amplifier (DRV8353xS)      */
    uint16_t    STAT01_RSV1 : 5;    /*!< Bits 15-11 : R/00000b  - Reserved                                                  */
} DRV8353_Stat01_t;

/**
 * @typedef DRV8353_Ctrl02_t
 * @brief   Bit fields for the DRV8353 CTRL02 register
 */
typedef struct _DRV8353_Ctrl02_t_
{
    uint16_t    CLR_FLT     : 1;    /*!< Bits 0 :   R/W/0b  - Write a 1 to this bit to clear latched fault bits. This bit automatically resets after being written.     */
    uint16_t    BRAKE       : 1;    /*!< Bits 1 :   R/W/0b  - Write a 1 to this bit to turn on all three low-side MOSFETs in 1x PWM mode.                               */
    uint16_t    COAST       : 1;    /*!< Bits 2 :   R/W/0b  - Write a 1 to this bit to put all MOSFETs in the Hi-Z state                                                */
    uint16_t    PWM1_DIR    : 1;    /*!< Bits 3 :   R/W/0b  - In 1x PWM mode this bit is ORed with the INHC (DIR) input                                                 */
    uint16_t    PWM1_COM    : 1;    /*!< Bits 4 :   R/W/0b  - (0b) 1x PWM mode uses sync rectification / (1b) 1x PWM mode uses async rectification (diode freewheeling) */
    uint16_t    PWM_MODE    : 2;    /*!< Bits 6-5 : R/W/00b - (00b) 6x PWM Mode, (01b) 3x PWM mode, (10b) 1x PWM mode, (11b) Independent PWM mode                       */
    uint16_t    OTW_REP     : 1;    /*!< Bits 7 :   R/W/0b  - (0b) OTW is not reported on nFAULT or the FAULT bit / (1b) OTW is reported on nFAULT and the FAULT bit    */
    uint16_t    DIS_GDF     : 1;    /*!< Bits 8 :   R/W/0b  - Gate drive fault is enabled(0b) / disabled(1b)                                                            */
    uint16_t    DIS_GDUV    : 1;    /*!< Bits 9 :   R/W/0b  - VCP and VGLS undervoltage lockout fault is enabled(0b) / disabled(1b)                                     */
    uint16_t    OCP_ACT     : 1;    /*!< Bits 10 :  R/W/0b  - Associated(0b) or all (1b) half-bridge is/are shutdown in response to VDS_OCP and SEN_OCP                 */
    uint16_t    CTRL02_RSV2 : 5;    /*!< Bits 15-11 :  R/W/00000b - Reserved                                                                                            */
} DRV8353_Ctrl02_t;

/**
 * @typedef DRV8353_Ctrl03_t
 * @brief   Bit fields for the DRV8353 CTRL03 register
 */
typedef struct _DRV8353_Ctrl03_t_
{
    uint16_t    IDRIVEN_HS  : 4;    /*!< Bits 3-0   */
    uint16_t    IDRIVEP_HS  : 4;    /*!< Bits 7-4   */
    uint16_t    LOCK        : 3;    /*!< Bits 10-8  */
    uint16_t    CTRL03_RSV1 : 5;    /*!< Bits 15-11 */
} DRV8353_Ctrl03_t;

/**
 * @typedef DRV8353_Ctrl04_t
 * @brief   Bit fields for the DRV8353 CTRL04 register
 */
typedef struct _DRV8353_Ctrl04_t_
{
    uint16_t    IDRIVEN_LS  : 4;    /*!< Bits 3-0   */
    uint16_t    IDRIVEP_LS  : 4;    /*!< Bits 7-4   */
    uint16_t    TDRIVE      : 2;    /*!< Bits 9-8   */
    uint16_t    CBC         : 1;    /*!< Bits 10    */
    uint16_t    CTRL04_RSV1 : 5;    /*!< Bits 15-11 */
} DRV8353_Ctrl04_t;

/**
 * @typedef DRV8353_Ctrl05_t
 * @brief   Bit fields for the DRV8353 CTRL05 register
 */
typedef struct _DRV8353_Ctrl05_t_
{
    uint16_t    VDS_LVL     : 4;    /*!< Bits 3-0   */
    uint16_t    OCP_DEG     : 2;    /*!< Bits 5-4   */
    uint16_t    OCP_MODE    : 2;    /*!< Bits 7-6   */
    uint16_t    DEAD_TIME   : 2;    /*!< Bits 9-8   */
    uint16_t    TRETRY      : 1;    /*!< Bits 10    */
    uint16_t    CTRL05_RSV1 : 5;    /*!< Bits 15-11 */

} DRV8353_Ctrl05_t;

/**
 * @typedef DRV8353_Ctrl06_t
 * @brief   Bit fields for the DRV8353 CTRL06 register
 */
typedef struct _DRV8353_Ctrl06_t_
{
    uint16_t    SEN_LVL     : 2;    /*!< Bits 1-0   */
    uint16_t    CSA_CAL_C   : 1;    /*!< Bits 2     */
    uint16_t    CSA_CAL_B   : 1;    /*!< Bits 3     */
    uint16_t    CSA_CAL_A   : 1;    /*!< Bits 4     */
    uint16_t    DIS_SEN     : 1;    /*!< Bits 5     */
    uint16_t    CSA_GAIN    : 2;    /*!< Bits 7-6   */
    uint16_t    LS_REF      : 1;    /*!< Bits 8     */
    uint16_t    VREF_DIV    : 1;    /*!< Bits 9     */
    uint16_t    CSA_FET     : 1;    /*!< Bits 10    */
    uint16_t    CTRL06_RSV1 : 5;    /*!< Bits 15-11 */
}DRV8353_Ctrl06_t;

/**
 * @typedef DRV8353_Ctrl07_t
 * @brief   Bit fields for the DRV8353 CTRL07 register
 */
typedef struct _DRV8353_Ctrl07_t_
{
    uint16_t    CAL_MODE    : 1;    /*!< Bits 0     */
    uint16_t    CTRL07_RSV1 : 15;   /*!< Bits 15-1  */
}DRV8353_Ctrl07_t;

/**
 * @union   DRV8353_Stat00_u
 * @brief   Object for read/write operations associated with DRV8353 STATUS00 register
 */
typedef union _DRV8353_Stat00_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Stat00_t  stat00;         /*!< @see DRV8353_Stat00_t      */
} DRV8353_Stat00_u;

/**
 * @union   DRV8353_Stat01_u
 * @brief   Object for read/write operations associated with DRV8353 STATUS01 register
 */
typedef union _DRV8353_Stat01_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Stat01_t  stat01;         /*!< @see DRV8353_Stat00_t      */
} DRV8353_Stat01_u;

/**
 * @union   DRV8353_Ctrl02_u
 * @brief   Object for read/write operations associated with DRV8353 CTRL02 register
 */
typedef union _DRV8353_Ctrl02_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Ctrl02_t  ctrl02;         /*!< @see DRV8353_Ctrl02_t      */
} DRV8353_Ctrl02_u;

/**
 * @union   DRV8353_Ctrl03_u
 * @brief   Object for read/write operations associated with DRV8353 CTRL03 register
 */
typedef union _DRV8353_Ctrl03_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Ctrl03_t  ctrl03;         /*!< @see DRV8353_Ctrl03_t      */
} DRV8353_Ctrl03_u;

/**
 * @union   DRV8353_Ctrl04_u
 * @brief   Object for read/write operations associated with DRV8353 CTRL04 register
 */
typedef union _DRV8353_Ctrl04_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Ctrl04_t  ctrl04;         /*!< @see DRV8353_Ctrl04_t      */
} DRV8353_Ctrl04_u;

/**
 * @union   DRV8353_Ctrl05_u
 * @brief   Object for read/write operations associated with DRV8353 CTRL05 register
 */
typedef union _DRV8353_Ctrl05_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Ctrl05_t  ctrl05;         /*!< @see DRV8353_Ctrl05_t      */
} DRV8353_Ctrl05_u;

/**
 * @union   DRV8353_Ctrl06_u
 * @brief   Object for read/write operations associated with DRV8353 CTRL06 register
 */
typedef union _DRV8353_Ctrl06_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Ctrl06_t  ctrl06;         /*!< @see DRV8353_Ctrl06_t      */
} DRV8353_Ctrl06_u;

/**
 * @union   DRV8353_Ctrl07_u
 * @brief   Object for read/write operations associated with DRV8353 CTRL07 register
 */
typedef union _DRV8353_Ctrl07_u_
{
  uint16_t          raw;            /*!< Short (16bits) type access */
  DRV8353_Ctrl07_t  ctrl07;         /*!< @see DRV8353_Ctrl07_t      */
} DRV8353_Ctrl07_u;

/**
 * @typedef DRV8353_Cfg_t
 * @brief   Defines the DRV8353 IO configuration
 */
typedef struct _DRV8353_Cfg_t_
{
    uint32_t  spiHandle;      /*!< handle for the serial peripheral interface       */
    uint32_t  gpioNumber_CS;  /*!< GPIO connected to the DRV8353 CS pin             */
    uint32_t  gpioNumber_EN;  /*!< GPIO connected to the DRV8353 enable pin         */
    bool      rxTimeOut;      /*!< timeout flag for the RX FIFO                     */
    bool      enableTimeOut;  /*!< timeout flag for DRV8353 enable                  */
} DRV8353_Cfg;

/**
 * @typedef DRV8353_Reg_t_
 * @brief   List the status \& command registers associated to the DRV8353
 */
typedef struct _DRV8353_Reg_t_
{
    DRV8353_Stat00_u    Stat_Reg_00;
    DRV8353_Stat01_u    Stat_Reg_01;
    DRV8353_Ctrl02_u    Ctrl_Reg_02;
    DRV8353_Ctrl03_u    Ctrl_Reg_03;
    DRV8353_Ctrl04_u    Ctrl_Reg_04;
    DRV8353_Ctrl05_u    Ctrl_Reg_05;
    DRV8353_Ctrl06_u    Ctrl_Reg_06;
    DRV8353_Ctrl07_u    Ctrl_Reg_07;
} DRV8353_Reg;

typedef struct _DRV8353_t_
{
    DRV8353_Cfg*    drvCfgHandler;
    DRV8353_Reg*    drvRegHandler;
} DRV8353;

/**
 * @typedef DRV8353_msg_t
 * @brief   General structure of a DRV8353 message
 */
typedef struct _DRV8353_msg_
{
    uint16_t    data    : 11;   /*!< Bits 10-0 : According Status registers (0 \& 1) or Control registers (2 -> 6) */
    uint16_t    addr    : 4;    /*!< Bits 14-11 : @see DRV8353_Address_e */
    uint16_t    rw_cmd  : 1;    /*!< Bits 15 : @see DRV8353_CtrlMode_e */
} DRV8353_msg;

/**
 * @typedef DRV8353_msg_u
 * @brief   General structure of a DRV8353 message included in a union typedef.
 */
typedef union _DRV8353_msg_u_ {
  uint16_t      raw;    /*!< Short (16bits) type access */
  DRV8353_msg   fields; /*!< @see DRV8353_msg_t         */
} DRV8353_msg_u;

// **************************************************************************
// Functions prototypes
void DRV8353_init(DRV8353*);
void DRV8353_readStatus(DRV8353_Cfg*, DRV8353_Reg*);
void DRV8353_readAll(DRV8353_Cfg*, DRV8353_Reg*);
void DRV8353_writeAll(DRV8353_Cfg*, DRV8353_Reg*);
uint16_t DRV8353_readOne(DRV8353_Cfg*, DRV8353_Address_e);
void DRV8353_writeOne(DRV8353_Cfg*, DRV8353_Address_e, uint16_t);

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of DRV8353_H definition
