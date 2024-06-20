#ifndef __UOMODRI_DRV_H__
#define __UOMODRI_DRV_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
//#include "f2838x_cla_typedefs.h"  // F2838x CLA Type definitions
//#include "f2838x_device.h"        // F2838x Headerfile Include File
//#include "f2838x_examples.h"      // F2838x Examples Include File

//#include "f2838x_device.h"
//#include "uomodri_3_1/uomodri_3_1_hal_defines.h"

#include "uomodri_user_defines.h"
#include "drv8353.h"

/***********************************************************************
 * VARIABLES
 ***********************************************************************/
/** @var    drv_cfg_t   drv_cfg[]
 *  @brief  List all IOs \& SPI peripherals used for DRV communication.
 */
#pragma DATA_SECTION(drv_cfg, "ramgs0")
static const drv_cfg_t  drv_cfg[] =
{
 {
  .spiHandle                = DRV_SPI_BASE,
  .gpioNumber_CS            = MOTOR1_DRV_SPI_CS_NUM,
  .gpioNumber_EN            = MOTOR1_DRV_EN_NUM,
  .gpioNumber_FAULT         = MOTOR1_DRV_FAULT_NUM,
 },
#if (USE_DUAL_AXIS_CONFIG == true)
 {
  .spiHandle                = DRV_SPI_BASE,
  .gpioNumber_CS            = MOTOR2_DRV_SPI_CS_NUM,
  .gpioNumber_EN            = MOTOR2_DRV_EN_NUM,
  .gpioNumber_FAULT         = MOTOR2_DRV_FAULT_NUM,
 },
#endif
};

/** @var    drv_reg_t   drv_reg[]
 *  @brief  List all configuration registers for each DRVs.
 */
#pragma DATA_SECTION(drv_reg, "ramgs0")
static drv_reg_t drv_reg[] =
{
 {
  .ctrl_reg_02.CLR_FLT      = false,
  .ctrl_reg_02.BRAKE        = false,
  .ctrl_reg_02.COAST        = false,
  .ctrl_reg_02.PWM1_DIR     = false,
  .ctrl_reg_02.PWM1_COM     = false,
  .ctrl_reg_02.PWM_MODE     = DRV_PWMMODE_6,
  .ctrl_reg_02.OTW_REP      = true, // Over Temperature Warning will raise nFault
  .ctrl_reg_02.DIS_GDF      = false,
  .ctrl_reg_02.DIS_GDUV     = false,
  .ctrl_reg_02.OCP_ACT      = true,
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.IDRIVEN_HS   = DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.IDRIVEP_HS   = DRV_ISOUR_HS_0P150_A,
  .ctrl_reg_03.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.IDRIVEN_LS   = DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.IDRIVEP_LS   = DRV_ISOUR_LS_0P150_A,
  .ctrl_reg_04.TDRIVE       = DRV_TSOUR_1000_NS,
  .ctrl_reg_04.CBC          = false,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.VDS_LVL      = DRV_VDS_LVL_0P080_V,
  .ctrl_reg_05.OCP_DEG      = DRV_OCPDEG_4_US,
  .ctrl_reg_05.OCP_MODE     = DRV_LATCHED_SHUTDOWN,
  .ctrl_reg_05.DEAD_TIME    = DRV_DEADTIME_50_NS,
  .ctrl_reg_05.TRETRY       = true,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.SEN_LVL      = DRV_SENLVL_0P25_V,
  .ctrl_reg_06.CSA_CAL_C    = false,
  .ctrl_reg_06.CSA_CAL_B    = false,
  .ctrl_reg_06.CSA_CAL_A    = false,
  .ctrl_reg_06.DIS_SEN      = false,
  .ctrl_reg_06.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.LS_REF       = false,
  .ctrl_reg_06.VREF_DIV     = true,
  .ctrl_reg_06.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
  .ctrl_reg_07.CAL_MODE     = false,
 },
#if (USE_DUAL_AXIS_CONFIG == true)
 {
  .ctrl_reg_02.CLR_FLT      = false,
  .ctrl_reg_02.BRAKE        = false,
  .ctrl_reg_02.COAST        = false,
  .ctrl_reg_02.PWM1_DIR     = false,
  .ctrl_reg_02.PWM1_COM     = false,
  .ctrl_reg_02.PWM_MODE     = DRV_PWMMODE_6,
  .ctrl_reg_02.OTW_REP      = true, // Over Temperature Warning will raise nFault
  .ctrl_reg_02.DIS_GDF      = false,
  .ctrl_reg_02.DIS_GDUV     = false,
  .ctrl_reg_02.OCP_ACT      = true,
  // Gate Drive HS register initialization (Address: 0x3)
  .ctrl_reg_03.IDRIVEN_HS   = DRV_ISINK_HS_0P600_A,//DRV_ISINK_HS_0P300_A,
  .ctrl_reg_03.IDRIVEP_HS   = DRV_ISOUR_HS_0P300_A,//DRV_ISOUR_HS_0P150_A,
  .ctrl_reg_03.LOCK         = DRV_REG_UNLOCK,
  // Gate Drive LS register initialization (Address: 0x4)
  .ctrl_reg_04.IDRIVEN_LS   = DRV_ISINK_LS_0P600_A,//DRV_ISINK_LS_0P300_A,
  .ctrl_reg_04.IDRIVEP_LS   = DRV_ISOUR_LS_0P300_A,//DRV_ISOUR_LS_0P150_A,
  .ctrl_reg_04.TDRIVE       = DRV_TSOUR_1000_NS,
  // Over Current Protection register initialization (Address: 0x5)
  .ctrl_reg_05.VDS_LVL      = DRV_VDS_LVL_0P080_V,
  .ctrl_reg_05.OCP_DEG      = DRV_OCPDEG_4_US,
  .ctrl_reg_05.OCP_MODE     = DRV_LATCHED_SHUTDOWN,
  .ctrl_reg_05.DEAD_TIME    = DRV_DEADTIME_50_NS,
  .ctrl_reg_05.TRETRY       = true,
  // Current Sense Amplifier register initialization (Address: 0x6)
  .ctrl_reg_06.SEN_LVL      = DRV_SENLVL_0P25_V,
  .ctrl_reg_06.CSA_CAL_C    = false,
  .ctrl_reg_06.CSA_CAL_B    = false,
  .ctrl_reg_06.CSA_CAL_A    = false,
  .ctrl_reg_06.DIS_SEN      = false,
  .ctrl_reg_06.CSA_GAIN     = DRV_CSA_GAIN_REG,
  .ctrl_reg_06.LS_REF       = false,
  .ctrl_reg_06.VREF_DIV     = true,
  .ctrl_reg_06.CSA_FET      = false,
  // Amplifier calibration calibration routine (Address: 0x7)
  .ctrl_reg_07.CAL_MODE     = false,
 },
#endif
};

#endif  /* __UOMODRI_DRV_H__ */
