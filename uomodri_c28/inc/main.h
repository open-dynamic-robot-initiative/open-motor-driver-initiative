#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include <stdint.h>
#include <string.h>
#include <hw_types.h>

#include "config/motor/tmotor_mn4004_kv300.h"           // T-MOTOR MN4000 KV300 motor definitions
#include "config/motor/myactuator_rmdx8h.h"             // MYACTUATOR RMDX8H motor definitions
#include "config/encoder/broadcom_aedt_9810_z00.h"      // BROADCOM AEDT-9810-Z00 encoder definitions
#include "config/encoder/cui_amt112s_4096_5000_s.h"     // CUI AMT112S-4096-5000-s encoder definitions

#include "uomodri_user_defines.h"
#include "config/uomodri_interrupts.h"
#include "config/uomodri_math.h"

#if (USE_UOMODRI_REV == USE_UOMODRI_V1)
//
#elif (USE_UOMODRI_REV == USE_UOMODRI_V2)
//
#elif (USE_UOMODRI_REV == USE_UOMODRI_V3)
#include "config/uomodri_3_1/uomodri_3_1_hal_defines.h"    // uOmodri v3.1 definitions
#include "config/uomodri_3_1/uomodri_3_1_handlers.h"   // uOmodri v3.1 configuration include file
#elif (USE_UOMODRI_REV == USE_UOMODRI_SA_V1)
//
#else
#error "@ least one config must be set @ a time."
#endif



/***********************************************************************
 * ENUM TYPES
 ***********************************************************************/

/***********************************************************************
 * UNION TYPES
 ***********************************************************************/
typedef union __uart_addr_u__
{
    uint8_t             addr;
    struct
    {
        uint8_t         addr_0          : 1;    /*!< Bit 0      : R/W - Address bit 0. */
        uint8_t         addr_1          : 1;    /*!< Bit 1      : R/W - Address bit 1. */
        uint8_t         addr_2          : 1;    /*!< Bit 2      : R/W - Address bit 2. */
    };
} uart_addr_u;

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
//#pragma DATA_ALIGN(status_tmp_msg, 4)
//#pragma DATA_SECTION(status_tmp_msg, "ramgs1")
//slv2mst_msg_t   status_tmp_msg;

#if (defined DEBUG) && (CM_CORE_ENABLE) && (0)
#pragma DATA_SECTION(cpu2cm_dbg_msg, "MSGRAM_CPU_TO_CM");
com_cpu_2_cm_t  cpu2cm_dbg_msg;
#endif

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
//__attribute__((interrupt)) void adc_isrMotor1(void);
//__attribute__((interrupt)) void adc_isrMotor2(void);

#endif /* __MAIN_H__ */
