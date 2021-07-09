#ifndef __MAIN_H__
#define __MAIN_H__

// Project includes
#include "device.h"
#include "driverlib.h"
#include "f2838x_device.h"

#include "hal.h"
#include "drv8353.h"
#include "encoder.h"
#include "foc.h"
#include "motor.h"

#include "communication_interface.h"
#include "spi_communication.h"
#include "legacy_spi.h"

#include "uomodri_user_defines.h"
#include "uomodri_hal_config_handlers.h"
#include "uomodri_prj_config_handlers.h"

static void MCU_init();

// ADC interrupt service routine
//__interrupt void adc1ISR(void);
//__interrupt void adc2ISR(void);
//__interrupt void adc3ISR(void);
//__interrupt void adc4ISR(void);
//__interrupt void isrMotor1(void);

#endif // end of MAIN_H definition
