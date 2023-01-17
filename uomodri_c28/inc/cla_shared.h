/*
 * spi_cla.h
 *
 *  Created on: Jul 19, 2022
 *      Author: amine
 */

#ifndef INC_CLA_SHARED_H_
#define INC_CLA_SHARED_H_


#ifdef __cplusplus
extern "C" {
#endif
//
//#include "as5047u.h"
#include "hal.h"
#include <stdint.h>

///{all=0x007FFC9A,bit={crc=0x9A,commandFrame=0x3FFC,rw=0x1,dnc=0x0,pad=0x00}



#define ANGLE_COMMAND       (0x007FFEA0)  //bit struct _CommandFrame32_t    {crc=160,commandFrame=16382,rw=1,dnc=0,pad=0} all unsigned long   0x007FFEA0 (Hex)
#define VELOCITY_COMMAND    (0x007FFC9A)  //{all=0x007FFC9A,bit={crc=0x9A,commandFrame=0x3FFC,rw=0x1,dnc=0x0,pad=0x00}

extern float32_t    thetaAbs;
extern float32_t    velocity;
extern bool_t       cla_new_flag;
extern bool_t       cpu1_ready_flag;

uint32_t send32CLA(uint32_t command);
uint16_t send16CLA(uint16_t command);

//
// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();
//bool_t as5047u_crc8check(BitDataFrame32_u);

//void initCLA(void);

#ifdef __cplusplus
}
#endif // extern "C"
#endif //end of INC_CLA_SHARED_H_ definition

