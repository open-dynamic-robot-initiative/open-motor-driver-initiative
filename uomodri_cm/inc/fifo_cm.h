/**
 ******************************************************************************
 * \file    fifo.h
 * \author  Jerome Manhes - CNRS/LAAS
 * \version V1.0
 * \date    April 19, 2018
 * \brief   FIFO memory management.
 ******************************************************************************
 * \attention
 * Copyright (c) 2015-2018 LAAS/CNRS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIFO_H
#define __FIFO_H

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
//#include <string.h>

#include "driverlib_cm.h"
#include "cm.h"
/* Exported types ------------------------------------------------------------*/
/**
  * @brief  First In First Out Buffer (FIFO) - Access status to the FIFO
  */
typedef enum {
  LOCKED    = 0x00,
  UNLOCKED  = 0x01
}AccessFlag;

/**
  * @brief  First In First Out Buffer (FIFO) - Word type access to the FIFO
  */
typedef enum {
  BYTE  = 0x01,
  SHORT = 0x02,
  LONG  = 0x04
} WordType;

/**
  * @brief  First In First Out Buffer (FIFO)  - Functional status of the FIFO
  */
typedef enum {
  EMPTY     = 0x00,
  DATA      = 0x01,
  FULL      = 0x02
} FifoFlag;

/**
 * @brief  First In First Out Buffer (FIFO) - Structure definition
 */
typedef struct {
    void*       p_baseAddr;
    void*       p_maxAddr;
    void*       p_msgRead;
    void*       p_msgWrite;
//    uint32_t    fifoMaxAddr;
    uint16_t    fifoSize;
    uint16_t    elemSize;
    uint16_t    elemNb;
    int16_t     elemCnt;
    FifoFlag    statusFlag;
    AccessFlag  accessFlag;
} Fifo_TypeDef;

/* Exported functions --------------------------------------------------------*/
//void FIFO_counter(Fifo_TypeDef*, int32_t);
void FIFO_init(Fifo_TypeDef*, uint16_t, uint16_t);
//void FIFO_init(Fifo_TypeDef*, WordType, uint16_t);
void FIFO_deInit(Fifo_TypeDef*);
uint16_t FIFO_read(Fifo_TypeDef*, void*);
bool FIFO_write(Fifo_TypeDef*, void*);
//uint16_t FIFO_pop(Fifo_TypeDef*, void*);
//bool FIFO_pop(Fifo_TypeDef*, void*, uint16_t);
//bool FIFO_push(Fifo_TypeDef*, void*);
//bool FIFO_push(Fifo_TypeDef*, void*, uint16_t);

#endif /*__FIFO_H */
