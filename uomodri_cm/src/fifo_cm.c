/**
 ******************************************************************************
 * \file    fifo.c
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

/* Includes ------------------------------------------------------------------*/
#include "fifo_cm.h"

/* Variables declaration -----------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * \brief   Initialize the current \a FIFO structure.
 * \param   *p_fifo     Pointer on a \a Fifo_TypeDef structure containing the elements of the \a FIFO.
 * \param   elem_size   Size of data to be saved into \a FIFO.
 * \param   nb_elem     Number of elements (of elem_size) contained in the \a FIFO.
 */
void FIFO_init(Fifo_TypeDef* p_fifo, uint16_t elem_size, uint16_t nb_elem)
{
    p_fifo->fifoSize    = elem_size * nb_elem;
    p_fifo->elemSize    = elem_size;
    p_fifo->elemNb      = nb_elem;
    p_fifo->elemCnt     = 0;

    p_fifo->p_baseAddr  = malloc(p_fifo->fifoSize);
    p_fifo->p_maxAddr   = (void *)((uint32_t)p_fifo->p_baseAddr + p_fifo->fifoSize);
    p_fifo->p_msgRead   = p_fifo->p_baseAddr;
    p_fifo->p_msgWrite  = p_fifo->p_baseAddr;

    p_fifo->statusFlag  = EMPTY;
    p_fifo->accessFlag  = UNLOCKED;
}

/**
 * \brief   De initialize the current \a FIFO structure.
 * \param   *p_fifo     Pointer on a \a Fifo_TypeDef structure containing the elements of the \a FIFO.
 */
void FIFO_deInit(Fifo_TypeDef* p_fifo)
{
    p_fifo->fifoSize    = 0U;
    p_fifo->elemSize    = 0U;
    p_fifo->elemNb      = 0U;
    p_fifo->elemCnt     = 0;

    if(p_fifo->p_baseAddr != NULL)
        free(p_fifo->p_baseAddr);
    p_fifo->p_maxAddr   = NULL;
    p_fifo->p_msgRead   = NULL;
    p_fifo->p_msgWrite  = NULL;

    p_fifo->statusFlag  = EMPTY;
    p_fifo->accessFlag  = UNLOCKED;
}

/**
 * \brief   Read data from the \a FIFO structure.
 * \param   *p_fifo     Pointer on a \a Fifo_TypeDef structure containing the elements of the \a FIFO.
 * \param   num     Number of elements to be read. Increment of the Read \a FIFO pointer.
 */
uint16_t FIFO_read(Fifo_TypeDef* p_fifo, void* p_dest)
{
    /* Read access flag */
    bool    read_done       = false;

    if((p_fifo->statusFlag != EMPTY) && (p_fifo->accessFlag != LOCKED))
    {
        p_fifo->accessFlag  = LOCKED;
        memcpy(p_dest, p_fifo->p_msgRead, p_fifo->elemSize);
        p_fifo->p_msgRead   = (void *)((uint32_t)p_fifo->p_msgRead + (uint32_t)p_fifo->elemSize);
        if(p_fifo->p_msgRead >= p_fifo->p_maxAddr)
            p_fifo->p_msgRead   = p_fifo->p_baseAddr;
//        p_fifo->p_msgRead   = (p_fifo->p_msgRead >= p_fifo->p_maxAddr)  ? (p_fifo->p_baseAddr)  : (p_fifo->p_msgRead);
        p_fifo->elemCnt--;
        if(p_fifo->elemCnt <= 0)
        {
            p_fifo->elemCnt     = 0;
            p_fifo->statusFlag  = EMPTY;
        }
        else
            p_fifo->statusFlag  = DATA;
//        p_fifo->elemCnt     = ((--p_fifo->elemCnt) <= 0)                ? (0)                   : (p_fifo->elemCnt);
//        p_fifo->statusFlag  = (p_fifo->elemCnt <= 0)                    ? (EMPTY)               : (DATA);
        read_done           = true;
        p_fifo->accessFlag  = UNLOCKED;
    }

    return(read_done);
}

/**
 * \brief   Add data from the \a FIFO structure.
 * \param   *fifo   Pointer on a \a Fifo_TypeDef structure containing the elements of the current \a FIFO.
 * \param   *msg    Pointer on buffered array to save.
 * \param   num     Number of elements to be Written. Increment of the Write \a FIFO pointer.
 */
bool FIFO_write(Fifo_TypeDef* p_fifo, void* p_src)
{
    /* Read access flag */
    bool    read_done       = false;

    if(p_fifo->statusFlag != FULL && p_fifo->accessFlag != LOCKED)
    {
        p_fifo->accessFlag  = LOCKED;
        memcpy(p_fifo->p_msgWrite, p_src, p_fifo->elemSize);
        p_fifo->p_msgWrite  = (void *)((uint32_t)p_fifo->p_msgWrite + (uint32_t)p_fifo->elemSize);
        if(p_fifo->p_msgWrite >= p_fifo->p_maxAddr)
            p_fifo->p_msgWrite  = p_fifo->p_baseAddr;
//        p_fifo->p_msgWrite  = (p_fifo->p_msgWrite >= p_fifo->p_maxAddr) ? (p_fifo->p_baseAddr)  : (p_fifo->p_msgWrite);
        p_fifo->elemCnt++;
        if(p_fifo->elemCnt >= p_fifo->elemNb)
        {
            p_fifo->elemCnt     = p_fifo->elemNb;
            p_fifo->statusFlag  = FULL;
        }
        else
            p_fifo->statusFlag  = DATA;
//        p_fifo->elemCnt     = ((++p_fifo->elemCnt) >= p_fifo->elemNb)   ? (p_fifo->elemNb)      : (p_fifo->elemCnt);
//        p_fifo->statusFlag  = (p_fifo->elemCnt >= p_fifo->elemNb)       ? (FULL)                : (DATA);
        read_done           = true;
        p_fifo->accessFlag  = UNLOCKED;
    }

    return(read_done);
}

