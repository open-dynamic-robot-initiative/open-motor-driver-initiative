/*
 * File name: hal_handlers.h
 * Description: Header file containing CPU2 HAL config structures.
 */

#ifndef __HAL_HANDLERS_H_
#define __HAL_HANDLERS_H_

#include "hal.h"
#include "hal_defines.h"

/***********************************************************************
 * FSI STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const fsi_cfg_t  fsiCfgList[] =
{
 FSI_TX_CPU2_DEF,
 FSI_RX_CPU2_DEF,
 {.fsiBase= UINT32_MAX},           /* END OF ARRAY */
};

/***********************************************************************
 * GLOBAL HAL STRUCTURE DECLARATION & INITIALIZATION
 ***********************************************************************/
static const hal_cfg_t  hal =
{
 .p_fsiHandle   = fsiCfgList,
};

#endif /* HAL_HANDLERS_H_ */
