/*
 * File name: hal.c
 * Description: Source file containing CPU2 Hardware Abstraction Layer source code
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <string.h>
#include "device.h"

#include "hal.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
void HAL_FSI_ini(const fsi_cfg_t*);

/**
 * @brief       Initialize all peripherals necessaries.
 * @param[in]   *p_hal  Pointer on the general HAL structure definition
 */
void HAL_ini(const hal_cfg_t* p_hal)
{
    if(p_hal != NULL)
    {

        EALLOW;
        HAL_FSI_ini(p_hal->p_fsiHandle);
        EDIS;
    }

    return;
}

/**
 * @brief       FSI module initialization function
 * @param[in]   *p_fsiCfg Pointer to the FSI configuration structure
 */
void HAL_FSI_ini(const fsi_cfg_t* p_fsiCfg)
{
    // TODO(rafael): Skew compensation?
    // TODO(rafael): Interrupt configuration?

    if(!p_fsiCfg)
        return;

    while (p_fsiCfg->fsiBase != UINT32_MAX)
    {
        // Receiver module:
        if (p_fsiCfg->direction == FSI_RX_DIR)
        {
            if (!p_fsiCfg->loopback)
            {
                // Disable FSI internal loopback (internal TX->RX):
                FSI_disableRxInternalLoopback(p_fsiCfg->fsiBase);
            }

            FSI_resetRxModule(p_fsiCfg->fsiBase, FSI_RX_MAIN_CORE_RESET);
            DEVICE_DELAY_US(1);
            FSI_clearRxModuleReset(p_fsiCfg->fsiBase, FSI_RX_MAIN_CORE_RESET);

            // Initialize receiver and set data width for transmissions:
            FSI_performRxInitialization(p_fsiCfg->fsiBase);
            FSI_setRxDataWidth(p_fsiCfg->fsiBase, p_fsiCfg->dataWidth);

            // Interrupt configuration:
            const fsi_evt_cfg_t* evtCfg = p_fsiCfg->eventInterrupts;
            while (evtCfg->event != UINT16_MAX)
            {
                FSI_enableRxInterrupt(p_fsiCfg->fsiBase, evtCfg->intNum, evtCfg->event);
                evtCfg++;
            }
        }

        // Transmitter module:
        else
        {
            FSI_performTxInitialization(p_fsiCfg->fsiBase, p_fsiCfg->clkPrescaler);
            FSI_setTxDataWidth(p_fsiCfg->fsiBase, p_fsiCfg->dataWidth);

            const fsi_evt_cfg_t* evtCfg = p_fsiCfg->eventInterrupts;
            while (evtCfg->event != UINT16_MAX)
            {
                FSI_enableTxInterrupt(p_fsiCfg->fsiBase, evtCfg->intNum, evtCfg->event);
                evtCfg++;
            }
        }

        p_fsiCfg++;
    }

    return;
}

