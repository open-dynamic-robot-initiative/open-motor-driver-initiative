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

            // RX interrupt setup, see `interruptList` for which
            // interrupts are enabled to propagate in the PIE.

            // TODO(rafael): Add necessary interrupts, eg. timeout.
            // FSI_enableRxInterrupt(p_fsiCfg->fsiBase, FSI_INT1, FSI_RX_EVT_FRAME_DONE);
        }

        // Transmitter module:
        else
        {
            FSI_performTxInitialization(p_fsiCfg->fsiBase, p_fsiCfg->clkPrescaler);
            FSI_setTxDataWidth(p_fsiCfg->fsiBase, p_fsiCfg->dataWidth);
            // FSI_enableTxInterrupt(p_fsiCfg->fsiBase, FSI_INT1, FSI_TX_EVT_FRAME_DONE);
        }

        p_fsiCfg++;
    }

    return;
}

