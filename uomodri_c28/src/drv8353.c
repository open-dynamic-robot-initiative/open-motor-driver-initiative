/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
//#include "f2838x_device.h"
//#include "device.h"
#include "config/uomodri_user_defines.h"
#include "drv8353.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static uint32_t DRV_Enable(const drv_cfg_t*);
static bool_t   DRV_SetConfig(const drv_cfg_t*, drv_reg_t*);
static void     DRV_Calibration(const drv_cfg_t*, drv_reg_t*);
uint16_t        DRV_readOne(const drv_cfg_t*, DRV_Address_e);
void            DRV_writeOne(const drv_cfg_t*, DRV_Address_e, uint16_t);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Start \& configure the DRV ICs.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
bool_t DRV_ini(const drv_cfg_t* p_drvCfg, drv_reg_t* p_drvReg)
{
    bool_t drv_err      = true;
    if((p_drvCfg != NULL) && (p_drvReg != NULL))
    {
        if(DRV_Enable(p_drvCfg) == 0)
        {
            if(DRV_SetConfig(p_drvCfg, p_drvReg))
            {
                DRV_Calibration(p_drvCfg, p_drvReg);
                drv_err = false;
            }
        }
    }

    return(drv_err);
}

/**
 * @brief       Enable the DRV IC pin.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 * @return      uint32_t    0 if DRV is enable else DRV is in fault state.
 */
static uint32_t DRV_Enable(const drv_cfg_t* p_drvCfg)
{
    drv_reg_t drv_status        = {.stat_reg_00 = 0, .stat_reg_01 = 0, .ctrl_reg_02 = 0, .ctrl_reg_03 = 0,
                                   .ctrl_reg_04 = 0, .ctrl_reg_05 = 0, .ctrl_reg_06 = 0, .ctrl_reg_07 = 0};
    drv_stat0x_u drv_status_s   = {.all = 0};
    uint16_t cpt_time_out       = 0;
//    uint16_t test_status        = 1;
    bool_t drv_enable           = true;
    // Enable the DRV8353
    GPIO_writePin(p_drvCfg->gpioNumber_EN, 1);
    // Wait 1ms
    DEVICE_DELAY_US(1000U);
    // Make sure the FAULT bit is not set during startup
    while(drv_enable && (DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_0) & FLT_STATUS_FAULT_Msk))
        drv_enable  = (++cpt_time_out < 1000) ? (true) : (false);
//    while((++cpt_time_out < 1000) && DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_0));
//            drv_enable  = (++cpt_time_out < 1000) ? (true) : (false);

    drv_status.stat_reg_00.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_0);
    drv_status.stat_reg_01.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_1);
    drv_status.ctrl_reg_02.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_2);
    drv_status.ctrl_reg_03.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_3);
    drv_status.ctrl_reg_04.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_4);
    drv_status.ctrl_reg_05.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_5);
    drv_status.ctrl_reg_06.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_6);
    drv_status.ctrl_reg_07.all  = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_7);

    //    if((DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_0) & FLT_STATUS_FAULT_Msk) == 0)
//        drv_enable = true;
//    if(DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_0))
//        return (1);

//    test_status = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_2);
//    if (test_status == 0)
//        test_status = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_1);
//    test_status = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_1);
    // Wait for the DRV8353 to go through start up sequence
    DEVICE_DELAY_US(1000U);
//    test_status = GPIO_readPin(p_drvCfg->gpioNumber_FAULT) ;
//    test_status = 1;
    // Check that fault pin is similar as register bit. FAULT is active low.
    //drv_enable      = drv_enable && GPIO_readPin(p_drvCfg->gpioNumber_FAULT);
//    drv_status.stat_reg_00.FAULT = !GPIO_readPin(p_drvCfg->gpioNumber_FAULT);

    drv_status_s.stat00         = drv_status.stat_reg_00;
    drv_status_s.stat01         = drv_status.stat_reg_01;
    drv_status_s.stat00.FAULT   = !GPIO_readPin(p_drvCfg->gpioNumber_FAULT);
//    if (GPIO_readPin(p_drvCfg->gpioNumber_FAULT))
//        return (2);
    return(drv_status_s.all);
}

/**
 * @brief       Initialize the DRV registers with defined structures.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 * @return      SUCCESS config structure \& read registers match else ERROR.
 */
static bool_t DRV_SetConfig(const drv_cfg_t* p_drvCfg, drv_reg_t* p_drvReg)
{
    drv_reg_t reg_test  = {.ctrl_reg_02 = p_drvReg->ctrl_reg_02, .ctrl_reg_03 = p_drvReg->ctrl_reg_03,
                           .ctrl_reg_04 = p_drvReg->ctrl_reg_04, .ctrl_reg_05 = p_drvReg->ctrl_reg_05,
                           .ctrl_reg_06 = p_drvReg->ctrl_reg_06, .ctrl_reg_07 = p_drvReg->ctrl_reg_07};
    // Write initialized data to device
    DRV_writeAll(p_drvCfg, p_drvReg);
    // Read it back to ensure it worked
    DRV_readAll(p_drvCfg, p_drvReg);

    return(((reg_test.ctrl_reg_02.all   == p_drvReg->ctrl_reg_02.all) &&
            (reg_test.ctrl_reg_03.all   == p_drvReg->ctrl_reg_03.all) &&
            (reg_test.ctrl_reg_04.all   == p_drvReg->ctrl_reg_04.all) &&
            (reg_test.ctrl_reg_05.all   == p_drvReg->ctrl_reg_05.all) &&
            (reg_test.ctrl_reg_06.all   == p_drvReg->ctrl_reg_06.all) &&
            (reg_test.ctrl_reg_07.all   == p_drvReg->ctrl_reg_07.all)) ? (true) : (false));
}

/**
 * @brief       Force amplifiers calibration (short circuit inputs) of the DRV channels.
 *              - Calibration ON  : Amp inputs shorted, load disconnected, amplifier gain (GCSA) set to the 40 V/V.
 *              - Calibration OFF : Amp inputs opened, load reconnected, gain set to the original gain setting.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
static void DRV_Calibration(const drv_cfg_t* p_drvCfg, drv_reg_t* p_drvReg)
{
    // Set auto calibration
    drv_ctrl07_u auto_calib = {.CAL_MODE = true};
    DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_7, auto_calib.all); // Write Control Register 7
    // Enable channels calibration
    drv_ctrl06_u calib      = p_drvReg->ctrl_reg_06;
    calib.CSA_CAL_A         = true;
    calib.CSA_CAL_B         = true;
    calib.CSA_CAL_C         = true;
    DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_6, calib.all);// Write Control Register 6
    // Wait for 1ms
    DEVICE_DELAY_US(1000U);
    // stop auto calibration
    auto_calib.CAL_MODE     = false;
    DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_7, auto_calib.all); // Write Control Register 7
    // stop channels calibration
    calib.CSA_CAL_A         = false;
    calib.CSA_CAL_B         = false;
    calib.CSA_CAL_C         = false;
    DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_6, calib.all); // Write Control Register 6

    return;
}

/**
 * @brief       Read the DRV8353 status registers
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
uint32_t DRV_readStatus(const drv_cfg_t* p_drvCfg, drv_reg_t* p_drvReg)
{
    drv_stat0x_u drv_err_status     = {.stat00 = 0, .stat01 = 0};

    if((p_drvCfg != NULL) && (p_drvReg != NULL))
    {
        p_drvReg->stat_reg_00.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_0); // Read Status Register 0
        p_drvReg->stat_reg_01.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_1); // Read Status Register 1
    }
    drv_err_status.stat00.all       = p_drvReg->stat_reg_00.all;
    drv_err_status.stat01.all       = p_drvReg->stat_reg_01.all;

    return(drv_err_status.all);
}

/**
 * @brief       Read all the DRV8353 registers (global read access)
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
void DRV_readAll(const drv_cfg_t* p_drvCfg, drv_reg_t* p_drvReg)
{
    if((p_drvCfg != NULL) && (p_drvReg != NULL))
    {
        // Read all internal DRV registers (status & control registers).
        p_drvReg->stat_reg_00.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_0);  // Read Status Register 0
        p_drvReg->stat_reg_01.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_STATUS_1);  // Read Status Register 1
        p_drvReg->ctrl_reg_02.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_2); // Read Control Register 2
        p_drvReg->ctrl_reg_03.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_3); // Read Control Register 3
        p_drvReg->ctrl_reg_04.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_4); // Read Control Register 4
        p_drvReg->ctrl_reg_05.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_5); // Read Control Register 5
        p_drvReg->ctrl_reg_06.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_6); // Read Control Register 6
        p_drvReg->ctrl_reg_07.all   = DRV_readOne(p_drvCfg, DRV_ADDRESS_CONTROL_7); // Read Control Register 7
    }

    return;
}

/**
 * @brief       Write to all the DRV8353 write access registers (global write access)
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
void DRV_writeAll(const drv_cfg_t* p_drvCfg, drv_reg_t* p_drvReg)
{
    if((p_drvCfg != NULL) && (p_drvReg != NULL))
    {
        // Unlock write access registers (CTRL03)
        DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_3, p_drvReg->ctrl_reg_03.all); // Write Control Register 3
        //Write all internal DRV control registers
        DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_2, p_drvReg->ctrl_reg_02.all); // Write Control Register 2
        DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_4, p_drvReg->ctrl_reg_04.all); // Write Control Register 4
        DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_5, p_drvReg->ctrl_reg_05.all); // Write Control Register 5
        DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_6, p_drvReg->ctrl_reg_06.all); // Write Control Register 6
        DRV_writeOne(p_drvCfg, DRV_ADDRESS_CONTROL_7, p_drvReg->ctrl_reg_07.all); // Write Control Register 7
    }

    return;
}

/**
 * @brief       Read data from a single DRV8353 register
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 * @param[in]   addr        Address of the register to read
 * @return      The value contained in the specified register
 */
uint16_t DRV_readOne(const drv_cfg_t* p_drvCfg, DRV_Address_e addr)
{
    uint32_t spi_base   = p_drvCfg->spiHandle;
    uint32_t pin_spi_cs = p_drvCfg->gpioNumber_CS;
    drv_msg_u msg       = {.rw_cmd = DRV_CTRLMODE_READ, .addr = addr, .data = 0};

    // Select chip
    GPIO_writePin(pin_spi_cs, 0);
    // wait for registers to update
    DEVICE_DELAY_US(10U);
    // Write data
    SPI_writeDataNonBlocking(spi_base, msg.all);
    // Read RX buffer
    msg.all = SPI_readDataBlockingNonFIFO(spi_base);
    // Delay before deactivating communication
    DEVICE_DELAY_US(10U);
    // Deselect chip
    GPIO_writePin(pin_spi_cs, 1);

    return (msg.all);
}

/**
 * @brief       Write data to a single DRV8353 register
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 * @param[in]   addr        Address of the register to write
 * @param[in]   data        Data to be written to the specified register
 */
void DRV_writeOne(const drv_cfg_t* p_drvCfg, DRV_Address_e addr, uint16_t data)
{
    uint32_t spi_base   = p_drvCfg->spiHandle;
    uint32_t pin_spi_cs = p_drvCfg->gpioNumber_CS;
    drv_msg_u msg       = {.rw_cmd = DRV_CTRLMODE_WRITE, .addr = addr, .data = data};

    // Select chip
    GPIO_writePin(pin_spi_cs, 0);
    // wait for GPIO
    DEVICE_DELAY_US(10U);
    // write the command
    SPI_writeDataNonBlocking(spi_base, msg.all);
    // Wait for read buffer to be full
    SPI_readDataBlockingNonFIFO(spi_base);
    // Delay before deactivating communication
    DEVICE_DELAY_US(10U);
    // Deselect chip
    GPIO_writePin(pin_spi_cs, 1);

    return;
}
// end of file
