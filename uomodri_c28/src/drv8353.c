/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"
#include "uomodri_user_defines.h"
#include "drv8353.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
static bool_t   DRV_Enable(drv8353_t*);
static bool_t   DRV_SetConfig(drv8353_t*);
static void     DRV_Calibration(drv8353_t*);
uint16_t        DRV_readOne(drv8353_t*, DRV_Address_e);
void            DRV_writeOne(drv8353_t*, DRV_Address_e, uint16_t);

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Start \& configure the DRV ICs.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
void DRV_ini(drv8353_t* p_drv)
{
    if((p_drv != NULL) && (p_drv->p_drvCfgHandler != NULL) && (p_drv->p_drvRegHandler != NULL))
    {
        DRV_Enable(p_drv);
        if(DRV_SetConfig(p_drv))
            DRV_Calibration(p_drv);
    }

    return;
}

/**
 * @brief       Enable the DRV IC pin.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 * @return      bool        True if DRV is in fault state else false.
 */
static bool_t DRV_Enable(drv8353_t* p_drv)
{
    uint32_t pin_drv_en     = p_drv->p_drvCfgHandler->gpioNumber_EN;
    uint16_t cpt_time_out   = 0;
    bool_t fault_drv        = false;
    // Enable the DRV8353
    GPIO_writePin(pin_drv_en, 1);
    // Wait 1ms
    DEVICE_DELAY_US(1000U);
    // Make sure the FAULT bit is not set during startup
    while((DRV_readOne(p_drv, DRV_ADDRESS_STATUS_0) & FLT_STATUS_FAULT_Msk) && (cpt_time_out < 1000))
        fault_drv = (++cpt_time_out > 999) ? (true) : (false);
    // Wait for the DRV8353 to go through start up sequence
    DEVICE_DELAY_US(1000U);

    return (fault_drv);
}

/**
 * @brief       Initialize the DRV registers with defined structures.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 * @return      SUCCESS config structure \& read registers match else ERROR.
 */
static bool_t DRV_SetConfig(drv8353_t* p_drv)
{
    drv_reg_t* p_reg    = p_drv->p_drvRegHandler;
    drv_reg_t reg_test  = {.ctrl_reg_02 = p_reg->ctrl_reg_02, .ctrl_reg_03 = p_reg->ctrl_reg_03,
                           .ctrl_reg_04 = p_reg->ctrl_reg_04, .ctrl_reg_05 = p_reg->ctrl_reg_05,
                           .ctrl_reg_06 = p_reg->ctrl_reg_06, .ctrl_reg_07 = p_reg->ctrl_reg_07};
    // Write initialized data to device
    DRV_writeAll(p_drv);
    // Read it back to ensure it worked
    DRV_readAll(p_drv);

    return (((reg_test.ctrl_reg_02.all  == p_reg->ctrl_reg_02.all) &&
            (reg_test.ctrl_reg_03.all   == p_reg->ctrl_reg_03.all) &&
            (reg_test.ctrl_reg_04.all   == p_reg->ctrl_reg_04.all) &&
            (reg_test.ctrl_reg_05.all   == p_reg->ctrl_reg_05.all) &&
            (reg_test.ctrl_reg_06.all   == p_reg->ctrl_reg_06.all) &&
            (reg_test.ctrl_reg_07.all   == p_reg->ctrl_reg_07.all)) ? (true) : (false));
}

/**
 * @brief       Force amplifiers calibration (short circuit inputs) of the DRV channels.
 *              - Calibration ON  : Amp inputs shorted, load disconnected, amplifier gain (GCSA) set to the 40 V/V.
 *              - Calibration OFF : Amp inputs opened, load reconnected, gain set to the original gain setting.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
static void DRV_Calibration(drv8353_t* p_drv)
{
    // Set auto calibration
    drv_ctrl07_u auto_calib = {.bit.CAL_MODE = true};
    DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_7, auto_calib.all); // Write Control Register 7
    // Enable channels calibration
    drv_ctrl06_u calib      = p_drv->p_drvRegHandler->ctrl_reg_06;
    calib.bit.CSA_CAL_A     = true;
    calib.bit.CSA_CAL_B     = true;
    calib.bit.CSA_CAL_C     = true;
    DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_6, calib.all);// Write Control Register 6
    // Wait for 1ms
    DEVICE_DELAY_US(1000U);
    // stop auto calibration
    auto_calib.bit.CAL_MODE = false;
    DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_7, auto_calib.all); // Write Control Register 7
    // stop channels calibration
    calib.bit.CSA_CAL_A     = false;
    calib.bit.CSA_CAL_B     = false;
    calib.bit.CSA_CAL_C     = false;
    DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_6, calib.all); // Write Control Register 6

    return;
}

/**
 * @brief       Read the DRV8353 status registers
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
void DRV_readStatus(drv8353_t* p_drv)
{
    drv_reg_t* p_reg = p_drv->p_drvRegHandler;
    if((p_drv != NULL) && (p_reg != NULL))
    {
        p_reg->stat_reg_00.all = DRV_readOne(p_drv, DRV_ADDRESS_STATUS_0); // Read Status Register 0
        p_reg->stat_reg_01.all = DRV_readOne(p_drv, DRV_ADDRESS_STATUS_1); // Read Status Register 1
    }

    return;
}

/**
 * @brief       Read all the DRV8353 registers (global read access)
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
void DRV_readAll(drv8353_t* p_drv)
{
    drv_reg_t* p_reg = p_drv->p_drvRegHandler;
    if((p_drv != NULL) && (p_reg != NULL))
    {
        // Read all internal DRV registers (status & control registers).
        p_reg->stat_reg_00.all = DRV_readOne(p_drv, DRV_ADDRESS_STATUS_0);  // Read Status Register 0
        p_reg->stat_reg_01.all = DRV_readOne(p_drv, DRV_ADDRESS_STATUS_1);  // Read Status Register 1
        p_reg->ctrl_reg_02.all = DRV_readOne(p_drv, DRV_ADDRESS_CONTROL_2); // Read Control Register 2
        p_reg->ctrl_reg_03.all = DRV_readOne(p_drv, DRV_ADDRESS_CONTROL_3); // Read Control Register 3
        p_reg->ctrl_reg_04.all = DRV_readOne(p_drv, DRV_ADDRESS_CONTROL_4); // Read Control Register 4
        p_reg->ctrl_reg_05.all = DRV_readOne(p_drv, DRV_ADDRESS_CONTROL_5); // Read Control Register 5
        p_reg->ctrl_reg_06.all = DRV_readOne(p_drv, DRV_ADDRESS_CONTROL_6); // Read Control Register 6
        p_reg->ctrl_reg_07.all = DRV_readOne(p_drv, DRV_ADDRESS_CONTROL_7); // Read Control Register 7
    }

    return;
}

/**
 * @brief       Write to all the DRV8353 write access registers (global write access)
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
void DRV_writeAll(drv8353_t* p_drv)
{
    drv_reg_t* p_reg = p_drv->p_drvRegHandler;
    if((p_drv != NULL) && (p_reg != NULL))
    {
        // Unlock write access registers (CTRL03)
        DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_3, p_reg->ctrl_reg_03.all); // Write Control Register 3
        //Write all internal DRV control registers
        DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_2, p_reg->ctrl_reg_02.all); // Write Control Register 2
        DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_4, p_reg->ctrl_reg_04.all); // Write Control Register 4
        DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_5, p_reg->ctrl_reg_05.all); // Write Control Register 5
        DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_6, p_reg->ctrl_reg_06.all); // Write Control Register 6
        DRV_writeOne(p_drv, DRV_ADDRESS_CONTROL_7, p_reg->ctrl_reg_07.all); // Write Control Register 7
    }

    return;
}

/**
 * @brief       Read data from a single DRV8353 register
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 * @param[in]   addr        Address of the register to read
 * @return      The value contained in the specified register
 */
uint16_t DRV_readOne(drv8353_t* p_drv, DRV_Address_e addr)
{
    uint32_t spi_base   = p_drv->p_drvCfgHandler->spiHandle;
    uint32_t pin_spi_cs = p_drv->p_drvCfgHandler->gpioNumber_CS;
    drv_msg_u msg       = {.bit.rw_cmd = DRV_CTRLMODE_READ, .bit.addr = addr, .bit.data = 0};

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
void DRV_writeOne(drv8353_t* p_drv, DRV_Address_e addr, uint16_t data)
{
    uint32_t spi_base   = p_drv->p_drvCfgHandler->spiHandle;
    uint32_t pin_spi_cs = p_drv->p_drvCfgHandler->gpioNumber_CS;
    drv_msg_u msg       = {.bit.rw_cmd = DRV_CTRLMODE_WRITE, .bit.addr = addr, .bit.data = data};

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
