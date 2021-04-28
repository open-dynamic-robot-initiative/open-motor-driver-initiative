//! \file   solutions/boostxl_DRV8353rs/f28004x/drivers/source/DRV8353.c
//! \brief  Contains the various functions related to the DRV8353 object.
//!         This file is based on the DRV8353.c file and modified for the
//!         DRV8353 device.
//!

// **************************************************************************
// the includes

// **************************************************************************
// drivers
#include "drv8353.h"
#include "uomodri_config_handlers.h"

// **************************************************************************
// the function prototypes
static void DRV8353_Bases_init(DRV8353*);
static void DRV8353_Enable(DRV8353_Cfg*);
static bool DRV8353_Set_Config(DRV8353*);
static void DRV8353_Calibration(DRV8353_Cfg*);

// Function definitions
/**
 * @brief       Start \& configure the DRV ICs.
 * @param[in]   *p_drvHandle    Pointer on the DRV definition structure
 */
void DRV8353_init(DRV8353* p_drvHandle)
{
    // Initialize DRV structure
    DRV8353_Bases_init(p_drvHandle);

    DRV8353_Cfg* p_cfgHandle = p_drvHandle->drvCfgHandler;

    // Configures the DRVs
    while(p_cfgHandle->spiHandle != 0xFFFFFFFF) {
        DRV8353_Enable(p_cfgHandle);
        DRV8353_Set_Config(p_drvHandle);
        DRV8353_Calibration(p_cfgHandle);
        p_cfgHandle++;// = ++(p_drvHandle->drvCfgHandler);
    }
}

/**
 * @brief       Link DRV structure definition (DRV8353 params) and initialization parameters
 * @param[in]   *p_drvHandle    Pointer on the DRV definition structure
 */
static void DRV8353_Bases_init(DRV8353* p_drvHandle)
{
    // DRV8353 initialization
    p_drvHandle->drvCfgHandler = DrvCfgList;
    p_drvHandle->drvRegHandler = DrvRegList;
}

/**
 * @brief       Enable the DRV IC pin.
 * @param[in]   *p_cfgHandle    Pointer on the DRV configuration structure
 */
static void DRV8353_Enable(DRV8353_Cfg* p_cfgHandle)
{
    uint16_t enableWaitTimeOut = 0;
    // Enable the DRV8353
    GPIO_writePin(p_cfgHandle->gpioNumber_EN, 1);
    // Wait 1ms
    DEVICE_DELAY_US(1000U);
    // Make sure the FAULT bit is not set during startup
    while((DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_STATUS_0) & FLT_STATUS_FAULT_Msk) && (enableWaitTimeOut < 1000))
        p_cfgHandle->enableTimeOut = (++enableWaitTimeOut > 999) ? (true) : (false);
    // Wait for the DRV8353 to go through start up sequence
    DEVICE_DELAY_US(1000U);

    return;
}

/**
 * @brief       Initialize the DRV registers with defined structure.
 * @param[in]   *p_drvHandle    Pointer on the DRV definition structure
 * @return      SUCCESS config structure \& read registers match else ERROR.
 */
static bool DRV8353_Set_Config(DRV8353* p_drvHandle)
{
    DRV8353_Reg*    p_regHandle = p_drvHandle->drvRegHandler;
    DRV8353_Reg     drvTest     = {.Ctrl_Reg_02 = p_regHandle->Ctrl_Reg_02,
                                   .Ctrl_Reg_03 = p_regHandle->Ctrl_Reg_03,
                                   .Ctrl_Reg_04 = p_regHandle->Ctrl_Reg_04,
                                   .Ctrl_Reg_05 = p_regHandle->Ctrl_Reg_05,
                                   .Ctrl_Reg_06 = p_regHandle->Ctrl_Reg_06,
                                   .Ctrl_Reg_07 = p_regHandle->Ctrl_Reg_07};
    // Write initialized data to device
    DRV8353_writeAll(p_drvHandle);
    // Read it back to ensure it worked
    DRV8353_readAll(p_drvHandle);

    return (((drvTest.Ctrl_Reg_02.raw == p_regHandle->Ctrl_Reg_02.raw) &&
            (drvTest.Ctrl_Reg_03.raw == p_regHandle->Ctrl_Reg_03.raw) &&
            (drvTest.Ctrl_Reg_04.raw == p_regHandle->Ctrl_Reg_04.raw) &&
            (drvTest.Ctrl_Reg_05.raw == p_regHandle->Ctrl_Reg_05.raw) &&
            (drvTest.Ctrl_Reg_06.raw == p_regHandle->Ctrl_Reg_06.raw) &&
            (drvTest.Ctrl_Reg_07.raw == p_regHandle->Ctrl_Reg_07.raw)) ? (true) : (false));
}

/**
 * @brief       Force amplifiers calibration (short circuit inputs) of the DRV channels.
 *              - Calibration ON  : Amp inputs shorted, load disconnected, amplifier gain (GCSA) set to the 40 V/V.
 *              - Calibration OFF : Amp inputs opened, load reconnected, gain set to the original gain setting.
 * @param[in]   *p_cfgHandle    Pointer on the DRV configuration structure.
 */
static void DRV8353_Calibration(DRV8353_Cfg* p_cfgHandle)
{
    // Set auto calibration
    DRV8353_Ctrl07_u auto_calib = {.ctrl07.CAL_MODE = true};
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_7, auto_calib.raw);
    // Enable calibration
    DRV8353_Ctrl06_u calib = {.ctrl06.CSA_CAL_A = true, .ctrl06.CSA_CAL_B = true, .ctrl06.CSA_CAL_C = true};
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_6, calib.raw);
    // Wait for 1ms
    DEVICE_DELAY_US(1000U);
    // stop calibration
    calib.ctrl06.CSA_CAL_A = false;
    calib.ctrl06.CSA_CAL_B = false;
    calib.ctrl06.CSA_CAL_C = false;
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_6, calib.raw);

    return;
}

/**
 * @brief       Read only the DRV8353 status registers
 * @param[in]   *p_drvHandle    Pointer on the DRV definition structure
 */
void DRV8353_readStatus(DRV8353* p_drvHandle)
{
    DRV8353_Cfg* p_cfgHandle = p_drvHandle->drvCfgHandler;
    DRV8353_Reg* p_regHandle = p_drvHandle->drvRegHandler;

    // Read all internal DRV registers (status & control registers).
    p_regHandle->Stat_Reg_00.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_STATUS_0);
    // Read Status Register 1
    p_regHandle->Stat_Reg_01.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_STATUS_1);

    return;
}

/**
 * @brief       Read all the DRV8353 registers (global read access)
 * @param[in]   *p_drvHandle    Pointer on the DRV definition structure
 */
void DRV8353_readAll(DRV8353* p_drvHandle)
{
    DRV8353_Cfg* p_cfgHandle = p_drvHandle->drvCfgHandler;
    DRV8353_Reg* p_regHandle = p_drvHandle->drvRegHandler;

    // Read all internal DRV registers (status & control registers).
    p_regHandle->Stat_Reg_00.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_STATUS_0);
    // Read Status Register 1
    p_regHandle->Stat_Reg_01.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_STATUS_1);
    // Read Control Register 2
    p_regHandle->Ctrl_Reg_02.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_2);
    // Read Control Register 3
    p_regHandle->Ctrl_Reg_03.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_3);
    // Read Control Register 4
    p_regHandle->Ctrl_Reg_04.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_4);
    // Read Control Register 5
    p_regHandle->Ctrl_Reg_05.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_5);
    // Read Control Register 6
    p_regHandle->Ctrl_Reg_06.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_6);
    // Read Control Register 7
    p_regHandle->Ctrl_Reg_07.raw = DRV8353_readOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_7);

    return;
}

/**
 * @brief       Write to all the DRV8353 write access registers (global write access)
 * @param[in]   *p_drvHandle    Pointer on the DRV definition structure
 */
void DRV8353_writeAll(DRV8353* p_drvHandle)
{
    DRV8353_Cfg* p_cfgHandle = p_drvHandle->drvCfgHandler;
    DRV8353_Reg* p_regHandle = p_drvHandle->drvRegHandler;

    // Unlock write access registers (CTRL03)
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_3, p_regHandle->Ctrl_Reg_03.raw);

    //Write all internal DRV control registers
    // Write Control Register 2
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_2, p_regHandle->Ctrl_Reg_02.raw);
    // Write Control Register 3
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_3, p_regHandle->Ctrl_Reg_03.raw);
    // Write Control Register 4
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_4, p_regHandle->Ctrl_Reg_04.raw);
    // Write Control Register 5
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_5, p_regHandle->Ctrl_Reg_05.raw);
    // Write Control Register 6
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_6, p_regHandle->Ctrl_Reg_06.raw);
    // Write Control Register 7
    DRV8353_writeOne(p_cfgHandle, DRV8353_ADDRESS_CONTROL_7, p_regHandle->Ctrl_Reg_07.raw);

    return;
}

/**
 * @brief       Read data from a single DRV8353 register
 * @param[in]   *p_cfgHandle    Pointer on the DRV8353 object handler
 * @param[in]   addr            Address of the register to read
 * @return      The value contained in the specified register
 */
uint16_t DRV8353_readOne(DRV8353_Cfg* p_cfgHandle, DRV8353_Address_e addr)
{
    DRV8353_msg_u msg = {.fields.rw_cmd = (uint16_t)DRV8353_CTRLMODE_READ, .fields.addr = addr, .fields.data = 0};

    // Select chip
    GPIO_writePin(p_cfgHandle->gpioNumber_CS, 0);
    // wait for registers to update
    DEVICE_DELAY_US(10U);
    // Write data
    SPI_writeDataNonBlocking(p_cfgHandle->spiHandle, msg.raw);
    // Read RX buffer
    msg.raw = SPI_readDataBlockingNonFIFO(p_cfgHandle->spiHandle);
    // Delay before deactivating communication
    DEVICE_DELAY_US(10U);
    // Deselect chip
    GPIO_writePin(p_cfgHandle->gpioNumber_CS, 1);

    return (msg.raw);
}

/**
 * @brief       Write data to a single DRV8353 register
 * @param[in]   *p_cfgHandle    Pointer on the DRV8353 object handler
 * @param[in]   addr            Address of the register to write
 * @param[in]   data            Data to be written to the specified register
 */
void DRV8353_writeOne(DRV8353_Cfg* p_cfgHandle, DRV8353_Address_e addr, uint16_t data)
{
    DRV8353_msg_u msg = {.fields.rw_cmd = (uint16_t)DRV8353_CTRLMODE_WRITE, .fields.addr = addr, .fields.data = data};

    // Select chip
    GPIO_writePin(p_cfgHandle->gpioNumber_CS, 0);
    // wait for GPIO
    DEVICE_DELAY_US(10U);
    // write the command
    SPI_writeDataNonBlocking(p_cfgHandle->spiHandle, msg.raw);
    // Wait for read buffer to be full
    SPI_readDataBlockingNonFIFO(p_cfgHandle->spiHandle);
    // Delay before deactivating communication
    DEVICE_DELAY_US(10U);
    // Deselect chip
    GPIO_writePin(p_cfgHandle->gpioNumber_CS, 1);

    return;
}
// end of file
