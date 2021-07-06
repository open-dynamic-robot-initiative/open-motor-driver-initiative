//-----------------------------------------------------------------------
//                          INCLUDES
//-----------------------------------------------------------------------

#include "spi_communication.h"
#include "checksum.h"
#include "lsb_msb_macros.h"

//-----------------------------------------------------------------------
//                      GLOBAL VARIABLES
//-----------------------------------------------------------------------
// Global dual buffer declarations
// Put them in the RAMG so DMA can reach them
#pragma DATA_SECTION(dualBufferW, "ramgs0");
DualBufferW dualBufferW;
#pragma DATA_SECTION(dualBufferR, "ramgs0");
DualBufferR dualBufferR;

// Debug
uint32_t comError = 0;
uint32_t comSuccess = 0;

//DualBufferW dualBufferW;
//DualBufferR dualBufferR;

//-----------------------------------------------------------------------
//                      INITIALIZATION ROUTINE
//-----------------------------------------------------------------------

// Communication initialization routine
void SPI_initCommunication(const DMA_TypeDef* p_dmaCfg)
{
    if(p_dmaCfg != NULL)
    {
        uint32_t dmaChBase = p_dmaCfg->dmaChBase;

        while(dmaChBase != 0xFFFFFFFF)
        {
            if(p_dmaCfg->dmaChBase == COM_SPI_TX_DMA_BASE_ADDR)
                dualBufferW.ptrDmaCh = (volatile struct CH_REGS*)p_dmaCfg->dmaChBase;
            else if(p_dmaCfg->dmaChBase == COM_SPI_RX_DMA_BASE_ADDR)
                dualBufferR.ptrDmaCh = (volatile struct CH_REGS*)p_dmaCfg->dmaChBase;
            // DMA register pointers initialization
            // Start channel
            DMA_startChannel(dmaChBase);
            dmaChBase = (++p_dmaCfg)->dmaChBase;
        }
    }
}

//-----------------------------------------------------------------------
//                      WRITE PACKETS ROUTINES
//-----------------------------------------------------------------------

// Returns a pointer to the packet that isn't sent to the master board
inline ComPacketWPtr SPI_getAvailableWPacket()
{
    return (dualBufferW.comPackets + !dualBufferW.index); // !index corresponds to the unused packet index
}

// Compute CRC and swap Tx packets after the unused one has been updated
inline void SPI_pushWPacket()
{
    ComPacketWPtr availablePacketAddress;

    // Swap DualBuffer index and compute new DMA begin address
    dualBufferW.index = !dualBufferW.index; // Toggle index
    availablePacketAddress = dualBufferW.comPackets + dualBufferW.index;

    // Compute packet CRC
    uint32_t crc = COM_crc32((uint16_t *)availablePacketAddress, COM_SPI_TX_RX_LENGTH - 2); // The last two words are CRC fields
    // Update packet CRC
    availablePacketAddress->crc_msb = MSB_32(crc);
    availablePacketAddress->crc_lsb = LSB_32(crc);

    // Wait for end of transfer before changing addresses
    while (dualBufferW.ptrDmaCh->CONTROL.bit.TRANSFERSTS);

    // Change source addresses
    EALLOW;
    dualBufferW.ptrDmaCh->SRC_BEG_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    dualBufferW.ptrDmaCh->SRC_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    EDIS;
}

//-----------------------------------------------------------------------
//                      READ PACKETS ROUTINES
//-----------------------------------------------------------------------

// Returns a pointer to the packet that doesn't receive master board data
// If the CRC of the returned packet isn't correct, return (ComPacketRPtr)0xFFFFFFFF pointer instead.
// The user MUST check if the returned pointer is (ComPacketRPtr)0xFFFF before using it.
inline ComPacketRPtr SPI_getAvailableRPacket()
{
    ComPacketRPtr availablePacketAddress;
    ComPacketRPtr currentPacketAddress;

    // Compute address of the packet currently used for the comunication
    currentPacketAddress = dualBufferR.comPackets + dualBufferR.index;

    // Check if packet have been read already
    if (currentPacketAddress->mode.bit.rsvd == 1)
    {
        return RPACKET_ERROR;
    }

    // Swap DualBuffer index and compute new DMA begin address
    dualBufferR.index = !dualBufferR.index; // Toggle index
    availablePacketAddress = dualBufferR.comPackets + dualBufferR.index;

    // Wait for end of transfer before changing addresses
    while (dualBufferR.ptrDmaCh->CONTROL.bit.TRANSFERSTS);

    // Change destination address
    EALLOW;
    dualBufferR.ptrDmaCh->DST_BEG_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    dualBufferR.ptrDmaCh->DST_ADDR_SHADOW = (uint32_t)availablePacketAddress;
    EDIS;

    // Compute CRC
    uint32_t crc = COM_crc32((uint16_t *)currentPacketAddress, COM_SPI_TX_RX_LENGTH - 2); // The last two words are CRC fields

    // Mark packet as read
    currentPacketAddress->mode.bit.rsvd = 1;

    // If CRC isn't correct, return error pointer
    if (MSB_32(crc) != currentPacketAddress->crc_msb || LSB_32(crc) != currentPacketAddress->crc_lsb)
    {
        comError++;

        return RPACKET_ERROR;
    }
    else
    {
        comSuccess++;

        return currentPacketAddress;
    }
}
