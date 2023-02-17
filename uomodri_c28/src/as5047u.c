
#ifdef USE_CM_CORE
#include "driverlib_cm.h"
#include "cm.h"
#else
#include "f2838x_device.h"
#include "driverlib.h"
#include "device.h"
#endif

#include <as5047u.h>

void AS5047UReset(as_enc_t* p_enc){

    p_enc->angleData.all        = 0x00000000;
    p_enc->velData.all          = 0x00000000;
    p_enc->degAngle             = 0.0f;
    p_enc->prevAngle            = 0.0f;
    p_enc->startAngle           = 0.0f;
    p_enc->totalAngle           = 0.0f;
    p_enc->turnNum              = 0.0f;
    p_enc->rndCount             = 0.0f;
    p_enc->vel                  = 0.0f;
    p_enc->quadNum              = 0.0f;
    p_enc->prevQuadNum          = 0.0f;


    readAngle(p_enc);
    p_enc->startAngle = p_enc->degAngle;

}


uint16_t send16(as_enc_t* p_enc , uint16_t command){
    uint32_t spi_base   = p_enc->asencHandle->spiHandle;
    uint32_t pin_spi_cs = p_enc->asencHandle->gpioNumber_CS;

    GPIO_writePin(pin_spi_cs, 0); //Drive chip select low

    SPI_writeDataBlockingFIFO(spi_base, command); // Write in TX buffer
    uint16_t receivedData = SPI_readDataBlockingFIFO(spi_base); //Read RX buffer

    GPIO_writePin(pin_spi_cs, 1); //Drive chip select high

    return receivedData;
}

uint32_t send32(as_enc_t* p_enc, uint32_t command){

    uint32_t spi_base   = p_enc->asencHandle->spiHandle;
    uint32_t pin_spi_cs = p_enc->asencHandle->gpioNumber_CS;
    uint16_t rData[2];
    uint32_t receivedData=0;

    GPIO_writePin(pin_spi_cs, 0);

    SPI_writeDataNonBlocking(spi_base, command>>16);
    SPI_writeDataNonBlocking(spi_base, command);
    rData[0] = SPI_readDataBlockingFIFO(spi_base);
    rData[1] = SPI_readDataBlockingFIFO(spi_base);

    GPIO_writePin(pin_spi_cs, 1);

    receivedData=((uint32_t)rData[0] << 16U) |
                (uint32_t) rData[1];

    return receivedData;
}

/* readData function, reads a 16 bit message from a certain register in the encoder */

uint16_t readData(as_enc_t* p_enc, uint16_t command){

    send16(p_enc,command);
    uint16_t receivedData=send16(p_enc,NOP_REG);
    return receivedData;
}

/*readData32 function, reads a 32 bit message from a certain register in the encoder*/

uint32_t readData32(as_enc_t* p_enc , uint32_t command){

    send32(p_enc,command);
    uint32_t receivedData=send32(p_enc,0x000000F1);
    return receivedData;

}
void getAngleandVelocity(as_enc_t* p_enc){
    CmdFrame32_u command;
    uint8_t com[2];


    command.bit.rw =READ;
    command.bit.dnc=0;
    command.bit.pad=0x00;
    command.bit.crc=0;
    command.bit.cmdFrame = ANGLE_REG;
    com[0]=(command.all&0x00FFFF00)>>16;
    com[1]=(command.all&0x00FFFF00)>>8;
    command.bit.crc= calcCRC(com);

    send32(p_enc,command.all);

    command.bit.cmdFrame = VEL_REG;
    com[0]=(command.all&0x00FFFF00)>>16;
    com[1]=(command.all&0x00FFFF00)>>8;
    command.bit.crc= calcCRC(com);

    p_enc->angleData.all=send32(p_enc,command.all);
    p_enc->velData.all=send32(p_enc,0x000000F1);

}


BitDataFrame16_u readRegister(as_enc_t* p_enc, uint16_t registerAddress){

    CmdFrame16_u command;
    BitDataFrame16_u receivedFrame;

    command.bit.rw= READ;
    command.bit.cmdFrame = registerAddress;
    command.bit.dnc = 0;

    receivedFrame.all=readData(p_enc,command.all);

    return receivedFrame;
}

BitDataFrame32_u readRegister32(as_enc_t* p_enc, uint16_t registerAddress){

    CmdFrame32_u command;
    BitDataFrame32_u receivedFrame;
    uint8_t com[2];


    command.bit.rw =READ;
    command.bit.dnc=0;
    command.bit.pad=0x00;
    command.bit.crc=0;
    command.bit.cmdFrame = registerAddress;

    com[0]=(command.all&0x00FFFF00)>>16;
    com[1]=(command.all&0x00FFFF00)>>8;

    command.bit.crc= calcCRC(com);

    receivedFrame.all=readData32(p_enc, command.all);

    return receivedFrame;
}







/*void readAngle32(as_enc_t* p_enc){
    BitDataFrame32_u readDataFrame = readRegister32(p_enc, ANGLE_REG);
    if (checkCRC(readDataFrame.bit.data,readDataFrame.bit.crc)){ //would need to split the bit into two later, don't forget
        p_enc->angleData = readDataFrame.bit.data;

        readDataFrame = readRegister32(p_enc, VEL_REG);
        p_enc->vel = readDataFrame.bit.data;

        p_enc->degAngle=p_enc->angleData/16384.*360.;
    }
}
*/



uint8_t calcCRC(uint8_t* command){

        uint32_t crc;
        int16_t i,bit;

        crc = 0xC4;
        for ( i=0 ; i<2 ; i++ )
        {
            crc ^= command[i];
            for ( bit=0 ; bit<8 ; bit++)
            {
                if ( (crc & 0x80)!=0 )
                {
                    crc <<= 1;
                    crc ^= 0x1D;
                }
                else
                {
                    crc <<= 1;
                }
            }
        }

        return (crc ^ 0xFF); //A^B=A&Bbar + B&Abar
}

bool checkCRC(uint8_t* frame, uint8_t crc){
    return(calcCRC(frame)== crc);
}

