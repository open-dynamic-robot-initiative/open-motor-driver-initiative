//
// Included Files
//
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <string.h>
//#include <float.h>

#include <stdbool.h>
#include <stdint.h>
#include "driverlib_cm.h"
#include "cm.h"

//#include "foc.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"
//#include "uomodri_prj_config_handlers.h"

extern const HAL_CM hal_cm;

#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];

//typedef float         float32_t;

/*********************************************************************************/
/**                                FOC STRUCTURE                                **/
/*********************************************************************************/
//typedef struct __LowpassFilter_Structure__
//{
//    float32_t a;
//    float32_t one_minus_a;
//} LpfStruct;

typedef struct __PI_STRUCT__
{
    float32_t*          in;         // pointer on the loopback current
    float32_t*          set;        // pointer on the setpoint current
    float32_t           err;        // Current error

    float32_t           kp;         // Proportional gain
    float32_t           ki;         // Integral gain

    float32_t           integral;   // Integral term partial computation
    float32_t           ff;         // Feed-forward term
    float32_t           out;        // Controller output
    float32_t*          sat;        // Controller saturation
} PI_STRUCT;

typedef struct __PD_STRUCT__
{
    float32_t*          p_;    // pointer on the loopback position
    float32_t*          setTheta;   // pointer on the setpoint position
    float32_t           errTheta;   // position error
    float32_t*          inSpeed;    // pointer on the loopback speed
    float32_t*          setSpeed;   // pointer on the setpoint speed
    float32_t           errSpeed;   // speed error

    float32_t           kp;         // Proportional gain
    float32_t           kd;         // Derivative gain

    float32_t           derivative; // Derivative term partial computation
    float32_t           ff;         // Feed-forward term
    float32_t           out;        // Controller output
    float32_t*          sat;        // Controller saturation
} PD_STRUCT;

typedef struct __PARAMETERS_STRUCT__
{
    // Motor constants
    float32_t           Rs;         // Stator resistance [Ohm]
    float32_t           Ld;         // Direct axis inductance [H]
    float32_t           Lq;         // Quadratic axis inductance [H]
    float32_t           kv;         // Motor speed constant [RPM/V or rad/s/V]
    float32_t           ke;         // Motor back EMF constant [V/rad/s] (electrical radians) = 1/kv
    float32_t           ki;         // Motor torque constant [N.m/A]
    uint8_t             polePairs;  // Number of pole pairs
} PARAMS_STRUCT;

typedef struct __ACQUISITIONS_STRUCT__
{
    float32_t*          p_adcRegs[5];
    float32_t*          p_adcResultRegs[5];
    // ADC result registers
    volatile uint16_t*                  p_vaMeasReg;
    volatile uint16_t*                  p_vbMeasReg;
    volatile uint16_t*                  p_vcMeasReg;
    volatile uint16_t*                  p_vBusMeasReg;
    volatile int16_t*                   p_iaMeasReg;
    volatile int16_t*                   p_ibMeasReg;
    volatile int16_t*                   p_icMeasReg;
    // ADC values scaled
    float32_t                           va;         // [V]
    float32_t                           vb;         // [V]
    float32_t                           vc;         // [V]
    float32_t                           vbus;       // [V]
    float32_t                           ia;         // [A]
    float32_t                           ib;         // [A]
    float32_t                           ic;         // [A]
    LpfStruct                           vBusLpf;

    bool                                measNew;    // ADC conversion done
} ACQ_STRUCT;

typedef struct __Anticoagging__
{
    // Enabled flag
    bool enabled;

    // Number of coefficients
    uint16_t coeffNb;

    // Frequency array
    const float *frequencies;
    // Fourier coefficients array
    const float *coeffs;
} Anticoagging;

typedef struct __SPEEDEST_STRUCT__
{
    // High speed estimation
    float32_t           speedHighScaler;// [rad/s/pu], diff_angular * scaler = velocity
    float32_t           speedHigh;    // [rad/s]
    // Low speed estimation
    float32_t           speedLowScaler; // [rad.register], scaler / dtRegister = velocity
    float32_t           speedLow;     // [rad/s]

    // Speed merge
    float32_t           alpha;     // Speed merge coefficient
    float32_t           speedRef;  // [rad/s] Merged of low speed and high speed estimation

    // Low-pass filter
    LpfStruct           lpf;

    // Output speeds
    float32_t           speedMech; // [rad/s]
    float32_t           speedElec; // [rad/s]

    // Utility variables
    float32_t           theta[2]; // [pu]
    bool                thetaDir; // 0 = CCW/reverse, 1 = CW/forward
} SPEED_STRUCT;

typedef struct __ENCODER_STRUCT__
{
    // pointers on HAL structure, QEP registers and direct access to QEP counter register
    float32_t*          p_qepHandle;
    float32_t*          p_qepRegs;

    uint8_t                     polePairs;    // Number of pole pairs of the motor
    // Scalers
    float32_t                   thetaMechScaler; // [pu/register]
    // Angles
    float32_t                   thetaElec; // [pu]
    float32_t                   thetaMech[2]; // [pu]
    bool                        thetaDir; //0 = CCW/reverse, 1 = CW/forward
//    bool                thetaNew; // New sample available?
    // Indexed angles
    bool                        indexCompensated;   // Is index compensation activated
    float32_t                   indexTheta;       // Index mech angle
    float32_t                   indexedThetaMech; // Mech angle relative to the index [pu]
    float32_t                   thetaAbsolute; // [turns]

    // Speed estimation structure
    SPEED_STRUCT                speedEst;
    // Communication variables
    int16_t                     turnNb;     // Number of full turns (signed)
    bool                        indexDetected; // Has index been detected
    bool                        indexToggle;   // Flag that toggles every time index is seen
    bool                        rollOverFlag;  // Roll-over error flag
} ENC_STRUCT;

typedef struct __FOC_STRUCT__
{
//    const ID_ENUM       motorID;
    // Motor configuration parameters
    PARAMS_STRUCT motor_cfg;
//    const PARAMS_STRUCT motor_cfg;
    // Current & Voltage measures
    ACQ_STRUCT          motor_acq;
    // Encoder position & speed
    ENC_STRUCT          motor_enc;

    // Low-pass control loop filter
    LpfStruct           iParkLpf;

    // Rotor electrical angle sine/cosine
    float32_t           sinTheta;
    float32_t           cosTheta;
    // Direct Clarke transform result
    float32_t           ialpha; // [A]
    float32_t           ibeta;  // [A]
    // Direct Park transform result
    float32_t           id;     // [A]
    float32_t           iq;     // [A]
    // Inverse Park transform result
    float32_t           ud;     // [V]
    float32_t           uq;     // [V]
    // Inverse Clarke transform result
    float32_t           ualpha; // [V]
    float32_t           ubeta;  // [V]
    // Phase commands
    float32_t           ua;     // [V]
    float32_t           ub;     // [V]
    float32_t           uc;     // [V]


    float32_t           vmax; // [V]
    float32_t           imax; // [A]
    // Duty-cycle saturation
    float32_t           dtcMax;
    float32_t           dtcMin;
    // Duty cycles
    float32_t           dtc_u;  // [pu] between -0.5 and 0.5
    float32_t           dtc_v;  // [pu] between -0.5 and 0.5
    float32_t           dtc_w;  // [pu] between -0.5 and 0.5

    // PI set point values
    float32_t           idRef;  // [A]
    float32_t           iqRef;  // [A]
    // PI controllers for current control
    PI_STRUCT           piId;
    PI_STRUCT           piIq;
    // PD set point values
    float32_t           posRef; // [rpm/s]
    // PD controller for position control
    PD_STRUCT           pdPos;

    // Q-axis resistance estimation
    float32_t           resistanceEst;    // [Ohm]
    // Resistance estimation low-pass filter
    LpfStruct           resEstLpf;
    // Anti-coagging structure
    Anticoagging        acog;
    // Enable flag
    bool                enabled;

    // Phase measured voltages
//    float32_t phaseAVolt;   // [V]
//    float32_t phaseBVolt;   // [V]
//    float32_t phaseCVolt;   // [V]

    // Auxiliary ADC voltage
//    float32_t auxAdc;   // [V]

    // ADC interrupt flag register
//    volatile union ADCINTFLG_REG *adcIntFlag;
} FOC_STRUCT;


//
// Defines
//
//#define IPC_CMD_READ_MEM    0x1001
//#define IPC_CMD_RESP        0x2001
//
//#define TEST_PASS           0x5555
//#define TEST_FAIL           0xAAAA
//
//#define DEBUG_LED_PIN2      34
//#define DEBUG_LED_PIN2_CFG  GPIO_34_GPIO34

//__interrupt void UART_RX_IntHandler(void);
__interrupt void IPC_ISR0(void);
void msgCreate(FOC_STRUCT*, uint8_t*);

//float32_t   arrayTx[4], arrayRx[4];
uint32_t    CntMsg = 0;
uint8_t     arrayTx[MSG_TX_SIZE], arrayRx[MSG_RX_SIZE];

//uint8_t     arrayRxShort[24];

//IPC_MessageQueue_t msgQueue;
////
//// IPC ISR for Flag 0.
//// C28x core sends data without message queue using Flag 0
////
//__interrupt void IPC_ISR0()
//{
//    int i;
//    uint32_t command, addr, data;
//    bool status = false;
//
//    // Read the command
//    IPC_readCommand(IPC_CM_L_CPU1_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data);
//
//    if(command == IPC_CMD_READ_MEM)
//    {
//        status = true;
//
//        // Read and compare data
//        for(i=0; i<data; i++)
//            if(*((uint32_t *)addr + i) != i)
//                status = false;
//    }
//    // Send response to C28x core
//    IPC_sendResponse(IPC_CM_L_CPU1_R, ((status) ? (TEST_PASS) : (TEST_FAIL)));
//    // Acknowledge the flag
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG0);
//}
//__interrupt void IPC_ISR1(void);

//
// IPC ISR for Flag 1
// C28x core sends data with message queue using Flag 0
//
__interrupt void IPC_ISR0(void)
{
//    FOC_STRUCT* p_foc = NULL;
    uint32_t    command, data, addr;
//    uint8_t* p_arrayRx = (uint8_t *)arrayRx;
//    uint8_t i;
    // Read the command
    IPC_readCommand(ipcCfgList[0].ipcType, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data);
//    p_foc = (FOC_STRUCT *)addr;

    msgCreate((FOC_STRUCT *)addr, arrayTx);

//    while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFE) == 0);
    while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY) == 1);
    while(UART_readCharNonBlocking(UART0_BASE) != -1);

    // Enable the UDMA channels
//    if(!UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_RX))
//    {
//        ucControlTable[UDMA_CHANNEL_UART0_RX].chControlAttr |= udmaCfgList[0].transferMode;
//        ucControlTable[UDMA_CHANNEL_UART0_RX].chControlAttr |= ((udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
//        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_RX);
//    }

    if(!UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_TX))
    {
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= udmaCfgList[1].transferMode;
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((udmaCfgList[1].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);//udmaCfgList[1].dmaChannel);
    }

//    UDMA_requestSoftwareTransfer(UDMA_BASE, UDMA_CHANNEL_SW_CH30);
    // Wait until the complete transfer is done
//    while(UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_TX));
//    while(UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_RX));
    // Wait until the complete transfer is done
//    while(UDMA_isChannelEnabled(UDMA_BASE, udmaCfgList[0].dmaChannel));

//    status = (command == IPC_CMD_READ_MEM) ? (true) : (false);
//    // Send response to C28x core
//    IPC_sendResponse(IPC_CM_L_CPU1_R, (status) ? (TEST_PASS) : (TEST_FAIL));
    // Acknowledge the flag
//    for(i = 0; i < 24; i++)
//        arrayRxShort[i] = *p_arrayRx++;

    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG0);

//    // Read the message from the message queue
//    IPC_readMessageFromQueue(IPC_CM_L_CPU1_R, &msgQueue, IPC_ADDR_CORRECTION_ENABLE, &msgRx, IPC_NONBLOCKING_CALL);
//
//
//    if(msgRx.command == IPC_CMD_READ_MEM)
//    {
//        status = true;
//        // Read and compare data
//        for(i = 0; i < msgRx.dataw1; i++)
//            if((*(uint32_t *)msgRx.address + i) != i)
//                status = false;
//    }
//
//    // Send response message
//    msgTx.command = IPC_CMD_RESP;
//    msgTx.address = 0; // Not used
//    msgTx.dataw1  = status ? TEST_PASS : TEST_FAIL;
//    msgTx.dataw2  = msgRx.dataw2; // Use the message identifier from the received message
//
//    IPC_sendMessageToQueue(IPC_CM_L_CPU1_R, &msgQueue, IPC_ADDR_CORRECTION_DISABLE, &msgTx, IPC_NONBLOCKING_CALL);
//    // Acknowledge the flag
//    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG1);
//
//    if(status == true)
//        GPIO_writePin(DEBUG_LED_PIN2, 0);
//    else if(status == false)
//        GPIO_writePin(DEBUG_LED_PIN2, 1);
}

//__interrupt void UART_RX_IntHandler(void)
//{
//    uint32_t ui32Status;
//
//    // Get the interrupt status.
//    ui32Status = UART_getInterruptStatus(UART0_BASE, UART_RAW_INT);
//    // Clear the asserted interrupts.
//    UART_clearInterruptStatus(UART0_BASE, ui32Status);
//    // Loop while there are characters in the receive FIFO.
//    while(UART_isDataAvailable(UART0_BASE))
//        // Read the next character from the UART and write it back to the UART.
//        UART_writeCharNonBlocking(UART0_BASE, UART_readCharNonBlocking(UART0_BASE));
//}


//
// Main
//
void main(void)
{
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);

    // Loop forever. Wait for IPC interrupt
    while(1);
}

void msgCreate(FOC_STRUCT* p_foc, uint8_t* p_array)
{
    float32_t   arrayTmp[4];
    //    uint8_t     aMsgCreate[MSG_SIZE] = {'a', 'b', 'c', 'd'};
    uint8_t     *pspl, *pCntMsg;
    uint8_t     i;

    arrayTmp[0] = p_foc->motor_acq.ia + 1.0;
    arrayTmp[1] = p_foc->motor_acq.ib + 1.0;
    arrayTmp[2] = p_foc->motor_acq.ic + 1.0;
    arrayTmp[3] = p_foc->motor_enc.thetaMech[NEW];
    //arrayTmp[4] = p_foc->motor_enc.thetaElec;
    pCntMsg     = (uint8_t *)&CntMsg;
    pspl        = (uint8_t *)&arrayTmp[0];

    for(i = 0; i < MSG_TX_SIZE; i++)
    {
        if(i < 4)
            *p_array = 'a' + i; //aMsgCreate[i] = 'a' + i;
        else if(i < 8)
            *p_array = *pCntMsg++; //aMsgCreate[i] = *pCntMsg++;
        else if(i < MSG_TX_SIZE)
            *p_array = *pspl++; //aMsgCreate[i] = *pspl++;
        p_array++;
    }
    CntMsg++;
}


//
// End of File
//
