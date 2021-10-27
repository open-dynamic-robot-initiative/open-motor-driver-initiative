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
#include <math.h>
#include "driverlib_cm.h"
#include "cm.h"

//#include "foc.h"
#include "communication_interface.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"
//#include "uomodri_prj_config_handlers.h"

extern const HAL_CM hal_cm;

#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];

#pragma DATA_SECTION(msgComInter, "MSGRAM_CM_TO_CPU1")
ComInterface msgComInter;
#pragma DATA_SECTION(msgArrayTx, "MSGRAM_CM_TO_CPU1");
uint8_t msgArrayTx[MSG_TX_SIZE];
#pragma DATA_SECTION(msgArrayRx, "MSGRAM_CM_TO_CPU1");
uint8_t msgArrayRx[MSG_RX_SIZE];

uint64_t    interruptCnt_M1 = 0;
uint64_t    interruptCnt_M2 = 0;

#define TIME_M1_S   ((float32_t)interruptCnt_M1 * dt)
#define TIME_M1_MS  ((float32_t)interruptCnt_M1 * 1000.0f * dt)
#define TIME_M2_S   ((float32_t)interruptCnt_M2 * dt)
#define TIME_M2_MS  ((float32_t)interruptCnt_M2 * 1000.0f * dt)

#define IPC_CMD_READ_MEM    0x1001
#define MSG_COM_INTER       0x2001
#define MSG_COM_ADDR        0x3001

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


// Sine for debug
const float32_t     debugFreq = 1.0f;
const float32_t     dt = 1.0 / PWM_FREQ;
//float32_t   arrayTx[4], arrayRx[4];
uint32_t    CntMsg = 0;

//static ComInterface comInterface =
//{
// .systemEnabled     = false,
// .lastCommandIndex  = 0,
// .time              = 0.0,
// .lastMessageMs     = 0,
// .timeoutThreshold  = 0xFFFF,
// .rolloverErrorFlag = false,
// .canError          = SPI_SENSOR_STATUS_ERROR_NO_ERROR,
// .positionRef       = {0.0, 0.0},
// .velocityRef       = {0.0, 0.0},
// .iqRef             = {0.0, 0.0},
// .positionLsb       = CI_POSITION_LSB,
// .velocityLsb       = CI_VELOCITY_LSB,
// .iqLsb             = CI_IQ_LSB,
// .currentSatLsb     = CI_CURRENT_SAT_LSB,
// .resistanceLsb     = CI_LSBISTANCE_LSB,
// .voltageLsb        = CI_VOLTAGE_LSB,
// .kpLsb             = CI_KP_LSB,
// .kdLsb             = CI_KD_LSB,
//};

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
    uint32_t    command, data, addr;
    uint8_t     ipcCfgIndex     = 0;
    uint8_t     udmaCfgIndex    = 0;

    // Get index associated to IPC structure configuration
    while((cm_ipcCfgList[ipcCfgIndex].p_ipcIntHandler != &IPC_ISR0) ? (ipcCfgIndex++) : (ipcCfgIndex));
    // Get structure address
    IPC_readCommand(cm_ipcCfgList[ipcCfgIndex].ipcType, cm_ipcCfgList[ipcCfgIndex].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data);
    // Create message
    msgCreate((FOC_STRUCT *)addr, msgArrayTx);
    // Wait Tx buffer + UART Tx module are empty.
    while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY) == 1);
    // Force purge of Rx buffer.
    while(UART_readCharNonBlocking(UART0_BASE) != -1);
    // Get the index associated to uDMA UART0_TX configuration
    while((cm_udmaCfgList[udmaCfgIndex].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIndex++) : (udmaCfgIndex));
    // Enable the uDMA UART0 Tx channel
    if(!UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_TX))
    {
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= cm_udmaCfgList[udmaCfgIndex].transferMode;
        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[udmaCfgIndex].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
    }
    // Acknowledge IT
    IPC_ackFlagRtoL(cm_ipcCfgList[ipcCfgIndex].ipcType, cm_ipcCfgList[ipcCfgIndex].ipcFlag);
    // Get index associated to IPC structure configuration
    ipcCfgIndex = 0;
    while((cm_ipcCfgList[ipcCfgIndex].p_ipcIntHandler != NULL) ? (ipcCfgIndex++) : (ipcCfgIndex));
    // Share the message generated with C28-CPU1
    IPC_sendCommand(cm_ipcCfgList[ipcCfgIndex].ipcType, cm_ipcCfgList[ipcCfgIndex].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, MSG_COM_INTER, (uint32_t)(&msgArrayTx), 1);
//    IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE, 0, 0, 1);

//    i = 0;
//    // Get The raw number associated to uDMA UART0_RX configuration
//    while((udmaCfgList[i].dmaChannel != UDMA_CHANNEL_UART0_RX) ? (i++) : (i));
//    // Enable the uDMA UART0 Rx channel
//    if(!UDMA_isChannelEnabled(UDMA_BASE, UDMA_CHANNEL_UART0_RX))
//    {
//        ucControlTable[UDMA_CHANNEL_UART0_RX].chControlAttr |= udmaCfgList[i].transferMode;
//        ucControlTable[UDMA_CHANNEL_UART0_RX].chControlAttr |= ((udmaCfgList[i].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
//        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_RX);
//    }

//#if UOMODRI_SPI_DEBUG_CONFIG
    // Compute predefined trajectory reference
//    msgComInter.positionRef[MOTOR_1]   = sinf(debugFreq * TIME_M1_S);//debugFreq * TIME_S;//
//    msgComInter.velocityRef[MOTOR_1]   = 4.0f * M_PI * M_PI * debugFreq * cosf(debugFreq * TIME_M1_S);//debugFreq * TIME_S;//
//    msgComInter.iqRef[MOTOR_1]         = 0.0f;
//    /* Send a message without message queue.
//     * Since C28x and CM does not share the same address space for shared RAM,
//     * ADDRESS_CORRECTION is enabled
//     * Length of the data to be read is passed as data. */
//    IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE, IPC_CMD_RESP, (uint32_t)(&msgComInter), 1);
//#endif
//    interruptCnt_M1++;

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
    uint8_t     ipcCfgIndex     = 0;
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);

    // Get index associated to IPC structure configuration
    while((cm_ipcCfgList[ipcCfgIndex].p_ipcIntHandler != NULL) ? (ipcCfgIndex++) : (ipcCfgIndex));
    // Share the message generated with C28-CPU1
    IPC_sendCommand(cm_ipcCfgList[ipcCfgIndex].ipcType, cm_ipcCfgList[ipcCfgIndex].ipcFlag, IPC_ADDR_CORRECTION_ENABLE, MSG_COM_ADDR, (uint32_t)(&msgArrayTx), 1);

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
