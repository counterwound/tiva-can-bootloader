/******************************************************************
 * Tiva CANbus Application Example
 * Developed by Sevun Scientific, Inc.
 * http://sevunscientific.com
 * *****************************************************************
 *
 *    _____/\\\\\\\\\\\_______/\\\\\\\\\\\____/\\\\\\\\\\\_
 *     ___/\\\/////////\\\___/\\\/////////\\\_\/////\\\///__
 *      __\//\\\______\///___\//\\\______\///______\/\\\_____
 *       ___\////\\\___________\////\\\_____________\/\\\_____
 *        ______\////\\\___________\////\\\__________\/\\\_____
 *         _________\////\\\___________\////\\\_______\/\\\_____
 *          __/\\\______\//\\\___/\\\______\//\\\______\/\\\_____
 *           _\///\\\\\\\\\\\/___\///\\\\\\\\\\\/____/\\\\\\\\\\\_
 *            ___\///////////_______\///////////_____\///////////__
 *
 * *****************************************************************
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_can.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "pinmux.h"

#include "tiva-bootloader/fw_forceupdate.h"

/****************************************************************
 * START bootloader settings
 *
 * The bootloader uses two CAN message objects and a predfined
 * trigger message. When the trigger message is received on the
 * correct device ID, the force update function will kick in put
 * the device back into the bootloader.  Once in the bootloader
 * mode, the device cannot go back into the original firmware.
 * It must be programmed again.
 * **************************************************************
 */

// Device ID must be unique to each device on the CANBus
#define BOOTLOADER_DEVICEID    0x1000

// Bootloader mailbox and trigger message defines
#define BOOTLOADER_MB_RX    31
#define BOOTLOADER_MB_TX    32

// Define message IDs
#define BOOTLOADER_HEARTBEAT   0x18510000
#define BOOTLOADER_TRIGGER     0x18540000

const uint32_t g_u32CANHeartbeatID  = BOOTLOADER_HEARTBEAT | BOOTLOADER_DEVICEID;
const uint32_t g_u32CANTriggerID    = BOOTLOADER_TRIGGER   | BOOTLOADER_DEVICEID;
uint64_t g_ui64Heartbeat;

#define BOOTLOADER_IDENT0       0x55
#define BOOTLOADER_IDENT1       0x78
#define BOOTLOADER_IDENT2       0x71
#define BOOTLOADER_IDENT3       0xCB

/* **************************************************************
 * END bootloader settings
 * **************************************************************
 */

// Define communication speeds for CAN
#define CAN_BAUD        1000000

//*****************************************************************************
// Global Variables
//*****************************************************************************

bool g_bIndicator1;
bool g_bIndicator2;

//*****************************************************************************
// CAN and CAN Buffer setup
//*****************************************************************************

// CAN message objects that will hold the separate CAN messages
volatile bool g_bCAN0ErFlag = 0;            // CAN0 transmission error occurred

// Configure the CAN and its pins.
void ConfigureCAN(void)
{
    // Initialize the CAN controller
    CANInit(CAN0_BASE);

    // Set up the bit rate for the CAN bus.
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), CAN_BAUD);
}

// The interrupt handler for the CAN0
void CAN0IntHandler(void)
{
    uint32_t ui32Status;
    tCANMsgObject sCANMsgObjectRxProcess;
    uint8_t pui8MsgDataRx[8];

    // Read the CAN interrupt status to find the cause of the interrupt
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    // If the cause is a controller status interrupt, then get the status
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done.
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        // Set a flag to indicate some errors may have occurred.
        g_bCAN0ErFlag = 1;
    }
    // Mailbox 1 Rx interrupt
    else if(ui32Status == 1)
    {
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, 1);

        sCANMsgObjectRxProcess.pui8MsgData = pui8MsgDataRx;
        CANMessageGet(CAN0_BASE, 1, &sCANMsgObjectRxProcess, 0);

        g_bIndicator1 = (0B00000001 == (sCANMsgObjectRxProcess.pui8MsgData[0] & 0B00000001));

        // Since a message was received, clear any error flags.
        g_bCAN0ErFlag = 0;
    }
    // Mailbox 2 Rx interrupt
    else if(ui32Status == 2)
    {
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, 2);

        sCANMsgObjectRxProcess.pui8MsgData = pui8MsgDataRx;
        CANMessageGet(CAN0_BASE, 2, &sCANMsgObjectRxProcess, 0);

        g_bIndicator2 = (0B00000001 == (sCANMsgObjectRxProcess.pui8MsgData[0] & 0B00000001));

        g_bCAN0ErFlag = 0;
    }
    // Mailbox 11 Tx interrupt
    else if(ui32Status == 11)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object 11, and the message TX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, 11);

        // Since the message was sent, clear any error flags.
        g_bCAN0ErFlag = 0;
    }
    // Mailbox 12 Tx interrupt
    else if(ui32Status == 12)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object 12, and the message TX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, 12);

        // Since the message was sent, clear any error flags.
        g_bCAN0ErFlag = 0;
    }

    /*
     * START bootloader CAN interrupts
     */

    // Mailbox BOOTLOADER_MB_RX setup for Rx
    else if(ui32Status == BOOTLOADER_MB_RX)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object BOOTLOADER_MB_RX, and the message TX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, BOOTLOADER_MB_RX);

        sCANMsgObjectRxProcess.pui8MsgData = pui8MsgDataRx;
        CANMessageGet(CAN0_BASE, BOOTLOADER_MB_RX, &sCANMsgObjectRxProcess, 0);

        uint16_t ui16Update = (sCANMsgObjectRxProcess.pui8MsgData[0] << 8) | ( sCANMsgObjectRxProcess.pui8MsgData[1]);
        bool bUpdate = ( BOOTLOADER_DEVICEID == ui16Update );

        if ( bUpdate )
        {
            // TODO: DO SHUTDOWN OPERATIONS FIRST

            uint8_t status = InitForceUpdate();

            // TODO: handle status in case of failed update
        }

        // Since the message was sent, clear any error flags.
        g_bCAN0ErFlag = 0;
    }
    // Mailbox BOOTLOADER_MB_TX setup for Rx
    else if(ui32Status == BOOTLOADER_MB_TX)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object BOOTLOADER_MB_TX, and the message TX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, BOOTLOADER_MB_TX);

        // Since the message was sent, clear any error flags.
        g_bCAN0ErFlag = 0;
    }

    /*
     * END bootloader CAN interrupts
     */

    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    else
    {
        // Spurious interrupt handling can go here.
    }
}

//*****************************************************************************
// Timers Setup
//*****************************************************************************
volatile bool g_bTimer0Flag = 0;        // Timer 0 occurred flag

// Configure Timers
void ConfigureTimers(void)
{
    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 10);      // 10 Hz

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// The interrupt handler for the first timer interrupt. 1 Hz
void Timer0IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    g_bTimer0Flag = 1;      // Set flag to indicate Timer 0 interrupt
}

//*****************************************************************************
// Configure the interrupts
//*****************************************************************************
void ConfigureInterrupts(void)
{
    // Enable processor interrupts.
    IntMasterEnable();

    IntEnable(INT_CAN0);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    CANEnable(CAN0_BASE);
}

//*****************************************************************************
// Main code starts here
//*****************************************************************************
int main(void)
{
    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
            SYSCTL_XTAL_16MHZ);

    // Initialize and setup the ports
    PortFunctionInit();
    ConfigureTimers();
    ConfigureCAN();
    ConfigureInterrupts();

    //*****************************************************************************
    // CAN Setup
    //*****************************************************************************

    // Setup CAN Rx message IDs and Masks
    tCANMsgObject sCANMsgObjectRxSetup;

    // Now load the message object into the CAN peripheral message object 1.
    sCANMsgObjectRxSetup.ui32MsgLen = 8;
    sCANMsgObjectRxSetup.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID
            | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER );
    sCANMsgObjectRxSetup.ui32MsgIDMask    = 0x1FFFFFFF;
    sCANMsgObjectRxSetup.ui32MsgID        = 0x14FE1100;
    CANMessageSet(CAN0_BASE, 1, &sCANMsgObjectRxSetup, MSG_OBJ_TYPE_RX);

    // Now load the message object into the CAN peripheral message object 2.
    sCANMsgObjectRxSetup.ui32MsgLen = 8;
    sCANMsgObjectRxSetup.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID
            | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER );
    sCANMsgObjectRxSetup.ui32MsgIDMask    = 0x1FFFFFFF;
    sCANMsgObjectRxSetup.ui32MsgID        = 0x14FE1101;
    CANMessageSet(CAN0_BASE, 2, &sCANMsgObjectRxSetup, MSG_OBJ_TYPE_RX);

    /*
     * START bootloader Rx mailbox
     */

    // Now load the message object into the CAN peripheral message object BOOTLOADER_MB_RX.
    // Messages being received from APC
    sCANMsgObjectRxSetup.ui32MsgLen = 8;
    sCANMsgObjectRxSetup.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID
            | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER );
    sCANMsgObjectRxSetup.ui32MsgIDMask    = 0x1FFFFFFF;
    sCANMsgObjectRxSetup.ui32MsgID        = g_u32CANTriggerID;
    CANMessageSet(CAN0_BASE, BOOTLOADER_MB_RX, &sCANMsgObjectRxSetup, MSG_OBJ_TYPE_RX);

    /*
     * END bootloader Rx mailbox
     */

    // Loop forever while the timers run.
    while(1)
    {
        //*****************************************************************************
        // Timers
        //*****************************************************************************

        // Timer 0
        if ( g_bTimer0Flag )
        {
            g_bTimer0Flag = 0;
            g_ui64Heartbeat++;

            if ( g_bIndicator1 )
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            }

            if ( g_bIndicator2 )
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            }

            //*****************************************************************************
            // CAN transmit code here
            //*****************************************************************************

            // Message buffers for message content
            tCANMsgObject sCANMsgObjectTx;
            uint8_t pui8CanDataTx[8];

            // Send data using object 11
            pui8CanDataTx[0] = 0x00;
            pui8CanDataTx[1] = 0x01;
            pui8CanDataTx[2] = 0x02;
            pui8CanDataTx[3] = 0x03;
            pui8CanDataTx[4] = 0x04;
            pui8CanDataTx[5] = 0x05;
            pui8CanDataTx[6] = 0x06;
            pui8CanDataTx[7] = 0x07;

            // Setup CAN Tx message object
            sCANMsgObjectTx.ui32MsgIDMask = 0;
            sCANMsgObjectTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
            sCANMsgObjectTx.ui32MsgID = 0x14FE1102;
            sCANMsgObjectTx.pui8MsgData = pui8CanDataTx;
            sCANMsgObjectTx.ui32MsgLen = sizeof(pui8CanDataTx);
            CANMessageSet(CAN0_BASE, 11, &sCANMsgObjectTx, MSG_OBJ_TYPE_TX);

            // minor delay
            SysCtlDelay(SysCtlClockGet()/1500); // Delay 2 ms

            // Send data using object 12
            pui8CanDataTx[0] = 0x00;
            pui8CanDataTx[1] = 0x01;
            pui8CanDataTx[2] = 0x02;
            pui8CanDataTx[3] = 0x03;
            pui8CanDataTx[4] = 0x04;
            pui8CanDataTx[5] = 0x05;
            pui8CanDataTx[6] = 0x06;
            pui8CanDataTx[7] = 0x07;

            // Setup CAN Tx general message objects
            sCANMsgObjectTx.ui32MsgIDMask = 0;
            sCANMsgObjectTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
            sCANMsgObjectTx.ui32MsgID = 0x14FE1103;
            sCANMsgObjectTx.pui8MsgData = pui8CanDataTx;
            sCANMsgObjectTx.ui32MsgLen = sizeof(pui8CanDataTx);
            CANMessageSet(CAN0_BASE, 12, &sCANMsgObjectTx, MSG_OBJ_TYPE_TX);

            // minor delay
            SysCtlDelay(SysCtlClockGet()/1500); // Delay 2 ms

            /*
             * START bootloader heartbeat
             */

            // Send heartbeat for use by bootloader
            pui8CanDataTx[0] = ((uint8_t) (((uint16_t) (g_ui64Heartbeat)) >> 8));
            pui8CanDataTx[1] = ((uint8_t) (g_ui64Heartbeat));
            pui8CanDataTx[2] = 0xFF;
            pui8CanDataTx[3] = 0xFF;
            pui8CanDataTx[4] = BOOTLOADER_IDENT0;
            pui8CanDataTx[5] = BOOTLOADER_IDENT1;
            pui8CanDataTx[6] = BOOTLOADER_IDENT2;
            pui8CanDataTx[7] = BOOTLOADER_IDENT3;

            // Setup CAN Tx general message objects
            sCANMsgObjectTx.ui32MsgIDMask = 0;
            sCANMsgObjectTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
            sCANMsgObjectTx.ui32MsgID = g_u32CANHeartbeatID;
            sCANMsgObjectTx.pui8MsgData = pui8CanDataTx;
            sCANMsgObjectTx.ui32MsgLen = sizeof(pui8CanDataTx);
            CANMessageSet(CAN0_BASE, BOOTLOADER_MB_TX, &sCANMsgObjectTx, MSG_OBJ_TYPE_TX);

            /*
             * END bootloader heartbeat
             */
        }
    }
}
