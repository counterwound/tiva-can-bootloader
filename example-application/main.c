/******************************************************************
 * Tiva CANbus Bootloader Example
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

// Extra low and high bytes from uint16_t
#define LOWBYTE(v)   ((uint8_t) (v))
#define HIGHBYTE(v)  ((uint8_t) (((uint16_t) (v)) >> 8))

// Define communication speeds for CAN
#define CAN_BAUD        1000000

//*****************************************************************************
// Global Variables
//*****************************************************************************

bool g_bIndicator1;
bool g_bIndicator2;

uint64_t g_ui64Heartbeat;

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
    tCANMsgObject sCANMessageRx;
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
    else if(ui32Status == 1)
    {
        // Battery fault message
        CANIntClear(CAN0_BASE, 1);

        sCANMessageRx.pui8MsgData = pui8MsgDataRx;
        CANMessageGet(CAN0_BASE, 1, &sCANMessageRx, 0);

        g_bIndicator1 = (0B00000001 == (sCANMessageRx.pui8MsgData[0] & 0B00000001));

        // Since a message was received, clear any error flags.
        g_bCAN0ErFlag = 0;
    }
    else if(ui32Status == 2)
    {
        // Cap charger commands
        CANIntClear(CAN0_BASE, 2);

        sCANMessageRx.pui8MsgData = pui8MsgDataRx;
        CANMessageGet(CAN0_BASE, 2, &sCANMessageRx, 0);

        g_bIndicator2 = (0B00000001 == (sCANMessageRx.pui8MsgData[0] & 0B00000001));

        g_bCAN0ErFlag = 0;
    }
    // Check if the cause is message object 10, which is used for sending
    // message 10.
    else if(ui32Status == 10)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object 10, and the message TX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, 10);

        // Since the message was sent, clear any error flags.
        g_bCAN0ErFlag = 0;
    }
    // Check if the cause is message object 11, which is used for sending
    // message 11.
    else if(ui32Status == 11)
    {
        // Getting to this point means that the TX interrupt occurred on
        // message object 11, and the message TX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CAN0_BASE, 11);

        // Since the message was sent, clear any error flags.
        g_bCAN0ErFlag = 0;
    }
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
    tCANMsgObject sCANMsgObjectRx;

    // Now load the message object into the CAN peripheral message object 1.
    // Messages being received from Battery Charger
    sCANMsgObjectRx.ui32MsgLen = 8;
    sCANMsgObjectRx.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID
            | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER );
    sCANMsgObjectRx.ui32MsgIDMask    = 0x1FFFFFFF;
    sCANMsgObjectRx.ui32MsgID        = 0x14FE1100;
    CANMessageSet(CAN0_BASE, 1, &sCANMsgObjectRx, MSG_OBJ_TYPE_RX);

    // Now load the message object into the CAN peripheral message object 3.
    // Messages being received from APC
    sCANMsgObjectRx.ui32MsgLen = 8;
    sCANMsgObjectRx.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID
            | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER );
    sCANMsgObjectRx.ui32MsgIDMask    = 0x1FFFFFFF;
    sCANMsgObjectRx.ui32MsgID        = 0x14FE1101;
    CANMessageSet(CAN0_BASE, 2, &sCANMsgObjectRx, MSG_OBJ_TYPE_RX);

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
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            }

            if ( g_bIndicator2 )
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            }

            //*****************************************************************************
            // CAN transmit code here
            //*****************************************************************************

            // Message buffers for message content
            tCANMsgObject sCANMsgObjectTx;
            uint8_t pui8CanDataTx[8];

            // Send the CC Heartbeat
            pui8CanDataTx[0] = HIGHBYTE(g_ui64Heartbeat);
            pui8CanDataTx[1] = LOWBYTE(g_ui64Heartbeat);
            pui8CanDataTx[2] = 0xFF;
            pui8CanDataTx[3] = 0xFF;
            pui8CanDataTx[4] = 0xFF;
            pui8CanDataTx[5] = 0xFF;
            pui8CanDataTx[6] = 0xFF;
            pui8CanDataTx[7] = 0xFF;

            // Setup CAN Tx general message objects
            sCANMsgObjectTx.ui32MsgIDMask = 0;
            sCANMsgObjectTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
            sCANMsgObjectTx.ui32MsgID = 0x14FE1000;
            sCANMsgObjectTx.pui8MsgData = pui8CanDataTx;
            sCANMsgObjectTx.ui32MsgLen = sizeof(pui8CanDataTx);
            CANMessageSet(CAN0_BASE, 10, &sCANMsgObjectTx, MSG_OBJ_TYPE_TX);

            // minor delay
            SysCtlDelay(SysCtlClockGet()/1500); // Delay 2 ms

            // Send the CC Heartbeat
            pui8CanDataTx[0] = 0xFF;
            pui8CanDataTx[1] = 0xFF;
            pui8CanDataTx[2] = 0xFF;
            pui8CanDataTx[3] = 0xFF;
            pui8CanDataTx[4] = 0xFF;
            pui8CanDataTx[5] = 0xFF;
            pui8CanDataTx[6] = 0xFF;
            pui8CanDataTx[7] = 0xFF;

            // Setup CAN Tx general message objects
            sCANMsgObjectTx.ui32MsgIDMask = 0;
            sCANMsgObjectTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
            sCANMsgObjectTx.ui32MsgID = 0x14FE1001;
            sCANMsgObjectTx.pui8MsgData = pui8CanDataTx;
            sCANMsgObjectTx.ui32MsgLen = sizeof(pui8CanDataTx);
            CANMessageSet(CAN0_BASE, 11, &sCANMsgObjectTx, MSG_OBJ_TYPE_TX);
        }
    }
}
