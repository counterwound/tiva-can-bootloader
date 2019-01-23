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

#include "bl_can.h"
#include "bl_can_helper.h"
#include "bl_check.h"
#include "bl_config.h"
#include "fw_forceupdate.h"
#include "inc/hw_nvic.h"

// declared and implemented in utils.s
extern void CallApplication(void);

// Define message IDs
#define CAN_DEVICEID    0x1000
#define CAN_HEARTBEAT   0x18700000

// Extra low and high bytes from uint16_t
#define LOWBYTE(v)   ((uint8_t) (v))
#define HIGHBYTE(v)  ((uint8_t) (((uint16_t) (v)) >> 8))

// Define communication speeds for CAN
#define CAN_BAUD        1000000

//*****************************************************************************
// Global Variables
//*****************************************************************************
const uint32_t g_u32CANHeartbeatID = CAN_HEARTBEAT | CAN_DEVICEID;

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
    //uint8_t pui8MsgDataTx[8];

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
    else
    {
        CANIntClear(CAN0_BASE, ui32Status);
        switch(ui32Status)
        {
            case 1:
                // Getting to this point means that the RX interrupt occurred on
                // message object 1.  Clear the message object interrupt.

                // Since the message was received, clear any error flags.
                sCANMessageRx.pui8MsgData = pui8MsgDataRx;
                CANMessageGet(CAN0_BASE, ui32Status, &sCANMessageRx, 0);

                // Since a message was received, clear any error flags.
                g_bCAN0ErFlag = 0;
                break;

//            case 2:
//                // Getting to this point means that the RX interrupt occurred on
//                // message object 2.  Clear the message object interrupt.
//
//                // Since the message was received, clear any error flags.
//                sCANMessageRx.pui8MsgData = pui8MsgDataRx;
//                CANMessageGet(CAN0_BASE, ui32Status, &sCANMessageRx, 0);
//
//                g_bCAN0ErFlag = 0;
//                break;

            case 3:
            {
                // TODO: DO SHUTDOWN OPERATIONS FIRST

                uint8_t status = InitForceUpdate();

                // TODO: handle status in case of failed update
                break;
            }

            case 10:
                // Getting to this point means that the TX interrupt occurred on
                // message object 10, and the message TX is complete.  Clear the
                // message object interrupt.

                // Since the message was sent, clear any error flags.
                g_bCAN0ErFlag = 0;
                break;

//            case 11:
//                // Getting to this point means that the TX interrupt occurred on
//                // message object 11, and the message TX is complete.  Clear the
//                // message object interrupt.
//
//                // Since the message was sent, clear any error flags.
//                g_bCAN0ErFlag = 0;
//                break;

            case mb_LM_API_UPD_PING:
            case mb_LM_API_UPD_DOWNLOAD:
            case mb_LM_API_UPD_SEND_DATA:
            case mb_LM_API_UPD_RESET:
                // let the handler function take these
                HandleCANBLMSG(ui32Status);

                break;

            default:
                // Otherwise, something unexpected caused the interrupt.  This should
                // never happen.
                // Spurious interrupt handling can go here.
                break;
        }
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

    // check if updates are required
    if(CheckForceUpdate())
    {
        // execute this bootloader if an update is needed
        // no_op
    }
    else
    {
        // if not, boot the program code
        CallApplication();
    }

    // Initialize and setup the ports
    PortFunctionInit();
    ConfigureTimers();
    ConfigureCAN();
    ConfigureInterrupts();

    //*****************************************************************************
    // CAN Setup
    //*****************************************************************************
    // set message objects for the existing HMI
//    ConfigureAndSetRxMessageObject(0x14FE1100, 1);
//    ConfigureAndSetRxMessageObject(0x14FE1101, 2);

    // set message object for InitForceUpdate demo
    ConfigureAndSetRxMessageObject(0x1DEDBEEF, 3);

    // set message objects for bootloader commands
    ConfigureCANBL();

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
            g_ui64Heartbeat--;

            if ( g_bIndicator1 )
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            }
            g_bIndicator1 = !g_bIndicator1;

            if ( g_bIndicator2 )
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            }
            g_bIndicator2 = !g_bIndicator2;

            //*****************************************************************************
            // CAN transmit code here
            //*****************************************************************************

            // buffers for message content
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

            ConfigureAndSetTxMessageObject(g_u32CANHeartbeatID, 10, pui8CanDataTx, sizeof(pui8CanDataTx));

            // minor delay
            SysCtlDelay(SysCtlClockGet()/1500); // Delay 2 ms

        }
    }
}
