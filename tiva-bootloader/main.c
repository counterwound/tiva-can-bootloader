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
#include "inc/hw_nvic.h"
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

/****************************************************************
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

// Declared and implemented in utils.s
extern void CallApplication(void);

// Bootloader mailbox and trigger message defines
#define BOOTLOADER_MB_RX    31
#define BOOTLOADER_MB_TX    32

// Define message IDs
#define BOOTLOADER_HEARTBEAT   0x18500000

const uint32_t g_u32CANHeartbeatID = BOOTLOADER_HEARTBEAT | BOOTLOADER_DEVICEID;
uint64_t g_ui64Heartbeat;

#define BOOTLOADER_IDENT0       0x94
#define BOOTLOADER_IDENT1       0xE1
#define BOOTLOADER_IDENT2       0xA5
#define BOOTLOADER_IDENT3       0x10

/* **************************************************************
 * End bootloader settings
 * **************************************************************
 */

// Define communication speeds for CAN
#define CAN_BAUD        1000000

//*****************************************************************************
// Global Variables
//*****************************************************************************

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
    else
    {
        CANIntClear(CAN0_BASE, ui32Status);
        switch(ui32Status)
        {
            case BOOTLOADER_MB_TX:
                // Getting to this point means that the TX interrupt occurred on
                // message object BOOTLOADER_MB_TX, and the message TX is complete.  Clear the
                // message object interrupt.

                // Since the message was sent, clear any error flags.
                g_bCAN0ErFlag = 0;
                break;

            case mb_LM_API_UPD_PING:        // 20
            case mb_LM_API_UPD_DOWNLOAD:    // 21
            case mb_LM_API_UPD_SEND_DATA:   // 22
            case mb_LM_API_UPD_RESET:       // 23
                // let the handler function take these
                HandleCANBLMSG(ui32Status);
                break;

            case mb_LM_FIXME:
                // Getting to this point means that the TX interrupt occurred on
                // message object mb_LM_FIXME, and the message TX is complete.  Clear the
                // message object interrupt.

                // FIXME why does application break when these lines are removed
                sCANMsgObjectRxProcess.pui8MsgData = pui8MsgDataRx;
                CANMessageGet(CAN0_BASE, ui32Status, &sCANMsgObjectRxProcess, 0);

                // Since a message was received, clear any error flags.
                g_bCAN0ErFlag = 0;
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
    if( CheckForceUpdate() )
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

            if ( 0B0001 == (g_ui64Heartbeat & 0B1111 ) )
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            }

            //*****************************************************************************
            // CAN transmit code here
            //*****************************************************************************

            // Message buffers for message content
            uint8_t pui8CanDataTx[8];

            // Send heartbeat for use by bootloader
            pui8CanDataTx[0] = ((uint8_t) (((uint16_t) (g_ui64Heartbeat)) >> 8));
            pui8CanDataTx[1] = ((uint8_t) (g_ui64Heartbeat));
            pui8CanDataTx[2] = 0xFF;
            pui8CanDataTx[3] = 0xFF;
            pui8CanDataTx[4] = BOOTLOADER_IDENT0;
            pui8CanDataTx[5] = BOOTLOADER_IDENT1;
            pui8CanDataTx[6] = BOOTLOADER_IDENT2;
            pui8CanDataTx[7] = BOOTLOADER_IDENT3;

            ConfigureAndSetTxMessageObject(g_u32CANHeartbeatID, BOOTLOADER_MB_TX, pui8CanDataTx, sizeof(pui8CanDataTx));
        }
    }
}
