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
 *  some content based on Tiva Firmware Development Package
 *  see below for relevant copyright notices
 * *****************************************************************
 */

//*****************************************************************************
//
// Copyright (c) 2008-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include "bl_can_helper.h"
#include "bl_can.h"
#include "bl_config.h"
#include "bl_flash.h"
#include "driverlib/can.h"
#include "inc/hw_flash.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

// declared and implemented in utils.s
extern void CallApplication(void);

//*****************************************************************************
//
// The results that can be returned by the CAN APIs.
// Note: from bl_can.c
//
//*****************************************************************************
#define CAN_CMD_SUCCESS         0x00
#define CAN_CMD_FAIL            0x01

//*****************************************************************************
//
// Holds the current address to write to when data is received via the Send
// Data Command.
// Note: from bl_can.c
//
//*****************************************************************************
static uint32_t g_ui32TransferAddress;

//*****************************************************************************
//
// Holds the remaining bytes expected to be received.
// Note: from bl_can.c
//
//*****************************************************************************
static uint32_t g_ui32TransferSize;

//*****************************************************************************
//
// These globals are used to store the first two words to prevent a partial
// image from being booted.
// Note: from bl_can.c
//
//*****************************************************************************
static uint32_t g_ui32StartValues[2];
static uint32_t g_ui32StartSize;
static uint32_t g_ui32StartAddress;


//*****************************************************************************
// Configure Tx message object ID
// Note: make sure this is always handled in main.c
//*****************************************************************************
#define CANBLTxObjID 31
#define CANBL_BASE CAN0_BASE

//*****************************************************************************
// Configure Rx message object for CAN0 with typical settings
//*****************************************************************************
void ConfigureAndSetRxMessageObject(uint32_t ui32MsgID, uint32_t ui32ObjID)
{
    // Setup CAN Rx message IDs and Masks
    tCANMsgObject sCANMsgObjectRx;

    // Now load the message object into the CAN peripheral message object 1.
    // Messages being received from Battery Charger
    sCANMsgObjectRx.ui32MsgLen = 8;
    sCANMsgObjectRx.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID
            | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER );
    sCANMsgObjectRx.ui32MsgIDMask    = 0x1FFFFFFF;
    sCANMsgObjectRx.ui32MsgID        =  ui32MsgID;
    CANMessageSet(CANBL_BASE, ui32ObjID, &sCANMsgObjectRx, MSG_OBJ_TYPE_RX);
}

//*****************************************************************************
// Configure message object for CAN0 with typical settings
//*****************************************************************************
void ConfigureAndSetTxMessageObject(uint32_t ui32MsgID, uint32_t ui32ObjID, uint8_t* pui8CanDataTx, uint32_t ui32MsgLen)
{
    // Message buffers for message content
    tCANMsgObject sCANMsgObjectTx;

    // Setup CAN Tx general message objects
    sCANMsgObjectTx.ui32MsgIDMask = 0;
    sCANMsgObjectTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
    sCANMsgObjectTx.ui32MsgID = ui32MsgID;
    sCANMsgObjectTx.pui8MsgData = pui8CanDataTx;
    sCANMsgObjectTx.ui32MsgLen = ui32MsgLen;
    CANMessageSet(CANBL_BASE, ui32ObjID, &sCANMsgObjectTx, MSG_OBJ_TYPE_TX);
}

//*****************************************************************************
// Configure message objects for CAN0 with known bl_can messages
//*****************************************************************************
void ConfigureCANBL(void)
{
    // set message objects for the messages supported by the booloader
    ConfigureAndSetRxMessageObject((uint32_t)LM_API_UPD_PING, (uint32_t)mb_LM_API_UPD_PING);
    ConfigureAndSetRxMessageObject((uint32_t)LM_API_UPD_DOWNLOAD, (uint32_t)mb_LM_API_UPD_DOWNLOAD);
    ConfigureAndSetRxMessageObject((uint32_t)LM_API_UPD_SEND_DATA, (uint32_t)mb_LM_API_UPD_SEND_DATA);
    ConfigureAndSetRxMessageObject((uint32_t)LM_API_UPD_RESET, (uint32_t)mb_LM_API_UPD_RESET);
    ConfigureAndSetRxMessageObject((uint32_t)LM_API_UPD_ACK, (uint32_t)mb_LM_API_UPD_ACK);
    ConfigureAndSetRxMessageObject((uint32_t)LM_API_UPD_REQUEST, (uint32_t)mb_LM_API_UPD_REQUEST);
    ConfigureAndSetRxMessageObject((uint32_t)cmd_INVALIDATE_APP_CODE, (uint32_t)mb_INVALIDATE_APP_CODE);
}

//*****************************************************************************
// Handle bl_can messages
//*****************************************************************************
void HandleCANBLMSG(uint32_t msgID)
{
    tCANMsgObject sCANMessageRx;
    uint8_t pui8MsgDataRx[8];
    uint8_t pui8MsgDataTx[8];

    switch(msgID)
    {
        case mb_LM_API_UPD_PING:
            // LM_API_UPD_PING

            /*
                LM_API_UPD_PING This command is used to receive an acknowledge command
                from the boot loader indicating that communication has been established.
                This command has no data. If the device is present it
                will respond with a LM_API_UPD_PING back to the CAN update
                application.
             */

            sCANMessageRx.pui8MsgData = pui8MsgDataRx;
            CANMessageGet(CANBL_BASE, mb_LM_API_UPD_PING, &sCANMessageRx, 0);

            ConfigureAndSetTxMessageObject(LM_API_UPD_PING, CANBLTxObjID, pui8MsgDataTx, 0);
            // no need to LM_API_UPD_ACK this packet, so just return here
            break;

        case mb_LM_API_UPD_RESET:
            // LM_API_UPD_RESET

            /*
                LM_API_UPD_RESET This command is used to tell the CAN boot loader to reset the
                microcontroller. This is used after downloading a new image to
                the microcontroller to cause the new application or the new boot
                loader to start from a reset. The normal boot sequence occurs
                and the image runs as if from a hardware reset. It can also
                be used to reset the boot loader if a critical error occurs and the
                CAN update application needs to restart communication with the
                boot loader.

             */

            //
            // Perform a software reset request.  This will cause the
            // microcontroller to reset; no further code will be executed.
            //
            HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
                                 NVIC_APINT_SYSRESETREQ);

            //
            // The microcontroller should have reset, so this should never
            // be reached.  Just in case, loop forever.
            //
            while(1)
            {
            }

        //
        // This is a start download packet.
        //
        case mb_LM_API_UPD_DOWNLOAD:
            // LM_API_UPD_DOWNLOAD

            /*
                LM_API_UPD_DOWNLOAD This command sets the base address for the download as well
                as the size of the data to write to the device. This command
                should be followed by a series of LM_API_UPD_SEND_DATA that
                send the actual image to be programmed to the device. The
                command consists of two 32-bit values that are transferred LSB
                first. The first 32-bit value is the address to start programming
                data into, while the second is the 32-bit size of the data that
                will be sent. This command also triggers an erasure of the full
                application area in the flash. This flash erase operation causes
                the command to take longer to send the LM_API_UPD_ACK in
                response to the command which should be taken into account
                by the CAN update application.
                The format of the command is as follows:
                unsigned char ucData[8];
                ucData[0] = Download Address [7:0];
                ucData[1] = Download Address [15:8];
                ucData[2] = Download Address [23:16];
                ucData[3] = Download Address [31:24];
                ucData[4] = Download Size [7:0];
                ucData[5] = Download Size [15:8];
                ucData[6] = Download Size [23:16];
                ucData[7] = Download Size [31:24];
             */
            {
                uint32_t ui32FlashSize;
                uint32_t ui32Temp;
                uint8_t ui8Status;
                //
                // Get the application address and size from the packet data.
                //
                sCANMessageRx.pui8MsgData = pui8MsgDataRx;
                CANMessageGet(CANBL_BASE, mb_LM_API_UPD_DOWNLOAD, &sCANMessageRx, 0);
                /* orig
                g_ui32TransferAddress =
                    *((uint32_t *)&g_pui8CommandBuffer[0]);
                g_ui32TransferSize = *((uint32_t *)&g_pui8CommandBuffer[4]);
                g_ui32StartSize = g_ui32TransferSize;
                g_ui32StartAddress = g_ui32TransferAddress;
                */
                g_ui32TransferAddress =
                    *((uint32_t *)&sCANMessageRx.pui8MsgData[0]);
                g_ui32TransferSize = *((uint32_t *)&sCANMessageRx.pui8MsgData[4]);
                g_ui32StartSize = g_ui32TransferSize;
                g_ui32StartAddress = g_ui32TransferAddress;

                //
                // Check for a valid starting address and image size.
                //
                if(!BL_FLASH_AD_CHECK_FN_HOOK(g_ui32TransferAddress,
                                              g_ui32TransferSize))
                {
                    //
                    // Set the code to an error to indicate that the last
                    // command failed.  This informs the updater program
                    // that the download command failed.
                    //
                    ui8Status = CAN_CMD_FAIL;

                    //
                    // This packet has been handled.
                    //
                    // LM_API_UPD_ACK, with ui8Status
                    pui8MsgDataTx[0] = ui8Status;
                    ConfigureAndSetTxMessageObject(LM_API_UPD_ACK, CANBLTxObjID, pui8MsgDataTx, 1);
                    break;
                }

                ui32FlashSize = g_ui32TransferAddress + g_ui32TransferSize;

                //
                // Clear the flash access interrupt.
                //
                BL_FLASH_CL_ERR_FN_HOOK();

                //
                // Leave the boot loader present until we start getting an
                // image.
                //
                for(ui32Temp = g_ui32TransferAddress; ui32Temp < ui32FlashSize;
                    ui32Temp += FLASH_PAGE_SIZE)
                {
                    //
                    // Erase this block.
                    //
                    BL_FLASH_ERASE_FN_HOOK(ui32Temp);
                }

                //
                // Return an error if an access violation occurred.
                //
                if(BL_FLASH_ERROR_FN_HOOK())
                {
                    ui8Status = CAN_CMD_FAIL;
                }

                //
                // See if the command was successful.
                //
                if(ui8Status != CAN_CMD_SUCCESS)
                {
                    //
                    // Setting g_ui32TransferSize to zero makes
                    // COMMAND_SEND_DATA fail to accept any data.
                    //
                    g_ui32TransferSize = 0;
                }

                // LM_API_UPD_ACK, with ui8Status
                pui8MsgDataTx[0] = ui8Status;
                ConfigureAndSetTxMessageObject(LM_API_UPD_ACK, CANBLTxObjID, pui8MsgDataTx, 1);
                break;
            }

        //
        // This is a data packet.
        //
        case mb_LM_API_UPD_SEND_DATA:

            // LM_API_UPD_SEND_DATA

            /*
                LM_API_UPD_SEND_DATA This command should only follow a LM_API_UPD_DOWNLOAD
                command or another LM_API_UPD_SEND_DATA command
                when more data is needed. Consecutive send data commands
                automatically increment the address and continue programming
                from the previous location. The transfer size is limited to 8 bytes
                at a time based on the maximum size of an individual CAN
                transmission. The command terminates programming once the
                number of bytes indicated by the LM_API_UPD_DOWNLOAD command
                have been received. The CAN boot loader will send a
                LM_API_UPD_ACK in response to each send data command to
                allow the CAN update application to throttle the data going to the
                device and not overrun the boot loader with data.
             */
            {
                uint32_t ui32Temp;
                uint8_t ui8Status;
                uint32_t ui32Bytes;

                CANMessageGet(CANBL_BASE, mb_LM_API_UPD_SEND_DATA, &sCANMessageRx, 0);
                ui32Bytes = sCANMessageRx.ui32MsgLen;

                //
                // If this is overwriting the boot loader then the application
                // has already been erased so now erase the boot loader.
                //
                if(g_ui32TransferAddress == 0)
                {
                    //
                    // Clear the flash access interrupt.
                    //
                    BL_FLASH_CL_ERR_FN_HOOK();

                    //
                    // Erase the application before the boot loader.
                    //
                    for(ui32Temp = 0; ui32Temp < APP_START_ADDRESS;
                        ui32Temp += FLASH_PAGE_SIZE)
                    {
                        //
                        // Erase this block.
                        //
                        BL_FLASH_ERASE_FN_HOOK(ui32Temp);
                    }

                    //
                    // Return an error if an access violation occurred.
                    //
                    if(BL_FLASH_ERROR_FN_HOOK())
                    {
                        //
                        // Setting g_ui32TransferSize to zero makes
                        // COMMAND_SEND_DATA fail to accept any more data.
                        //
                        g_ui32TransferSize = 0;

                        //
                        // Indicate that the flash erase failed.
                        //
                        ui8Status = CAN_CMD_FAIL;
                    }
                }

                //
                // Check if there are any more bytes to receive.
                //
                if(g_ui32TransferSize >= ui32Bytes)
                {
                    //
                    // Decrypt the data if required.
                    //
#ifdef BL_DECRYPT_FN_HOOK
                    BL_DECRYPT_FN_HOOK(g_pui8CommandBuffer, ui32Bytes);
#endif

                    //
                    // Clear the flash access interrupt.
                    //
                    BL_FLASH_CL_ERR_FN_HOOK();

                    //
                    // Skip the first transfer.
                    //
                    if(g_ui32StartSize == g_ui32TransferSize)
                    {
                        g_ui32StartValues[0] =
                            *((uint32_t *)&sCANMessageRx.pui8MsgData[0]);
                        g_ui32StartValues[1] =
                            *((uint32_t *)&sCANMessageRx.pui8MsgData[4]);
                    }
                    else
                    {
                        //
                        // Loop over the words to program.
                        //
                        BL_FLASH_PROGRAM_FN_HOOK(g_ui32TransferAddress,
                                                 sCANMessageRx.pui8MsgData,
                                                 ui32Bytes);
                    }

                    //
                    // Return an error if an access violation occurred.
                    //
                    if(BL_FLASH_ERROR_FN_HOOK())
                    {
                        //
                        // Indicate that the flash programming failed.
                        //
                        ui8Status = CAN_CMD_FAIL;
                    }
                    else
                    {
                        //
                        // Now update the address to program.
                        //
                        g_ui32TransferSize -= ui32Bytes;
                        g_ui32TransferAddress += ui32Bytes;

                        //
                        // If a progress hook function has been provided, call
                        // it here.
                        //
#ifdef BL_PROGRESS_FN_HOOK
                        BL_PROGRESS_FN_HOOK(g_ui32StartSize -
                                            g_ui32TransferSize,
                                            g_ui32StartSize);
#endif
                    }
                }
                else
                {
                    //
                    // This indicates that too much data is being sent to the
                    // device.
                    //
                    ui8Status = CAN_CMD_FAIL;
                }

                //
                // If the last expected bytes were received then write out the
                // first two words of the image to allow it to boot.
                //
                if(g_ui32TransferSize == 0)
                {
                    //
                    // Loop over the words to program.
                    //
                    BL_FLASH_PROGRAM_FN_HOOK(g_ui32StartAddress,
                                             (uint8_t *)&g_ui32StartValues,
                                             8);

                    //
                    // If an end signal hook function has been provided, call
                    // it here since we have finished a download.
                    //
#ifdef BL_END_FN_HOOK
                    BL_END_FN_HOOK();
#endif
                }
                // LM_API_UPD_ACK, with ui8Status
                pui8MsgDataTx[0] = ui8Status;
                ConfigureAndSetTxMessageObject(LM_API_UPD_ACK, CANBLTxObjID, pui8MsgDataTx, 1);
                break;
            }

        default:
            // not supported yet
            break;
    }
}
