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
// Copyright (c) 2007-2015 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.1.71 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include "fw_forceupdate.h"
#include "bl_flash.h"
#include "inc/hw_flash.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

// starting address of application; must match setting of in-use bootloader
#define APP_START_ADDRESS 0x00002000

uint8_t InitForceUpdate(void)
{
    // erase the first block of the application space
    {
        // Clear the flash access interrupt.
        BL_FLASH_CL_ERR_FN_HOOK();

        // Erase the first application block
        BL_FLASH_ERASE_FN_HOOK(APP_START_ADDRESS);

        // Return an error if an access violation occurred.
        if(BL_FLASH_ERROR_FN_HOOK())
        {
            return 1;
        }
    }

    // now reset, waiting in bootloader for update
    {
        // Perform a software reset request.
        HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
                             NVIC_APINT_SYSRESETREQ);

        // Just in case, loop forever.
        while(1)
        {
        }
    }
}
