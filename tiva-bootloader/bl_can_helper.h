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

#ifndef __BL_CAN_HELPER_H__
#define __BL_CAN_HELPER_H__

#include <stdint.h>
#include <stdbool.h>

// mailboxes for LM_UPD_API messages
#define mb_LM_API_UPD_PING      20
#define mb_LM_API_UPD_DOWNLOAD  21
#define mb_LM_API_UPD_SEND_DATA 22
#define mb_LM_API_UPD_RESET     23
#define mb_LM_API_UPD_ACK       24
#define mb_LM_API_UPD_REQUEST   25
#define mb_LM_FIXME             30

// command and mailbox for forcing update by
// invalidating app code
#define cmd_INVALIDATE_APP_CODE 0x14FE1101
#define mb_INVALIDATE_APP_CODE  26

//*****************************************************************************
// Configure Rx message object for CAN0 with typical settings
//*****************************************************************************
void ConfigureAndSetRxMessageObject(uint32_t ui32MsgID, uint32_t ui32ObjID);

//*****************************************************************************
// Configure message object for CAN0 with typical settings
//*****************************************************************************
void ConfigureAndSetTxMessageObject(uint32_t ui32MsgID, uint32_t ui32ObjID, uint8_t* pui8CanDataTx, uint32_t ui32MsgLen);

//*****************************************************************************
// Configure message objects for CAN0 with known bl_can messages
//*****************************************************************************
void ConfigureCANBL(void);

//*****************************************************************************
// Handle bl_can messages
//*****************************************************************************
void HandleCANBLMSG(uint32_t msgID);

#endif // __BL_CAN_HELPER_H__
