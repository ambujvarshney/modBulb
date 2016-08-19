/* main.c
 *
 * Example that illustrates the use of the modulation interface functions of CC3200 device.
 *
 * Copyright (C) 2016 Uppsala Netowrked Objects.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * 
 * * Neither the name of the  nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Contributors: Abdalah Hilmia, Kasun Hewage and Ambuj Varshney.
 * 
 */

// standard library includes
#include <stdlib.h>
#include <stdint.h>

// fix __CONCAT redefined warning
#ifdef __CONCAT
#undef __CONCAT
#endif
// simplelink includes
#include "simplelink.h"
#include "wlan.h"

// Driverlib includes
#include "hw_types.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "hw_timer.h"
#include "hw_gpio.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "timer.h"
#include "utils.h"
#include "pin.h"
#include "gpio.h"

// commom interface includes
#include "mod_if.h"
#include "network_if.h"
#ifndef NOTERM
#include "uart_if.h"
#endif
#include "common.h"

#define SOCKET_PORT         5001                ///< The port to bind the UDP socket to.

#define GPIO_PIN            10                  ///< Output signal GPIO pin number.
#define PKG_PIN             PIN_01              ///< Output signal package pin number.

#define BUFF_LEN            11000               ///< Length of RX buffer.

#define CMD_IND_MASK        0xF0                ///< Command indicator mask.
#define CMD_IND_VAL         0xE0                ///< Command indicator value.

#define CMD_INIT_VAL        0x07                ///< Initialize command value.
#define CMD_MOD_VAL         0x0A                ///< Modulate command value.

#define CMD_IND         u8Buffer[0]             ///< Location of command indicator in RX buffer.

#define CMD_SCHEME      u8Buffer[1]             ///< Location of modulation scheme in RX buffer.
#define CMD_DEVICE      u8Buffer[2]             ///< Location of device in RX buffer.
#define CMD_BFSKF1      u8Buffer[3]             ///< Location of BFSK freq1 in RX buffer.
#define CMD_BFSKF2      u8Buffer[7]             ///< Location of BFSK freq2 in RX buffer.
#define CMD_DUTY        u8Buffer[11]            ///< Location of BFSK duty cycle in RX buffer.
#define CMD_PPMBS       u8Buffer[12]            ///< Location of PPM bit/symbol in RX buffer.

#define CMD_BITRATE     u8Buffer[1]             ///< Location of modulation datarate in RX buffer.
#define CMD_DLEN        u8Buffer[5]             ///< Location of data length in RX buffer.
#define CMD_DATA        u8Buffer[9]             ///< Location of data in RX buffer.
#define CMD_DATA_OH     9

/// Application specific status/error codes.
typedef enum{
    SOCKET_CREATE_ERROR = -0x7D0,               ///< Socket could not be created.
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,       ///< Socket count not be bound to address.
    RECV_ERROR = BIND_ERROR -1                  ///< Reception failed.
} eErrorCodes;

/// Allowed commands.
typedef enum {
    CMD_INIT = 1,                               ///< Initiate command.
    CMD_MOD                                     ///< Modulate command.
} eCommands;

/// Allowed devices.
typedef enum {
    DEV_MCU = 1,                                ///< Device MCU.
    DEV_FPGA                                    ///< Device FPGA.
} eDevices;

static uint8_t  u8Buffer[BUFF_LEN];             ///< RX buffer.

#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);     ///< Vector table.
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;             ///< Vector table.
#endif

static uint8_t  u8CmdFlag;                      ///< Command type flag.
static uint8_t  u8CmdScheme;                    ///< Modulation scheme.
static uint8_t  u8CmdDevice;                    ///< Selected device.
static uint32_t u32CmdBFSKF1;                   ///< BFSK freq1 value.
static uint32_t u32CmdBFSKF2;                   ///< BFSK freq2 value.
static uint8_t  u8CmdDutyCycle;                 ///< BFSK duty cycle value.
static uint8_t  u8CmdPPMBS;                     ///< PPM bit/symbol value.
static uint32_t u32CmdBitRate;                  ///< Modulation bitrate.
static uint8_t  *u8CmdData;                     ///< Pointer to data.
static uint32_t u32CmdDataLen;                  ///< Data length.

/**********************************************************************************************//**
 * Broad initialization and configuration.
 *
 * \param       None
 *
 * \return      None
 *
 *************************************************************************************************/
static void
BoardInit(void) {
#ifndef USE_TIRTOS
    // set vector table base
#if defined(ccs) || defined(gcc)
    MAP_IntVTableBaseSet((uint32_t)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((uint32_t)&__vector_table);
#endif
#endif
    // enable processor
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/**********************************************************************************************//**
 * \brief Modulation done callback funciton.
 *
 * The function sets the command flag to 0.
 *
 * \param[in]   vArgs          - Argument vector (not used).
 *
 * \return      None
 *
 *************************************************************************************************/
static void
ModDone(void *vArgs) {
    // reset command flag
    u8CmdFlag = 0;
}

/**********************************************************************************************//**
 * Combine an array of bytes into an integer.
 *
 * \param[in]   u8Bbuffer      - Pointer to the array of bytes which to combine.

 * \param[in]   u8Len          - The number of bytes to combine.
 * 
 *
 * \return      The integer result of combining the bytes
 *
 *************************************************************************************************/
static uint64_t
Bytes2Int(uint8_t *u8Bbuffer, uint8_t u8Len) {
    uint8_t i = 0;
    uint64_t u64Res = 0;
    
    for (; i < u8Len; i++) {
        u64Res |= (uint64_t)u8Bbuffer[i] << (u8Len - i - 1) * 8;
    }
    
    return u64Res;
}

/**********************************************************************************************//**
 * \brief Create a UDP socket.
 *
 * This function open a socket with the specified port and returns a socket ID.
 *
 * \param[in]   u16Port        - The port on which to bind the socket.
 *
 * \return      Socket ID on success. SOCKET_CREATE_ERROR if socket could not be opened.
 *              BIND_ERROR if socket could not be bound.
 *
 *************************************************************************************************/
static int32_t
CreateUDPSocket(uint16_t u16Port) {
    int32_t s32SocketID = -1;
    int32_t s32RetVal = -1;
    SlSockAddrIn_t sAddr = {0};

    // create UDP socket
    s32SocketID = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);

    if (s32SocketID < 0) {
		sl_Close(s32SocketID);
        return SOCKET_CREATE_ERROR;
    }

    // set addresss parameters
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons(u16Port);
    sAddr.sin_addr.s_addr = INADDR_ANY;

    // bind socket to address
    s32RetVal = sl_Bind(s32SocketID, (SlSockAddr_t *)&sAddr, sizeof(SlSockAddrIn_t));
									
	if (s32RetVal < 0) {
        sl_Close(s32SocketID);
        return BIND_ERROR;
    }

    // return socket ID
    return s32SocketID;
}

/**********************************************************************************************//**
 * \brief Await and parse command packet.
 *
 * This function uses the provided UDP socket to wait indefinitely for a command packet to be
 * received. Once the command is received, it is parsed and static variables are set.
 *
 * \param[in]   port           - The port on which to bind the socket.
 * \param[in]   s32SocketID    - The socket ID to use to receive the command packet.
 *
 * \return      0 on success. RECV_ERROR on reception error.    
 *
 *************************************************************************************************/
static int32_t
GetConnectionlessCmd(int32_t s32SocketID) {
    int32_t s32RetVal = -1;
    uint32_t i;
    uint32_t u32RecvBytes = 0;
    SlSockAddrIn_t sUDPClientAddr = {0};
    SlSockAddrIn_t sUDPTmpAddr = {0};
	SlSocklen_t usUDPClientAddrSize = sizeof(SlSockAddrIn_t);

    u8CmdFlag = 0;

    while(!u8CmdFlag) {
        // receive UDP socket
        s32RetVal = sl_RecvFrom(s32SocketID, u8Buffer, BUFF_LEN, 0,
                                (SlSockAddr_t *)&sUDPClientAddr,  &usUDPClientAddrSize);
        
        if (s32RetVal < 0) {
            sl_Close(s32SocketID);
            return RECV_ERROR;
        }

        // check command indicator
        if ((uint8_t)(CMD_IND & CMD_IND_MASK) == CMD_IND_VAL) {
            // get command type
            switch(CMD_IND & ~CMD_IND_MASK) {
                case CMD_INIT_VAL:
                    u8CmdFlag = CMD_INIT;
                    break;
                case CMD_MOD_VAL:
                    u8CmdFlag = CMD_MOD;
                    break;
            }
        }
    }

    u32RecvBytes += s32RetVal;

    switch(u8CmdFlag) {
        case CMD_INIT:
            // set initialization parameters
            u8CmdScheme = CMD_SCHEME;
            u8CmdDevice = CMD_DEVICE;
            u32CmdBFSKF1 = (uint32_t)Bytes2Int(&CMD_BFSKF1, sizeof(uint32_t));
            u32CmdBFSKF2 = (uint32_t)Bytes2Int(&CMD_BFSKF2, sizeof(uint32_t));
            u8CmdDutyCycle = CMD_DUTY;
            u8CmdPPMBS = CMD_PPMBS;
            break;
        case CMD_MOD:

            // set modulation paramters
            u32CmdBitRate = (uint32_t)Bytes2Int(&CMD_BITRATE, sizeof(uint32_t));
            u32CmdDataLen = (uint32_t)Bytes2Int(&CMD_DLEN, sizeof(uint32_t));

            // allocate memory for new data
            if (u8CmdData != NULL) {
                free(u8CmdData);
            }
            u8CmdData = malloc(u32CmdDataLen);

            // copy data
            memcpy(u8CmdData, &CMD_DATA, u32RecvBytes - CMD_DATA_OH);

            // make sure that all data is received
            while (u32RecvBytes < u32CmdDataLen + CMD_DATA_OH) {
                s32RetVal = sl_RecvFrom(s32SocketID, u8Buffer,
                                        u32CmdDataLen + CMD_DATA_OH - u32RecvBytes, 0,
                                        (SlSockAddr_t *)&sUDPTmpAddr,  &usUDPClientAddrSize);

                if (s32RetVal < 0) {
                    sl_Close(s32SocketID);
                    return RECV_ERROR;
                }

                if (sUDPTmpAddr.sin_addr.s_addr != sUDPClientAddr.sin_addr.s_addr) {
                    continue;
                }

                memcpy(&u8CmdData[u32RecvBytes - CMD_DATA_OH], u8Buffer, s32RetVal);

                u32RecvBytes += s32RetVal;
            }
            break;
    }
    return 0;
}

/**********************************************************************************************//**
 * Main application routine.
 *
 * \param       None
 *
 * \return      None
 *
 *************************************************************************************************/
void
main(void) {

    int32_t s32RetVal = -1;
    int32_t  s32SocketID;
    SlSecParams_t sSecParams = {0};

    // configure and initialize CC3200
    BoardInit();

    // configure UART module
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);

    // initialize serial terminal
    InitTerm();

    // clear terminal
    ClearTerm();

    // display banner
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t      CC3200 Modulator Application       \n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");

    // initialize simplelink to station mode
    Network_IF_InitDriver(ROLE_STA);

    // set network security parameters
    sSecParams.Key = SECURITY_KEY;
    sSecParams.KeyLen = strlen(SECURITY_KEY);
    sSecParams.Type = SECURITY_TYPE;

    // connect to wifi network
    s32RetVal = Network_IF_ConnectAP(SSID_NAME, sSecParams);

    if (s32RetVal < 0) {
        UART_PRINT("Could not connect to AP %d\r\n", s32RetVal);
        while(1);
    }

    // create a UDP socket
    s32SocketID = CreateUDPSocket(SOCKET_PORT);

    if (s32SocketID < 0) {
        while(1);
    }

    while (1) {
        // get command parameters
        s32RetVal = GetConnectionlessCmd(s32SocketID);

        if (s32RetVal == 0) {
            if (u8CmdDevice == DEV_MCU) {
                if (u8CmdFlag == CMD_INIT) {
                    // initialize modulation based on command parameters
                    switch(u8CmdScheme) {
                        case MOD_OOK:
                            while (MOD_IF_InitModulation_OOK(TIMERA0_BASE, GPIO_PIN, PKG_PIN) != 0);
                            UART_PRINT("Initialized to OOK \r\n");
                            break;
                        case MOD_BFSK:
                            while (MOD_IF_InitModulation_BFSK(TIMERA0_BASE, PKG_PIN, u32CmdBFSKF1,
                                    u32CmdBFSKF2, u8CmdDutyCycle) != 0);
                            UART_PRINT("Initialized to BFSK, f1 = %d, f2 = %d , dc = %d%\r\n",
                                        u32CmdBFSKF1, u32CmdBFSKF2, u8CmdDutyCycle);
                            break;
                        case MOD_PPM:
                            while (MOD_IF_InitModulation_PPM(TIMERA0_BASE, GPIO_PIN, PKG_PIN,
                                                                u8CmdPPMBS) != 0);
                            UART_PRINT("Initialized to PPM, bit/symbol = %d \r\n", u8CmdPPMBS);
                            break;
                        default:
                            break;
                    }
                } else if (u8CmdFlag == CMD_MOD) {
                    int i;
                    // send received data 
                    MOD_IF_Modulate(u8CmdData, (uint16_t)u32CmdDataLen, u32CmdBitRate,
                                    1, ModDone, NULL);
                    UART_PRINT("%d bytes sent at %d b/s \r\n", u32CmdDataLen, u32CmdBitRate);
                }
            } else if (u8CmdDevice == DEV_FPGA) {
                // @TODO: implement MCU -> FPGA communication
            }
        }
    }
}
