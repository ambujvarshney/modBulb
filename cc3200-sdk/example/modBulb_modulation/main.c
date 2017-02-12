/* main.c
 *
 * Example that illustrates the use of the modulation interface functions
 * of CC3200 device.
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
#if defined(__CONCAT) && !defined(__SIMPLELINK_H__)
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
#include "spi.h"

// common interface includes
#include "mod_if.h"
#include "spi_if.h"
#include "timer_if.h"
#include "network_if.h"
#ifndef NOTERM
#include "uart_if.h"
#endif
#include "common.h"


#define SPI_BITRATE         100000              ///< SPI bitrate
#define SPI_CS_FPGA_GPIO    7                   ///< SPI chipselect of FPGA board
#define SPI_CS_FPGA_IDX     0                   ///< Index of FPGA chip in SPI chipselect table
#define SPI_CS_NUM          1                   ///< Number of chipselect pins in SPI

#define FPGA_CLK            20000000ul          ///< Frequency at which the FPGA operates
#define MCU_CLK             80000000ul          ///< Frequency at which the MCU operates

#define LOOP_PER_MS         (MCU_CLK / 1000 / 5)///< Number of 5 cycle loops per millisecond

#define SOCKET_PORT         5001                ///< The port to bind the UDP socket to.

#define SO_GPIO_PIN         10                  ///< Output signal GPIO pin number.
#define SO_PKG_PIN          PIN_01              ///< Output signal package pin number.

#define BUFF_LEN            11000               ///< Length of RX buffer.

#define CMD_IND_MASK        0xF0                ///< Command indicator mask.
#define CMD_IND_VAL         0xE0                ///< Command indicator value.

#define CMD_INIT_VAL        0x07                ///< Initialize command value.
#define CMD_MOD_VAL         0x0A                ///< Modulate command value.

#define CMD_IND             u8Buffer[0]         ///< Location of command indicator.

#define CMD_SCHEME          u8Buffer[1]         ///< Location of modulation scheme.
#define CMD_DEVICE          u8Buffer[2]         ///< Location of device.
#define CMD_BFSKF1          u8Buffer[3]         ///< Location of BFSK freq1.
#define CMD_BFSKF2          u8Buffer[7]         ///< Location of BFSK freq2.
#define CMD_DUTY            u8Buffer[11]        ///< Location of BFSK duty cycle.
#define CMD_PPMBS           u8Buffer[12]        ///< Location of PPM bit/symbol.

#define CMD_PACKET_CNT      u8Buffer[1]         ///< Location of the number of packets to send.
#define CMD_PACKET_DELAY    u8Buffer[3]         ///< Location of the delay between packets.
#define CMD_BITRATE         u8Buffer[5]         ///< Location of modulation datarate.
#define CMD_DLEN            u8Buffer[9]         ///< Location of data length.
#define CMD_DATA            u8Buffer[11]        ///< Location of data.
#define CMD_DATA_OH         11                  ///< Number of bytes that come before the data.

#define FPGA_CMD_BR         u8Buffer[0]         ///< Location of bitrate word in FPGA command
#define FPGA_CMD_CTRL       u8Buffer[2]         ///< Location of control word in FPGA command
#define FPGA_CMD_LEN        u8Buffer[4]         ///< Location of data length word in FPGA command 
#define FPGA_CMD_CNT        u8Buffer[6]         ///< Location of packet count word in FPGA command
#define FPGA_CMD_DLY        u8Buffer[8]         ///< Location of interpacket delay word in FPGA command
#define FPGA_CMD_F1         u8Buffer[10]        ///< Location of BFSK freq1 word in FPGA command
#define FPGA_CMD_F2         u8Buffer[12]        ///< Location of BFSK freq2 word in FPGA command
#define FPGA_CMD_M1         u8Buffer[14]        ///< Location of BFSK match1 word in FPGA command
#define FPGA_CMD_M2         u8Buffer[16]        ///< Location of BFSK match2 word in FPGA command
#define FPGA_CMD_SLT        u8Buffer[10]        ///< Location of PPM slot count word in FPGA command
#define FPGA_CMD_BS         u8Buffer[12]        ///< Location of PPM bit/symbol word in FPGA command
#define FPGA_CMD_DATA       u8Buffer[18]        ///< Location of first data byte in FPGA command
#define FPGA_CMD_DATA_OH    18                  ///< Number of bytes before data in FPGA command
#define FPGA_CMD_LASTW8_IDX 3                   ///< Index of bit in CTRL to set when last byte is ignored



/// Application specific status/error codes.
typedef enum{
    SOCKET_CREATE_ERROR = -0x7D0,               ///< Socket could not be created.
    BIND_ERROR  = SOCKET_CREATE_ERROR - 1,      ///< Socket count not be bound to address.
    RECV_ERROR  = BIND_ERROR - 1,               ///< Reception failed.
    AGAIN_ERROR = RECV_ERROR - 1                ///< Reception timed out.
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
static uint16_t u16CmdPacketCnt;                ///< Number of packets to send.
static uint16_t u16CmdPacketDelay;              ///< Delay betwteen packets.
static uint32_t u32CmdBitRate;                  ///< Modulation bitrate.
static uint8_t  *u8CmdData;                     ///< Pointer to data.
static uint16_t u16CmdDataLen;                  ///< Data length.

static SlSockAddrIn_t sUDPClientAddr = {0};     ///< Structure which holds the client address
static SlSocklen_t usUDPClientAddrSize = sizeof(SlSockAddrIn_t);///< size of the address structure

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
 * Combine an array of bytes into an integer.
 *
 * \param[in]   u8Bbuffer      - Pointer to the array of bytes which to combine.

 * \param[in]   u8Len          - The number of bytes to combine.
 * 
 * \return      The integer result of combining the bytes.
 * 
 * \note        This function uses big endian notation.
 *
 *************************************************************************************************/
static uint32_t
Bytes2Int(uint8_t *u8Buffer, uint8_t u8Len) {
    uint8_t i = 0;
    uint32_t u32Res = 0;
    
    for (; i < u8Len; i++) {
        u32Res |= (uint32_t)u8Buffer[i] << (u8Len - i - 1) * 8;
    }
    
    return u32Res;
}

/**********************************************************************************************//**
 * Break an integer into an array of bytes.
 *
 * \param[in]   u8Bbuffer      - Pointer to the array where to place the integer bytes.
 * 
 * \param[in]   u8Len          - The number of bytes to break.
 *
 * \note        This function uses big endian notation.
 * 
 *************************************************************************************************/
static void
Int2Bytes(uint8_t *u8Buffer, uint32_t u32Val, uint8_t u8Len) {
    uint8_t i = 0;
    
    for (; i < u8Len; i++) {
        u8Buffer[i] = u32Val >> (u8Len - i - 1) * 8;
    }
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
GetConnectionlessCmd(int32_t s32SocketID, uint8_t u8Blocking) {
    int32_t s32RetVal = -1;
    uint32_t i;
    uint32_t u32RecvBytes = 0;
    uint8_t u8NonBlocking;
    u8CmdFlag = 0;
    
    if (!u8Blocking) {
        // set socket to operate in nonblocking mode
        u8NonBlocking = 1;
        sl_SetSockOpt(s32SocketID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &u8NonBlocking, 
                      sizeof(uint8_t));
    }

    while(!u8CmdFlag) {
        // receive UDP socket
        s32RetVal = sl_RecvFrom(s32SocketID, u8Buffer, BUFF_LEN, 0,
                                (SlSockAddr_t *)&sUDPClientAddr,  &usUDPClientAddrSize);
        
        if (s32RetVal == SL_EAGAIN) {
            if (!u8Blocking) {
                // set socket to operate in blocking mode
                u8NonBlocking = 0;
                sl_SetSockOpt(s32SocketID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &u8NonBlocking, 
                              sizeof(uint8_t));
            }
            return SL_EAGAIN;
        }
        
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
    
    
    if (!u8Blocking) {
        // set socket to operate in blocking mode
        u8NonBlocking = 0;
        sl_SetSockOpt(s32SocketID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &u8NonBlocking, 
                      sizeof(uint8_t));
    }

    u32RecvBytes += s32RetVal;

    switch(u8CmdFlag) {
        case CMD_INIT:
            // set initialization parameters
            u8CmdScheme = CMD_SCHEME;
            u8CmdDevice = CMD_DEVICE;
            u32CmdBFSKF1 = Bytes2Int(&CMD_BFSKF1, sizeof(uint32_t));
            u32CmdBFSKF2 = Bytes2Int(&CMD_BFSKF2, sizeof(uint32_t));
            u8CmdDutyCycle = CMD_DUTY;
            u8CmdPPMBS = CMD_PPMBS;
            break;
        case CMD_MOD:

            // set modulation paramters
            u16CmdPacketCnt = (uint16_t)Bytes2Int(&CMD_PACKET_CNT, sizeof(uint16_t));
            u16CmdPacketDelay = (uint16_t)Bytes2Int(&CMD_PACKET_DELAY, sizeof(uint16_t));
            u32CmdBitRate = Bytes2Int(&CMD_BITRATE, sizeof(uint32_t));
            u16CmdDataLen = (uint16_t)Bytes2Int(&CMD_DLEN, sizeof(uint16_t));

            // allocate memory for new data
            u8CmdData = realloc(u8CmdData, u16CmdDataLen);

            // copy data
            memcpy(u8CmdData, &CMD_DATA, u32RecvBytes - CMD_DATA_OH);

            // make sure that all data is received
            while (u32RecvBytes < u16CmdDataLen + CMD_DATA_OH) {
                s32RetVal = sl_RecvFrom(s32SocketID, u8Buffer,
                                        u16CmdDataLen + CMD_DATA_OH - u32RecvBytes, 0,
                                        (SlSockAddr_t *)&sUDPClientAddr,  &usUDPClientAddrSize);

                if (s32RetVal < 0) {
                    sl_Close(s32SocketID);
                    return RECV_ERROR;
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

    int32_t  s32RetVal = -1;
    int32_t  s32SocketID;
    uint32_t i;
    uint8_t  u8GPIOPins[] = {SPI_CS_FPGA_GPIO};
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
    UART_PRINT("\t\t ************************************************\n\r");
    UART_PRINT("\t\t           CC3200 Modulator Application\n\r");
    UART_PRINT("\t\t ************************************************\n\r");
    UART_PRINT("\n\n\n\r");
    
    // enable SPI CS FPGA pin
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    MAP_PinTypeGPIO(PIN_62, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT);
    
    // init SPI
    s32RetVal = SPI_IF_Init(SPI_BITRATE, SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS | 
                            SPI_3PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVELOW | SPI_WL_16), 
                            u8GPIOPins, SPI_CS_NUM);
    
    if (s32RetVal < 0) {
        UART_PRINT("Failed to initialize SPI\n\r");
        while (1);
    }

    // initialize simplelink to station mode
    Network_IF_InitDriver(ROLE_STA);

    // set network security parameters
    sSecParams.Key = SECURITY_KEY;
    sSecParams.KeyLen = strlen(SECURITY_KEY);
    sSecParams.Type = SECURITY_TYPE;

    // connect to WiFi network
    s32RetVal = Network_IF_ConnectAP(SSID_NAME, sSecParams);

    if (s32RetVal < 0) {
        UART_PRINT("Could not connect to AP. Error: %d\r\n", s32RetVal);
        while(1);
    }

    // create a UDP socket
    s32SocketID = CreateUDPSocket(SOCKET_PORT);

    if (s32SocketID < 0) {
        UART_PRINT("Failed to create a socket\n\r");
        while(1);
    }

    while (1) {
        // get command parameters
        if (u8CmdFlag == 0) {
            s32RetVal = GetConnectionlessCmd(s32SocketID, 1);
            if (s32RetVal != 0) {
                UART_PRINT("Command reception failed \n\r");
                while (1);
            }
        } else {
            s32RetVal = 0;
        }
        if (u8CmdDevice == DEV_MCU) {
            if (u8CmdFlag == CMD_INIT) {
                // initialize modulation based on command parameters
                switch(u8CmdScheme) {
                    case MOD_OOK:
                        while (MOD_IF_InitModulation_OOK(TIMERA0_BASE, SO_GPIO_PIN, SO_PKG_PIN) 
                               != 0);
                        UART_PRINT("Initialized to OOK \r\n");
                        break;
                    case MOD_BFSK:
                        while (MOD_IF_InitModulation_BFSK(TIMERA0_BASE, SO_PKG_PIN, 
                               u32CmdBFSKF1, u32CmdBFSKF2, u8CmdDutyCycle) != 0);
                        UART_PRINT("Initialized to BFSK, Freq1 = %d, Freq2 = %d , DutyCycle"
                                   " = %d%\r\n", u32CmdBFSKF1, u32CmdBFSKF2, u8CmdDutyCycle);
                        break;
                    case MOD_PPM:
                        while (MOD_IF_InitModulation_PPM(TIMERA0_BASE, SO_GPIO_PIN, SO_PKG_PIN,
                               u8CmdPPMBS) != 0);
                        UART_PRINT("Initialized to PPM, Bits/Symbol = %d \r\n", u8CmdPPMBS);
                        break;
                    default:
                        break;
                }
                u8CmdFlag = 0;
            } else if (u8CmdFlag == CMD_MOD) {
                // send received data
                if (u16CmdPacketCnt == 1) {
                    // one packet to send
                    MOD_IF_Modulate(u8CmdData, u16CmdDataLen, u32CmdBitRate, 1, NULL, NULL);
                    
                    UART_PRINT("1 packet of %d bytes sent at %d b/s \r\n", u16CmdDataLen, 
                               u32CmdBitRate);
                    
                    u8CmdFlag = 0;

                } else {
                    i = 1;
                    u8CmdFlag = 0;
                    MOD_IF_Modulate(u8CmdData, u16CmdDataLen, u32CmdBitRate, 1, NULL, NULL);
                    while (!u16CmdPacketCnt || i < u16CmdPacketCnt) {
                        while (!MOD_IF_u8ModReady);
                        if (u16CmdPacketDelay > 0) {
                            MAP_UtilsDelay(LOOP_PER_MS * (uint32_t)(u16CmdPacketDelay * 0.835 + 
                                           0.5));
                        }
                        
                        if (GetConnectionlessCmd(s32SocketID, 0) == 0) {
                            break;
                        }
                        
                        MOD_IF_Modulate(u8CmdData, u16CmdDataLen, u32CmdBitRate, 1, NULL, NULL);
                        i++;
                    }
                    UART_PRINT("%d packets of %d bytes sent at %d b/s \r\n", i, 
                               u16CmdDataLen, u32CmdBitRate);
                }
            }
        } else if (u8CmdDevice == DEV_FPGA) {
            if (u8CmdFlag == CMD_INIT) {
                UART_PRINT("Initialized to %s \r\n", u8CmdScheme == MOD_OOK ? "OOK" :
                                                     u8CmdScheme == MOD_BFSK? "BFSK":
                                                     u8CmdScheme == MOD_PPM ? "PPM" : "");
                u8CmdFlag = 0;
            } else if (u8CmdFlag == CMD_MOD) {
                
                // set bitrate
                if (u8CmdScheme == MOD_PPM) {
                    Int2Bytes(&FPGA_CMD_BR, (uint16_t)(u8CmdPPMBS * FPGA_CLK /
                                            (float)(u32CmdBitRate * (1ul << u8CmdPPMBS)) - 0.5),
                              sizeof(uint16_t));
                } else {
                    Int2Bytes(&FPGA_CMD_BR, (uint16_t)(FPGA_CLK / (float)u32CmdBitRate - 0.5),
                              sizeof(uint16_t));
                }
                // set control
                Int2Bytes(&FPGA_CMD_CTRL, u8CmdScheme | ((1 << FPGA_CMD_LASTW8_IDX) & 
                                          (u16CmdDataLen % 2) << FPGA_CMD_LASTW8_IDX),
                          sizeof(uint16_t));
                // set length
                Int2Bytes(&FPGA_CMD_LEN, (uint16_t)(u16CmdDataLen / 2.0 + 0.51), 
                          sizeof(uint16_t));
                // set count
                Int2Bytes(&FPGA_CMD_CNT, u16CmdPacketCnt, sizeof(uint16_t));
                // set delay
                Int2Bytes(&FPGA_CMD_DLY, u16CmdPacketDelay, sizeof(uint16_t));
                
                if (u8CmdScheme == MOD_BFSK) {
                    // set freq1
                    Int2Bytes(&FPGA_CMD_F1, (uint16_t)(FPGA_CLK / (float)u32CmdBFSKF1 - 0.5),
                              sizeof(uint16_t));
                    // set freq2
                    Int2Bytes(&FPGA_CMD_F2, (uint16_t)(FPGA_CLK / (float)u32CmdBFSKF2 - 0.5),
                              sizeof(uint16_t));
                    // set match1
                    Int2Bytes(&FPGA_CMD_M1, (uint16_t)((FPGA_CLK / (float)u32CmdBFSKF1 - 0.5) * 
                                            (uint32_t)u8CmdDutyCycle / 100), sizeof(uint16_t));
                    // set match2
                    Int2Bytes(&FPGA_CMD_M2, (uint16_t)((FPGA_CLK / (float)u32CmdBFSKF2 - 0.5) * 
                                            (uint32_t)u8CmdDutyCycle / 100), sizeof(uint16_t));
                } else if (u8CmdScheme == MOD_PPM) {
                    // set slot count
                    Int2Bytes(&FPGA_CMD_SLT, 1ul << u8CmdPPMBS, sizeof(uint16_t));
                    // set bit/symbol
                    Int2Bytes(&FPGA_CMD_BS, u8CmdPPMBS, sizeof(uint16_t));
                }
                // copy data
                memcpy(&FPGA_CMD_DATA, u8CmdData, u16CmdDataLen);
                
                // convert all words to little endian
                for (i = 0; i < FPGA_CMD_DATA_OH + u16CmdDataLen + (u16CmdDataLen % 2); i += 2) {
                    *(uint16_t*)&u8Buffer[i] = sl_Htons(*(uint16_t*)&u8Buffer[i]);
                }
                
                // send the data
                SPI_IF_Send(u8Buffer, u16CmdDataLen + FPGA_CMD_DATA_OH + (u16CmdDataLen % 2),
                            SPI_CS_FPGA_IDX);
                u8CmdFlag = 0;
                UART_PRINT("Command was sent through SPI \r\n");
            }
        }
    }
}
