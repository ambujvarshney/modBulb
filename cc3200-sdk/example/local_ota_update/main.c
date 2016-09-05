/* main.c
 *
 * Example that illustrates the use of the OTA update interface functions
 * and DirectC implementation of CC3200 device.
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
 
// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#if defined(__CONCAT) && !defined(__SIMPLELINK_H__)
#undef __CONCAT
#endif
// simplelink includes 
#include "simplelink.h"
#include "wlan.h"

// driverlib includes 
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"
#include "gpio.h"
#include "timer.h"
#include "hw_timer.h"
#include "hw_gpio.h"
#include "bootmgr.h"
#include "pin.h"

// DirectC includes
#include "dpuser.h"
#include "dpalg.h"

// common interface includes
#include "network_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif
#include "ota_if.h"

// file download definitions
#define FPGA_CONF_FILE          "/fpga/configfile.dat"  ///< FPGA configuration file name.
#define NETWORKINFO_FNAME       "/sys/networkinfo.bin"  ///< Network info file name.
#define OTA_SERVER_ADDR         "/sys/otasrvaddr.bin"   ///< OTA server address file name.

#define OTA_LISTEN_PORT         5001                    ///< Port to listen for update requests on.

#define MCU_IMG_UPDATER         IMG_ACT_USER1           ///< MCU updater image index.
#define MCU_IMG_1               IMG_ACT_USER2           ///< MCU image 1 index.
#define MCU_IMG_2               IMG_ACT_USER3           ///< MCU image 2 index.

#define MCU_IMG_1_FNAME         IMG_USER_2              ///< MCU image 1 file name.
#define MCU_IMG_2_FNAME         IMG_USER_3              ///< MCU image 2 file name.

#define MCU_IMG_1_TOKEN         USER_IMG_2_TOKEN        ///< MCU image 1 access token.
#define MCU_IMG_2_TOKEN         USER_IMG_3_TOKEN        ///< MCU image 2 access token.

// compiler switch
// #define OTA_TEST                                        ///< Build OTA test image.

/// WiFi network information structure
typedef struct sNetworkInfo {
    uint8_t u8SSID[SSID_LEN_MAX + 1];                   ///< Network SSID.
    uint8_t u8SecurityKey[32];                          ///< Network security key.
    uint8_t u8SecurityType;                             ///< Network security type.
} sNetworkInfo_t;


/// OTA update client Error codes.
typedef enum {
    CONNECT_ERROR =  -0x7D0,                            ///< Failed to connect to server
    RECV_ERROR = CONNECT_ERROR - 1,                     ///< Failed to receive valid data.
    FILE_READ_FAILED = RECV_ERROR - 1,                  ///< Failed to read from file.
    FILE_WRITE_FAILED = FILE_READ_FAILED - 1,           ///< Failed to write to file.
} eErrorCodes;

#ifndef OTA_TEST
uint8_t  g_u8MCUUpdateAvailable;                        ///< MCU update available flag.
uint8_t  g_u8FPGAUpdateAvailable;                       ///< FPGA update available flag.
int32_t  g_s32ConfFileHandle = -1;                      ///< FPGA configuration file handle
uint8_t  g_u8NextActiveImage = MCU_IMG_UPDATER;         ///< MCU next active image.
#endif

#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

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
 * \brief Read data from file.
 *
 * This function reads the specified number of bytes from the beginning of the specified file into
 * the specified memory location.
 *
 * \param[in]   u8FileName     - The file name from which to read the data.
 * \param[in]   u32Token       - The file access token.
 * \param[in]   vData          - The pointer to the memory location of where to store the read data.
 * \param[in]   u32Len         - The number of bytes to read.
 *
 * \return      On success 0 is returned. Otherwise -1 is returned.
 *
 *************************************************************************************************/
static int32_t
ReadFromFile(uint8_t *u8FileName, uint32_t *u32Token, void *vData, uint32_t u32Len) {
    int32_t  s32FileHandle;

    if(sl_FsOpen(u8FileName, FS_MODE_OPEN_READ, u32Token, &s32FileHandle) == 0) {
        if(sl_FsRead(s32FileHandle, 0, (uint8_t *)vData, u32Len) > 0) {
            sl_FsClose(s32FileHandle, 0, 0, 0);
            return 0;
        }
        sl_FsClose(s32FileHandle, 0, 0, 0);
    }
    return -1;
}

/**********************************************************************************************//**
 * \brief Write data to file.
 *
 * This function write the specified number of bytes from the specified memory location to the
 * beginning of thte specified file.
 *
 * \param[in]   u8FileName     - The file name to which to write the data.
 * \param[in]   u32Token       - The file access token.
 * \param[in]   vData          - The pointer to the memory location where the data to write is
 *                               stored.
 * \param[in]   u32Len         - The number of bytes to write.
 *
 * \return      On success 0 is returned. Otherwise -1 is returned.
 *
 *************************************************************************************************/
static int32_t
WriteToFile(uint8_t *u8FileName, uint32_t *u32Token, void *vData, uint32_t u32Len) {
    int32_t s32FileHandle;

    if (sl_FsOpen(u8FileName, FS_MODE_OPEN_WRITE, u32Token, &s32FileHandle) != 0) {
        sl_FsClose(s32FileHandle, 0, 0, 0);
        if (sl_FsOpen(u8FileName, FS_MODE_OPEN_CREATE(u32Len, _FS_FILE_PUBLIC_WRITE), u32Token,
                      &s32FileHandle) != 0) {
            return -1;
        }
    }
    
    if (sl_FsWrite(s32FileHandle, 0, (uint8_t *)vData, u32Len) < 0) {
        sl_FsClose(s32FileHandle, 0, 0, 0);
        return -1;
    }
    sl_FsClose(s32FileHandle, 0, 0, 0);
    return 0;
}

#ifndef OTA_TEST

/**********************************************************************************************//**
 * This function commits the downloaded updates.
 *
 * \param[in]   None.
 *
 * \return      On success, 0 is returned. Otherwise, error code is returned. See #eErrorCodes and
 *              dpalg.h
 *
 *************************************************************************************************/
static int32_t
CommitUpdate(void) {
    int32_t s32RetVal = -1;
    uint32_t u32Token;

    if (g_u8MCUUpdateAvailable) {
        sBootInfo_t sBootInfo;
        
        UART_PRINT("Committing MCU image file... \n\r");
        
        // change boot info to boot the updated image
        u32Token = 0;
        sBootInfo.ucActiveImg = g_u8NextActiveImage;
        sBootInfo.ucPrvActImg = MCU_IMG_UPDATER;
        if (WriteToFile(IMG_BOOT_INFO, &u32Token, &sBootInfo, sizeof(sBootInfo_t)) != 0) {
            ASSERT_ON_ERROR(FILE_WRITE_FAILED);
        }

        // delete the old image
        if (g_u8NextActiveImage == MCU_IMG_1) {
            u32Token = MCU_IMG_2_TOKEN;
            sl_FsDel(MCU_IMG_2_FNAME, u32Token);
        } else {
            u32Token = MCU_IMG_1_TOKEN;
            sl_FsDel(MCU_IMG_1_FNAME, u32Token);
        }

        UART_PRINT("MCU image file committed \n\r");
        g_u8MCUUpdateAvailable = 0;
    } else {
        sBootInfo_t sBootInfo;
        if (ReadFromFile(IMG_BOOT_INFO, &u32Token, &sBootInfo, sizeof(sBootInfo_t)) != 0) {
            ASSERT_ON_ERROR(FILE_READ_FAILED);
        }
        // change boot info to boot the old image
        sBootInfo.ucActiveImg = sBootInfo.ucPrvActImg;
        sBootInfo.ucPrvActImg = MCU_IMG_UPDATER;
        if (WriteToFile(IMG_BOOT_INFO, &u32Token, &sBootInfo, sizeof(sBootInfo_t)) != 0) {
            ASSERT_ON_ERROR(FILE_WRITE_FAILED);
        }
    }
    
    if (g_u8FPGAUpdateAvailable) {

        u32Token = 0;

        s32RetVal = sl_FsOpen((uint8_t *)FPGA_CONF_FILE, FS_MODE_OPEN_READ, &u32Token,
                              &config_file_handle);
        if (s32RetVal < 0) {
            ASSERT_ON_ERROR(s32RetVal);
        }
        UART_PRINT("Configuring the FPGA array (takes about 35s)... \n\r");

        Action_code = DP_PROGRAM_ACTION_CODE;

        s32RetVal = dp_top();
        if (s32RetVal != 0) {
            UART_PRINT("FPGA configuration failed \r\n");
            ASSERT_ON_ERROR(s32RetVal);
        }
        UART_PRINT("FPGA configuration was successful \n\r");
        g_u8FPGAUpdateAvailable = 0;
    }

    return 0;
}

/**********************************************************************************************//**
 * This function connects to the OTA server and downloads the updates.
 *
 * \param[in]   sSrvAddr       - The address of the OTA server.
 *
 * \return      On success, 0 is returned. Otherwise, error code is returned. See #eErrorCodes and
 *              #eOTAErrorCodes.
 *
 *************************************************************************************************/
static int32_t
OTAUpdate(SlSockAddrIn_t *sSrvAddr) {
    int32_t  s32RetVal = -1;
    int32_t  s32SocketID = -1;
    int32_t  s32FileLen = -1;
    uint32_t u32Token = 0;
    uint8_t  u8MCUSigMD5[SIG_MD5_LEN];
    uint8_t  u8FPGASigMD5[SIG_MD5_LEN];
    int8_t   s8SigMatch = 0;
    uint32_t i;
    uint8_t  u8DownloadLocation[50];
    sBootInfo_t sBootInfo;

    g_u8MCUUpdateAvailable  = 0;
    g_u8FPGAUpdateAvailable = 0;

    // connect to OTA server
    s32SocketID = OTA_IF_ConnectToServer(sSrvAddr);
    
    if (s32SocketID < 0) {
        ASSERT_ON_ERROR(CONNECT_ERROR);
    }

    UART_PRINT("Connected to server \n\r");

    // send your MAC address to the server
    s32RetVal = OTA_IF_SendMACAddress(s32SocketID);
    if (s32RetVal < 0) {
        ASSERT_ON_ERROR(s32RetVal);
    }

    UART_PRINT("MAC address sent \n\r");

    // receive MCU image file length
    s32FileLen = OTA_IF_ReceiveFileLen(s32SocketID);
    if (s32FileLen < 0) {
        OTA_IF_CloseConnection(s32SocketID, s32FileLen);
        ASSERT_ON_ERROR(s32FileLen);
    }

    if (s32FileLen > 0) {
        // MCU update available
        UART_PRINT("Receiving MCU image file... \n\r");
        UART_PRINT("File length received \n\rFile length: %dB \n\r", (int)s32FileLen);

        // read MCU boot info
        if (ReadFromFile(IMG_BOOT_INFO, &u32Token, &sBootInfo, sizeof(sBootInfo_t)) != 0) {
            ASSERT_ON_ERROR(FILE_READ_FAILED);
        }

        // choose download location
        if (sBootInfo.ucPrvActImg == MCU_IMG_1) {
            g_u8NextActiveImage = MCU_IMG_2;
            strcpy(u8DownloadLocation, MCU_IMG_2_FNAME);
            u32Token = MCU_IMG_2_TOKEN;
        } else {
            g_u8NextActiveImage = MCU_IMG_1;
            strcpy(u8DownloadLocation, MCU_IMG_1_FNAME);
            u32Token = MCU_IMG_1_TOKEN;
        }

        UART_PRINT("Downloading update file to: %s \n\r", u8DownloadLocation);

        // download update file
        s32RetVal = OTA_IF_DownloadFile(s32SocketID, u8DownloadLocation, &u32Token,
                    (uint32_t)s32FileLen);
        if (s32RetVal < 0) {
            OTA_IF_CloseConnection(s32SocketID, s32RetVal);
            ASSERT_ON_ERROR(s32RetVal);
        }
        
        UART_PRINT("File received \n\r");

        // receive file signature
        s32RetVal = OTA_IF_ReceiveSignature(s32SocketID, u8MCUSigMD5, SIG_MD5_LEN);
        
        if (s32RetVal < 0) {
            OTA_IF_CloseConnection(s32SocketID, s32RetVal);
            ASSERT_ON_ERROR(s32RetVal);
        }

        if (s32RetVal > 0) {
            // signature is available
            UART_PRINT("Signature received \n\rMD5 signature: ");
            for (i = 0; i < SIG_MD5_LEN; i++) {
                UART_PRINT("%02x", (unsigned int)u8MCUSigMD5[i]);
            }
            UART_PRINT("\n\r");

            // verify signature
            s32RetVal = OTA_IF_VerifySignature(u8MCUSigMD5, SIG_MD5, u8DownloadLocation, 
                                               &u32Token, &s8SigMatch);
                                            
            if (s32RetVal != 0) {
                OTA_IF_CloseConnection(s32SocketID, s32RetVal);
                ASSERT_ON_ERROR(s32RetVal);
            }
            
            if (s8SigMatch) {
                OTA_IF_CloseConnection(s32SocketID, s8SigMatch);
                UART_PRINT("Singature verification failed\n\r");
                ASSERT_ON_ERROR(RECV_ERROR);
            }
            
            UART_PRINT("Singature verification was successful\n\r");
        }
        
        g_u8MCUUpdateAvailable = 1;
    }

    // receive FPGA configuration file length
    s32FileLen = OTA_IF_ReceiveFileLen(s32SocketID);
    if (s32FileLen < 0) {
        OTA_IF_CloseConnection(s32SocketID, s32FileLen);
        ASSERT_ON_ERROR(s32FileLen);
    }
    if (s32FileLen > 0) {
        // FPGA update available
        UART_PRINT("Receiving FPGA image file... \n\r");
        UART_PRINT("File length received \n\rFile length: %dB \n\r", s32FileLen);

        u32Token = 0;
        
        // download config file
        s32RetVal = OTA_IF_DownloadFile(s32SocketID, FPGA_CONF_FILE, &u32Token,
                                        (uint32_t)s32FileLen);
        if (s32RetVal < 0) {
            OTA_IF_CloseConnection(s32SocketID, s32RetVal);
            ASSERT_ON_ERROR(s32RetVal);
        }
        UART_PRINT("File received \n\r");
        
        // receive signature
        s32RetVal = OTA_IF_ReceiveSignature(s32SocketID, u8FPGASigMD5, SIG_MD5_LEN);
    
        if (s32RetVal < 0) {
            OTA_IF_CloseConnection(s32SocketID, s32RetVal);
            ASSERT_ON_ERROR(s32RetVal);
        }

        if (s32RetVal > 0) {
            // signature available
            UART_PRINT("Signature received \n\rMD5 signature: ");
            for (i = 0; i < SIG_MD5_LEN; i++) {
                UART_PRINT("%02x", (unsigned int)u8FPGASigMD5[i]);
            }
            UART_PRINT("\n\r");
            
            // verify signature
            s32RetVal = OTA_IF_VerifySignature(u8FPGASigMD5, SIG_MD5, FPGA_CONF_FILE, 
                                            &u32Token, &s8SigMatch);
                                            
            if (s32RetVal != 0) {
                OTA_IF_CloseConnection(s32SocketID, s32RetVal);
                ASSERT_ON_ERROR(s32RetVal);
            }
            
            if (s8SigMatch) {
                OTA_IF_CloseConnection(s32SocketID, s8SigMatch);
                UART_PRINT("Singature verification failed\n\r");
                ASSERT_ON_ERROR(RECV_ERROR);
            }
            
            UART_PRINT("Singature verification was successful\n\r");
        }

        g_u8FPGAUpdateAvailable = 1;
    }

    // commit updates
    if (g_u8MCUUpdateAvailable || g_u8FPGAUpdateAvailable) {
        UART_PRINT("Commiting updates... \n\r");
        s32RetVal = CommitUpdate();
        if(s32RetVal != 0) {
            UART_PRINT("Commit failed! \n\r");
            ASSERT_ON_ERROR(s32RetVal);
        }
        UART_PRINT("Updates committed \n\r");
    }

    // close connection to server
    s32RetVal = OTA_IF_CloseConnection(s32SocketID, 0);
    if (s32RetVal != 0) {
        ASSERT_ON_ERROR(s32RetVal);
    }
    
    return 0;
}
#endif

/**********************************************************************************************//**
 * Main application routine.
 *
 * \param       None
 *
 * \return      None
 *
 *************************************************************************************************/
void main() {

    int32_t  s32RetVal = -1;
    uint32_t u32Token;
    sNetworkInfo_t sNetworkInfo;
    SlSockAddrIn_t sAddr;
    SlSecParams_t sSecParams = {0};
#ifdef OTA_TEST
    sBootInfo_t sBootInfo;
#endif

    BoardInit();

    // configure UART
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);

#ifndef OTA_TEST
    // configure JTAG GPIOs.
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);

    MAP_PinTypeGPIO(PIN_58, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT);

    MAP_PinTypeGPIO(PIN_59, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT);

    MAP_PinTypeGPIO(PIN_60, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT);

    MAP_PinTypeGPIO(PIN_62, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT);

    MAP_PinTypeGPIO(PIN_61, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);
#endif

    // initialize terminal
    InitTerm();
    ClearTerm();

    // initialize simplelink to station mode
    Network_IF_InitDriver(ROLE_STA);

#ifndef OTA_TEST
    if (ReadFromFile(NETWORKINFO_FNAME, &u32Token, &sNetworkInfo, sizeof(sNetworkInfo_t)) != 0) {
        UART_PRINT("Could not read network info \r\n");
        while(1);
    }
#else
    // set network info
    strcpy(sNetworkInfo.u8SSID, SSID_NAME);
    strcpy(sNetworkInfo.u8SecurityKey, SECURITY_KEY);
    sNetworkInfo.u8SecurityType = SECURITY_TYPE;
#endif

    // set network security parameters
    sSecParams.Key = sNetworkInfo.u8SecurityKey;
    sSecParams.KeyLen = strlen(sNetworkInfo.u8SecurityKey);
    sSecParams.Type = sNetworkInfo.u8SecurityType;

    // connect to wifi network
    s32RetVal = Network_IF_ConnectAP(sNetworkInfo.u8SSID, sSecParams);

    if (s32RetVal < 0) {
        UART_PRINT("Could not connect to AP %d\r\n", s32RetVal);
        while(1);
    }

#ifndef OTA_TEST
    if (ReadFromFile(OTA_SERVER_ADDR, &u32Token, &sAddr, sizeof(SlSockAddrIn_t)) != 0) {
        UART_PRINT("Could not read server address \r\n");
        while(1);
    }
                       
    if (OTAUpdate(&sAddr) < 0) {
        UART_PRINT("OTA update failed \n\r");
        while(1);
    }
#else
    // write network info to file
    if (WriteToFile(NETWORKINFO_FNAME, &u32Token, &sNetworkInfo, sizeof(sNetworkInfo_t)) != 0) {
        UART_PRINT("Could not write network info \r\n");
        while(1);
    }
    
    if (ReadFromFile(IMG_BOOT_INFO, &u32Token, &sBootInfo, sizeof(sBootInfo_t)) != 0) {
        UART_PRINT("Could not read from boot info \n\r");
        while(1);
    }
    sBootInfo.ucPrvActImg = sBootInfo.ucActiveImg;
    sBootInfo.ucActiveImg = MCU_IMG_UPDATER;
    
    
    UART_PRINT("Awaiting update request... \n\r");
    
    s32RetVal = OTA_IF_AwaitUpdateRequest(OTA_LISTEN_PORT, &sAddr);
    if (s32RetVal < 0) {
        UART_PRINT("Failed to receive update request: %d\n\r", s32RetVal);
        while(1);
    }

    if (WriteToFile(OTA_SERVER_ADDR, &u32Token, &sAddr, sizeof(SlSockAddrIn_t)) != 0) {
        UART_PRINT("Could not write OTA server address to file \n\r");
        while(1);
    }
    
    UART_PRINT("Update request received \n\rBooting updater image... \r\n");
    
    if (WriteToFile(IMG_BOOT_INFO, &u32Token, &sBootInfo, sizeof(sBootInfo_t)) != 0) {
        UART_PRINT("Could not write to boot info \n\r");
        while(1);
    }

#endif

    Network_IF_DeInitDriver();

    UART_PRINT("Exiting Application... \n\r");
    
    MAP_UtilsDelay(8000000);
    
    MAP_PRCMHibernateIntervalSet(330);
    MAP_PRCMHibernateWakeupSourceEnable(PRCM_HIB_SLOW_CLK_CTR);
    MAP_PRCMHibernateEnter();
}
