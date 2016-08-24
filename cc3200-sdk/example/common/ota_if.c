/* ota_if.c
 *
 * OTA update interface functions for CC3200 device.
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
#include <stdbool.h>

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
#include "utils.h"
#include "shamd5.h"

// common interface includes
#include "ota_if.h"
#include "uart_if.h"
#include "common.h"


static uint32_t
_Bytes2Int(uint8_t *u8Bbuffer, uint8_t u8Len) {
    uint8_t i = 0;
    uint32_t u32Res = 0;
    
    for (; i < u8Len; i++) {
        u32Res |= (uint32_t)u8Bbuffer[i] << (u8Len - i - 1) * 8;
    }
    
    return u32Res;
}

static int32_t
_CreateINETSocket(uint8_t u8Type, SlSockAddrIn_t *sAddr) {

    int32_t s32SocketID;
    int32_t s32RetVal;

    // create socket
    s32SocketID = sl_Socket(SL_AF_INET, u8Type, 0);

    // check if socket was created successfully
    if (s32SocketID < 0) {
        return OTA_SOCKET_CREATE_FAILED;
    }

    if (sAddr != NULL) {
        // bind socket to address
        s32RetVal = sl_Bind(s32SocketID, (SlSockAddr_t *)sAddr, sizeof(SlSockAddr_t));
        if (s32RetVal < 0) {
            sl_Close(s32SocketID);
            return OTA_SOCKET_BIND_FAILED;
        }
    }
    return s32SocketID;
}

/*************************************************************************************************/

int32_t
OTA_IF_AwaitUpdateRequest(uint16_t u16Port, SlSockAddrIn_t *sClientAddr) {
	
	int32_t  s32RetVal = -1;
	int32_t  s32SocketID  = -1;
    uint8_t  u8Buffer[OTA_UPDATE_INIT_MSG_LEN + sizeof(uint16_t)] = {0};
	uint16_t u16ClientAddrSize = sizeof(SlSockAddrIn_t);
    SlSockAddrIn_t sAddr;

    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons(u16Port);
    sAddr.sin_addr.s_addr = INADDR_ANY;
	
	// create udp socket
    s32SocketID = _CreateINETSocket(SL_SOCK_DGRAM, &sAddr);

    // check if socket was created successfully
    if (s32SocketID < 0) {
        return s32SocketID;
    }
    
    // wait for update request
    while (strncmp(u8Buffer, OTA_UPDATE_INIT_MSG, OTA_UPDATE_INIT_MSG_LEN)) {
        // receive update command and server port
		s32RetVal = sl_RecvFrom(s32SocketID, u8Buffer, OTA_UPDATE_INIT_MSG_LEN + sizeof(uint16_t),
                                0, (SlSockAddr_t *)sClientAddr, &u16ClientAddrSize);
        
		if (s32RetVal <= 0) {
			sl_Close(s32SocketID);
			return OTA_RECV_ERROR;
		}
	}

    // save server port
    sClientAddr->sin_port = sl_Htons((uint16_t)_Bytes2Int(&u8Buffer[OTA_UPDATE_INIT_MSG_LEN],
                                                    sizeof(uint16_t)));
	
	// update message is received   
	// close socket and return
	s32RetVal = sl_Close(s32SocketID);
	if (s32RetVal < 0) {
        return OTA_SOCKET_CLOSE_FAILED;
    }
	
	return 0;
}

int32_t
OTA_IF_ConnectToServer(SlSockAddrIn_t *sSrvAddr) {
	
	int32_t s32SocketID = -1;
    
	// create TCP socket
	s32SocketID = _CreateINETSocket(SL_SOCK_STREAM, NULL);
	if(s32SocketID < 0) {
        return s32SocketID;
    }
	
	// connect to TCP server
    if(sl_Connect(s32SocketID, (SlSockAddr_t*)sSrvAddr, sizeof(SlSockAddr_t)) < 0) {
		sl_Close(s32SocketID);
        return OTA_CONNECT_ERROR;
    }
    
    return s32SocketID;
}

int32_t
OTA_IF_CloseConnection(int32_t s32SocketID, int32_t s32Status) {

    uint8_t u8Msg[sizeof(int32_t)];
    int32_t i;

    // pack the status code
    for (i = 0; i < sizeof(int32_t); i++) {
        u8Msg[i] = s32Status >> (sizeof(int32_t) - i - 1) * 8;
    }

    // send the code
    if (sl_Send(s32SocketID, u8Msg, sizeof(int32_t), 0) <= 0) {
        return OTA_SEND_ERROR;
    }
    
	MAP_UtilsDelay(8000000);

    // close the socket
    if (sl_Close(s32SocketID) < 0) {
        return OTA_SOCKET_CLOSE_FAILED;
    }

    return 0;
}

int32_t
OTA_IF_SendMACAddress(int32_t s32SocketID) {
    uint8_t u8MACAddressVal[SL_MAC_ADDR_LEN];
    uint8_t u8MACAddressLen = SL_MAC_ADDR_LEN;

    // get MAC address
    if (sl_NetCfgGet(SL_MAC_ADDRESS_GET, NULL,
            &u8MACAddressLen, (uint8_t *)u8MACAddressVal) < 0) {
        return OTA_GET_MAC_ERROR;
    }

    // send MAC address
    if (sl_Send(s32SocketID, u8MACAddressVal, u8MACAddressLen, 0) < 0) {
        return OTA_SEND_ERROR;
    }

    return 0;
}

int32_t
OTA_IF_ReceiveFileLen(int32_t s32SocketID) {
	int16_t s8RecvLen = -1;
	uint8_t u8BytesReceived = 0;
	uint8_t u8Buffer[sizeof(int32_t)];
	int32_t i = 0;
	
	while (u8BytesReceived < sizeof(int32_t)) {
		// receive file length
		s8RecvLen = sl_Recv(s32SocketID, u8Buffer, sizeof(int32_t) - u8BytesReceived, 0);
		if(s8RecvLen <= 0) {
			return OTA_RECV_ERROR;
		}
		u8BytesReceived += s8RecvLen;
	}

    // unpack the value and return it
	return _Bytes2Int(u8Buffer, sizeof(int32_t));
}

int32_t
OTA_IF_DownloadFile(int32_t s32SocketID, uint8_t *u8FileName, uint32_t *u32Token,
                    uint32_t u32FileLen) {
	
	int32_t  s32RetVal = -1;
	int32_t  s32RecvLen = -1;
	int32_t  s32FileHandle;
	uint32_t u32ReceivedBytes = 0;
	uint8_t  u8Buffer[DOWNLOAD_BUF_LEN];
	SlFsFileInfo_t sFileInfo;
	
	// delete the file if it already exists
	s32RetVal = sl_FsGetInfo(u8FileName, *u32Token, &sFileInfo);
	if (s32RetVal == 0) {
		s32RetVal = sl_FsDel(u8FileName, *u32Token);
		if (s32RetVal < 0) {
			return OTA_FILE_DELETE_ERROR;
		}
	}
	
	// create file
	s32RetVal = sl_FsOpen(u8FileName, FS_MODE_OPEN_CREATE(u32FileLen, _FS_FILE_PUBLIC_WRITE),
                            u32Token,
                            &s32FileHandle);
	if(s32RetVal < 0) {
		sl_FsClose(s32FileHandle, 0, 0, 0);
		return OTA_FILE_CREATE_ERROR;
	}
	
	// receive file data
	while (u32ReceivedBytes < u32FileLen) {
		
		s32RecvLen = sl_Recv(s32SocketID, u8Buffer, 
                             (u32FileLen - u32ReceivedBytes < DOWNLOAD_BUF_LEN ? 
                             u32FileLen - u32ReceivedBytes : DOWNLOAD_BUF_LEN), 0);
		
		if(s32RecvLen <= 0) {
		  sl_FsClose(s32FileHandle, 0, 0, 0);
		  return OTA_RECV_ERROR;
		}
		
		// write received data to file
		s32RetVal = sl_FsWrite(s32FileHandle, u32ReceivedBytes, 
							 u8Buffer, s32RecvLen);
							 
		if (s32RetVal < 0) {
			sl_FsClose(s32FileHandle, 0, 0, 0);
			return OTA_FILE_WRITE_ERROR;
		}
		u32ReceivedBytes += s32RecvLen;
	}

    // close file
	s32RetVal = sl_FsClose(s32FileHandle, 0, 0, 0);
	if (s32RetVal < 0) {
		return OTA_FILE_CLOSE_FAILED;
	}

	return u32ReceivedBytes;
}

int32_t
OTA_IF_ReceiveSignature(int32_t s32SocketID, uint8_t *u8Signature, uint16_t u16SignatureLen) {
	
	uint16_t u16ReceivedBytes = 0;
    int32_t  s32RecvLen = -1;
    int32_t  i;

    // receive the 1st 4 bytes of the signature
    s32RecvLen = sl_Recv(s32SocketID, u8Signature, sizeof(int32_t), 0);
                
    if(s32RecvLen <= 0) {
      return OTA_RECV_ERROR;
    }
    
    u16ReceivedBytes += s32RecvLen;

    // if the the first 4 bytes are equal to 0 then no signature is used
    if (_Bytes2Int(u8Signature, sizeof(int32_t)) == 0) {
        return 0;
    }

    // receive signature data
    while(u16ReceivedBytes < u16SignatureLen) {
        s32RecvLen = sl_Recv(s32SocketID, &u8Signature[u16ReceivedBytes], 
                             u16SignatureLen - u16ReceivedBytes, 0);
        if(s32RecvLen <= 0) {
          return OTA_RECV_ERROR;
        }
        u16ReceivedBytes += s32RecvLen;
    }
	
	return u16ReceivedBytes;
}

int32_t
OTA_IF_VerifySignature(uint8_t *u8Signature, uint16_t u16SignatureAlgo, uint8_t *u8FileName,
                               uint32_t *u32Token, int8_t *s8Result) {
	
	const uint16_t u16BufferLen = 64;
	int32_t s32RetVal = -1;
	int32_t s32FileHandle;
	SlFsFileInfo_t sFileInfo;
	uint8_t u8Buffer[u16BufferLen];
	uint32_t u32ReadBytes = 0;
	uint32_t u32Algo;
	uint16_t u16SigLen;
	
	// init signature params
	switch (u16SignatureAlgo) {
		case SIG_MD5:
			u32Algo = SHAMD5_ALGO_MD5;
			u16SigLen = SIG_MD5_LEN;
			break;
		case SIG_SHA1:
			u32Algo = SHAMD5_ALGO_SHA1;
			u16SigLen = SIG_SHA1_LEN;
			break;
		case SIG_SHA224:
			u32Algo = SHAMD5_ALGO_SHA224;
			u16SigLen = SIG_SHA224_LEN;
			break;
		case SIG_SHA256:
			u32Algo = SHAMD5_ALGO_SHA256;
			u16SigLen = SIG_SHA256_LEN;
			break;
		default:
			return OTA_INVALID_PARAM;
	}
	
	// get file info
	s32RetVal = sl_FsGetInfo(u8FileName, *u32Token, &sFileInfo);
	if (s32RetVal < 0) {
		return OTA_FILE_INFO_GET_ERROR;
	}
	
	// open file for reading
	s32RetVal = sl_FsOpen((uint8_t *)u8FileName, FS_MODE_OPEN_READ, u32Token, &s32FileHandle);   
    if (s32RetVal < 0) {
		// file doesn't exist
		sl_FsClose(s32FileHandle, 0, 0, 0);
		return OTA_FILE_OPEN_READ_FAILED;
	}
	
	// enable the SHA/MD5 module
	MAP_PRCMPeripheralClkEnable(PRCM_DTHE, PRCM_RUN_MODE_CLK);
	
	// enable the SHA/MD5 interrupt
	MAP_SHAMD5IntEnable(SHAMD5_BASE, SHAMD5_INT_CONTEXT_READY | SHAMD5_INT_OUTPUT_READY);
	
	// wait for context ready flag
	while(!(MAP_SHAMD5IntStatus(SHAMD5_BASE, 0) & SHAMD5_INT_CONTEXT_READY));
	
	// set the algorithm
	MAP_SHAMD5ConfigSet(SHAMD5_BASE, u32Algo);
	
	// set the data length
	SHAMD5DataLengthSet(SHAMD5_BASE, sFileInfo.FileLen);
	
	// read from file and feed to SHA/MD5 generator
	while (u32ReadBytes < sFileInfo.FileLen) {
        s32RetVal = sl_FsRead(s32FileHandle, u32ReadBytes, u8Buffer, u16BufferLen);
		
		if (s32RetVal < 0) {
            sl_FsClose(s32FileHandle, 0, 0, 0);
			return OTA_FILE_READ_ERROR;
		}			
		
		if (s32RetVal != u16BufferLen) {
			memset(&u8Buffer[s32RetVal], 0, u16BufferLen - s32RetVal);
		}
		
		SHAMD5DataWrite(SHAMD5_BASE, u8Buffer);
		
		u32ReadBytes += s32RetVal;
	}
	if (u32ReadBytes != sFileInfo.FileLen) {
		// error reading file
		sl_FsClose(s32FileHandle, 0, 0, 0);
		return OTA_FILE_READ_ERROR;
	}
	
	// close the file
	s32RetVal = sl_FsClose(s32FileHandle, 0, 0, 0);
	if (s32RetVal < 0) {
		return OTA_FILE_CLOSE_FAILED;
	}
	
	// wait for output ready flag
	while(!(MAP_SHAMD5IntStatus(SHAMD5_BASE, 0) & SHAMD5_INT_OUTPUT_READY));
	
	// read the result
	SHAMD5ResultRead(SHAMD5_BASE, u8Buffer);
	
	// compare the result with the provided value
	*s8Result = memcmp(u8Signature, u8Buffer, u16SigLen);
	
	// disable interrupts and the SHA/MD5 module
	SHAMD5IntDisable(SHAMD5_BASE, SHAMD5_INT_CONTEXT_READY |
									SHAMD5_INT_OUTPUT_READY);
	PRCMPeripheralClkDisable(PRCM_DTHE, PRCM_RUN_MODE_CLK);
	
	return 0;
}
