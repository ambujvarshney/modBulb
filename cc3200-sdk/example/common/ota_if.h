#ifndef OTA_IF_
#define OTA_IF_

#define OTA_UPDATE_INIT_MSG			"update"    ///< The update preamble.
#define OTA_UPDATE_INIT_MSG_LEN		6           ///< The update preamble length.
#define DOWNLOAD_BUF_LEN			(4 * 1024)  ///< The file download buffer length.

#define SIG_MD5_LEN					16          ///< The length of an MD5 signature.
#define SIG_SHA1_LEN				20          ///< The length of an SHA1 signature.
#define SIG_SHA224_LEN				28          ///< The length of an SHA224 signature.
#define SIG_SHA256_LEN				32          ///< The length of an SHA256 signature.

/// Supported signature algorithms
typedef enum {
	SIG_MD5,
	SIG_SHA1,
	SIG_SHA224,
	SIG_SHA256
} eSHAMD5Algo;

/// Error codes
typedef enum {
    OTA_FILE_OPEN_READ_FAILED = -0x9C4,                         ///< Failed to open file for reading.
    OTA_FILE_OPEN_WRITE_FAILED = OTA_FILE_OPEN_READ_FAILED - 1, ///< Failed to open file for writing.
    OTA_FILE_CLOSE_FAILED = OTA_FILE_OPEN_WRITE_FAILED - 1,     ///< Failed to close file.
    OTA_FILE_CREATE_ERROR = OTA_FILE_CLOSE_FAILED - 1,          ///< Failed to create file.
    OTA_FILE_DELETE_ERROR = OTA_FILE_CREATE_ERROR - 1,          ///< Failed to delete file.
    OTA_FILE_READ_ERROR = OTA_FILE_DELETE_ERROR - 1,            ///< Failed to read from file.
    OTA_FILE_WRITE_ERROR = OTA_FILE_READ_ERROR - 1,             ///< Failed to write to file.
    OTA_FILE_INFO_GET_ERROR = OTA_FILE_WRITE_ERROR - 1,         ///< Failed to get file information.
    
    OTA_SOCKET_CREATE_FAILED = OTA_FILE_INFO_GET_ERROR - 1,     ///< Failed to create socket.
    OTA_SOCKET_BIND_FAILED = OTA_SOCKET_CREATE_FAILED - 1,      ///< Failed to bind socket.
    OTA_SOCKET_CLOSE_FAILED = OTA_SOCKET_BIND_FAILED - 1,       ///< Failed to close socket.
    OTA_RECV_ERROR = OTA_SOCKET_CLOSE_FAILED - 1,               ///< Failed to read data from socket.
    OTA_SEND_ERROR = OTA_RECV_ERROR - 1,                        ///< Failed to write data to socket.
    OTA_CONNECT_ERROR = OTA_SEND_ERROR - 1,                     ///< Failed to connect socket.

    OTA_GET_MAC_ERROR = OTA_CONNECT_ERROR - 1,                  ///< Failed to get MAC of device.
    
    OTA_INVALID_PARAM = OTA_GET_MAC_ERROR - 1                   ///< Invalid input parameter(s).
    
} eOTAErrorCodes;

/**********************************************************************************************//**
 * \brief Wait for an update request.
 *
 * This function creates a UDP socket and listens for an update request sent to the specified prot.
 *
 * \param[in]   u16Port          - The port to listen to.
 * \param[out]  sClientAddr      - The address of the server.
 *
 * \return      0 on success. Error code otherwise (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_AwaitUpdateRequest(uint16_t u16Port, SlSockAddrIn_t *sClientAddr);


/**********************************************************************************************//**
 * \brief Connect to OTA TCP server.
 *
 * This function creates a TCP socket and connects to the provided server address.
 *
 * \param[in]   sSrvAddr         - The server address to connect to.
 *
 * \return      Socket ID on success. Error code otherwise (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_ConnectToServer(SlSockAddrIn_t *sSrvAddr);

/**********************************************************************************************//**
 * \brief Disconnect from OTA TCP server.
 *
 * This function send the provided status code to the OTA server and closes the connection.
 *
 * \param[in]   s32SocketID      - The socket ID to close
 * \param[in]   s32Status        - The status message to send before closingt he socket.
 *
 * \return      0 on success. Error code otherwise (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_CloseConnection(int32_t s32SocketID, int32_t s32Status);

/**********************************************************************************************//**
 * \brief Send device MAC address to OTA server.
 *
 * This function gets the device MAC address and sends it to the sever through the provided socket.
 *
 * \param[in]   s32SocketID      - The socket to send the address to.
 *
 * \return      0 on success. Error code otherwise (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_SendMACAddress(int32_t s32SocketID);

/**********************************************************************************************//**
 * \brief Receive file length.
 *
 * \param[in]   s32SocketID      - The socket to read file length from.
 *
 * \return      A positive integer indicating file length on success. Error code otherwise
 *              (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_ReceiveFileLen(int32_t s32SocketID);

/**********************************************************************************************//**
 * \brief Receive file data into a file.
 *
 * \param[in]   s32SocketID      - The socket to read the file data from.
 * \param[in]   u8FileName       - The file name which to save data to.
 * \param[in]   u32Token         - The access token of the file.
 * \param[in]   u32FileLen       - The size of the file to receive.
 *
 * \return      A positive integer indicating the number of received bytes on success. Error code
 *              otherwise (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_DownloadFile(int32_t s32SocketID, uint8_t *u8FileName, uint32_t *u32Token,
                    uint32_t u32FileLen);
                    
/**********************************************************************************************//**
 * \brief Receive an SHA/MD5 signature.
 *
 * This function reads 4 bytes of the signature. If these bytes are equal to zero, no signature is
 * used and the function returns. Otherwise iti receives the remaining bytes of the signature.
 *
 * \param[in]   s32SocketID      - The socket to read the signature data from.
 * \param[out]  u8Signature      - Pointer to the location where to store the signature data.
 * \param[in]   u16SignatureLen  - The length of the signature to receive.
 *
 * \return      0 indicating no signature or a positive integer indicating the number of received
 *              bytes on success. Error code otherwise (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_ReceiveSignature(int32_t s32SocketID, uint8_t *u8Signature, uint16_t u16SignatureLen);

/**********************************************************************************************//**
 * \brief Verify SHA/MD5 signature
 *
 * This function computes the requested signature of a file and comapres it to the provided value.
 *
 * \param[in]   u8Signature      - The signature value to compare the computed value to.
 * \param[out]  u16SignatureAlgo - The signing algorithm to use.
 * \param[in]   u8FileName       - The name of the file which signature is requested.
 * \param[in]   u32Token         - The access token of the file.
 * \param[out]  s8Result         - The result of comparing the provided and the computed signatures.
 *                                 0 when signatures are match.
 *
 * \return      0 on success. Error code otherwise (#eOTAErrorCodes).
 *
 *************************************************************************************************/
int32_t
OTA_IF_VerifySignature(uint8_t *u8Signature, uint16_t u16SignatureAlgo, uint8_t *u8FileName,
                               uint32_t *u32Token, int8_t *s8Result);
#endif
