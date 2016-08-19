/* mod_if.h
 *
 * Modulation interface macros and function prototypes for CC3200 device.
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

#ifndef MOD_IF_
#define MOD_IF_

/// \addtogroup modBulb_Modulation @{

#ifdef __cplusplus
extern "C"
{
#endif



/// compiler switch indecating weather to copy data passed to MOD_IF_Modulate function
#define MOD_STORE_DATA

/// Error codes that the interface functions may return.
typedef enum {
    MOD_INPUT_ERROR = -0xBB8,               ///< One or more inputs are invalid.
    MOD_INIT_ERROR = MOD_INPUT_ERROR - 1,   ///< The modulation is not initialized correctly.
    MOD_TRY_AGAIN = MOD_INIT_ERROR - 1      ///< Modulator is current busy. Try again.

} e_MODErrorCodes;

/// Implemented modulation schemes
typedef enum {
    MOD_OOK = 1,
    MOD_BFSK,
    MOD_PPM
} e_ModulationSchemes;

/// Callback funtion type.
typedef void (*MOD_fnCallback_t)(void *);

uint8_t MOD_IF_u8ModReady;          ///< Read only modulation ready flag.
uint8_t MOD_IF_u8ModScheme;         ///< Read only initialized modulation scheme.

/**********************************************************************************************//**
 * \brief This function initializes OOK.
 *
 * This function initializes the the provided datarate timer module, GPIO pin and state variables
 * to generate OOK.
 *
 * \param[in]   u32DrTimerBase -  The timer base used to generate datarate interrupts.
 * \param[in]   u32SoGPIONum   -  The GPIO pin number used to output the modulated signal.
 * \param[in]   u32SoPinNum    -  The package pin associated with \p u32SoGPIONum.
 *
 * \return      On success 0 will be returned. on error a negative (int32) value will be returned
 *              specifying the error code. Error codes are specified in #e_MODErrorCodes.
 *
 * \note        The provided datarate timer will be used in 32-bit mode to support both high and
 *              low datarates.
 *
 * \warning     A successful return does not guarantee a successful initialization if the package
 *              pin and GPIO pin are not associated with each other.
 * 
 *************************************************************************************************/
int32_t
MOD_IF_InitModulation_OOK(uint32_t u32DrTimerBase, uint32_t u32SoGPIONum, uint32_t u32SoPinNum);

/**********************************************************************************************//**
 * \brief This function initializes BFSK.
 *
 * This function initializes the the provided datarate timer module, PWM module and state variables
 * to generate BFSK.
 *
 * \param[in]   u32DrTimerBase -  The timer base used to generate datarate interrupts.
 * \param[in]   u32PWMPinNum   -  The package pin associated with the desired PWM output signal.
 * \param[in]   u32Freq1       -  The signal frequency associated with logical 0.
 * \param[in]   u32Freq2       -  The signal frequency associated with logical 1.
 * \param[in]   u8DutyCycle    -  The duty cycle of the generated frequencies (0 <= value <= 100).
 *
 * \return      On success 0 will be returned. on error a negative (int32) value will be returned
 *              specifying the error code. Error codes are specified in #e_MODErrorCodes.
 *
 * \note        The provided datarate timer will be used in 32-bit mode to support both high and
 *              low datarates.
 *
 * \warning     The actual generated frequencies may not be identical to provided frequency due to
 *              clock division constraints.
 * 
 *************************************************************************************************/
int32_t
MOD_IF_InitModulation_BFSK(uint32_t u32DrTimerBase, uint32_t u32PWMPinNum, uint32_t u32Freq1,
                           uint32_t u32Freq2, uint8_t u8DutyCycle);

/**********************************************************************************************//**
 * \brief This function initializes PPM.
 *
 * This function initializes the the provided datarate timer module, GPIO pin and state variables
 * to generate PPM.
 *
 * \param[in]   u32DrTimerBase -  The timer base used to generate datarate interrupts.
 * \param[in]   u32SoGPIONum   -  The GPIO pin number used to output the modulated signal.
 * \param[in]   u32SoPinNum    -  The package pin associated with \p u32SoGPIONum.
 * \param[in]   u8BitSymbol    -  The number of bits to send per PPM symbol (value in {2, 4, 8}).
 *
 * \return      On success 0 will be returned. on error a negative (int32) value will be returned
 *              specifying the error code. Error codes are specified in #e_MODErrorCodes.
 *
 * \note        The provided datarate timer will be used in 32-bit mode to support both high and
 *              low datarates.
 *
 * \warning     A successful return does not guarantee a successful initialization if the package
 *              pin and GPIO pin are not associated with each other.
 * 
 *************************************************************************************************/
int32_t
MOD_IF_InitModulation_PPM(uint32_t u32DrTimerBase, uint32_t u32SoGPIONum, uint32_t u32SoPinNum,
                          uint8_t u8BitSymbol);

/**********************************************************************************************//**
 * \brief This function deinitializes any perviously initialized modulation scheme.
 *
 * This function deinitializes any perviously initialized timers and resets state variables.
 *
 * \param       None
 *
 * \return      On success 0 will be returned. on error a negative (int32) value will be returned
 *              specifying the error code. Error codes are specified in #e_MODErrorCodes.
 *
 * \note        This function will not disable and perviously enabled GPIO pins or ports.
 * 
 *************************************************************************************************/
int32_t
MOD_IF_DeinitModulation(void);

/**********************************************************************************************//**
 * \brief This function modulates a packet of data.
 *
 * This function uses the initialized modulation scheme to modulate a provided packet at the
 * specified datarate. When the modulate is done, \p fnCallback function is called with
 * \p vCallbackArgs argument. This function can be used is both blocking and non-blocking modes.
 * If the \p u8Blocking flag is set, the function will wait for any running modulation before
 * attempting to modulate a new packet.
 *
 * \param[in]   u8Data         -  The pointer to the packet to be modulated
 * \param[in]   u16Len         -  The length of the packet. i.e. number of bytes.
 * \param[in]   u32BitRate     -  The bitrate/datarate at which to modulate the packet data.
 * \param[in]   u8Blocking     -  The blocking flag.
 * \param[in]   fnCallback     -  The callback function to call when the modulation is done.
 * \param[in]   vCallbackArgs  -  The argument to be passed to \p fnCallback when its called.
 *
 * \return      On success 0 will be returned. on error a negative (int32) value will be returned
 *              specifying the error code. Error codes are specified in #e_MODErrorCodes.
 *
 * \note        The callback function is called from the datarate interrupt handler when all bits
 *              are modulated.
 *
 * \warning     The actual generated datarate may not be identical to provided datarate due to
 *              clock division constraints.
 *
 * \warning     The data provided in \p u8Data will not be copied to a local buffer if
 *              MOD_STORE_DATA is undefined. Thus, changing the the content of where \p u8Data
 *              points will cause the output signal to change. Define MOD_STORE_DATA to copy data
 *              to a local buffer instead.
 * 
 *************************************************************************************************/
int32_t
MOD_IF_Modulate(uint8_t *u8Data, uint16_t u16Len, uint32_t u32BitRate, uint8_t u8Blocking,
                MOD_fnCallback_t fnCallback, void *vCallbackArgs);

#ifdef __cplusplus
}
#endif

/// @}

#endif
