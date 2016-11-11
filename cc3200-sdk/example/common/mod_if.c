/* mod_if.c
 *
 * Modulation interface functions for CC3200 device.
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

// standard includes
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// driverlib includes
#include "mod_if.h"
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
#include "gpio.h"
#include "pin.h"

// common interface includes
#include "mod_if.h"

// assertion definitions
#define ASSERT_INPUT(val)                               \
                            if (!(val)) {               \
                                return MOD_INPUT_ERROR; \
                            }
#define ASSERT_ERROR(val, error)                        \
                            if (!(val)) {               \
                                return error;           \
                            }

// frequency of the clock fed to the timers
#define MOD_CLK_FREQ        80000000ul


#ifdef MOD_STORE_DATA
// data buffer length
#define MOD_BUFFER_LEN      4096

static uint8_t u8Buffer[MOD_BUFFER_LEN];
#endif

static uint8_t u8ModScheme;
static uint8_t u8ModReady = 0;

static uint8_t u8ModInit = 0;

static MOD_fnCallback_t fnCb;
static void *vCbArgs;

static uint32_t u32DrTimBase;
static uint32_t u32DrTimInterval;

static uint32_t u32PWMTimBase;
static uint32_t u32PWMTim;
static uint32_t u32PWMTimLoadReg;
static uint32_t u32PWMTimSwitchReg;
static uint32_t u32PWMTimRldInt1;
static uint32_t u32PWMTimRldInt2;
static uint32_t u32PWMTimSwtInt1;
static uint32_t u32PWMTimSwtInt2;

static uint32_t u32PPMSlotCntr;
static uint32_t u32PPMSlotCntrMax;
static uint16_t u16PPMSlotCnt;
static uint8_t  u8PPMCurrentSlot;
static uint8_t  u8PPMBitSymbol;
static uint8_t  u8PPMNextValue;
static uint8_t  u8PPMMask;

static uint32_t u32GPIOBase;
static uint8_t  u8GPIOPin;
static uint32_t u32GPIOPinReg;

static uint8_t  *u8DataOut;
static uint32_t u32DataLen;
static uint32_t u32BitCntr;

static void
_GetGPIOInfo(uint32_t u32GPIONum, uint32_t *u32Base, uint8_t *u8Offset, uint32_t *u32Peripheral) {

    const uint32_t cBaseReg[] = {GPIOA0_BASE, GPIOA1_BASE, GPIOA2_BASE, GPIOA3_BASE, GPIOA4_BASE};
    const uint32_t cPeriReg[] = {PRCM_GPIOA0, PRCM_GPIOA1, PRCM_GPIOA2, PRCM_GPIOA3, PRCM_GPIOA4};

    // get GPIO info
    *u32Base = cBaseReg[u32GPIONum / 8];
    *u8Offset = 1 << (u32GPIONum % 8);
    *u32Peripheral = cPeriReg[u32GPIONum / 8];
}

static void
_GetTimerInfo(uint32_t u32Base, uint32_t *u32Peripheral, uint32_t *u32Interrupt) {

    // get timer info
    switch (u32Base) {
        case TIMERA0_BASE:
            *u32Peripheral = PRCM_TIMERA0;
            *u32Interrupt = INT_TIMERA0A;
            break;
        case TIMERA1_BASE:
            *u32Peripheral = PRCM_TIMERA1;
            *u32Interrupt = INT_TIMERA1A;
            break;
        case TIMERA2_BASE:
            *u32Peripheral = PRCM_TIMERA2;
            *u32Interrupt = INT_TIMERA2A;
            break;
        case TIMERA3_BASE:
            *u32Peripheral = PRCM_TIMERA3;
            *u32Interrupt = INT_TIMERA3A;
            break;
    }
}

static void
_EnableDrTimer(void) {
    // set timer reload value
    MAP_TimerLoadSet(u32DrTimBase, TIMER_BOTH, u32DrTimInterval);

    // enable the timer and its interrupt
    MAP_TimerIntEnable(u32DrTimBase, TIMER_BOTH);
    MAP_TimerEnable(u32DrTimBase, TIMER_BOTH);
}

static void
_DisableDrTimer(void) {
    // disable the timer and its interrupt
    MAP_TimerDisable(u32DrTimBase, TIMER_BOTH);
    MAP_TimerIntDisable(u32DrTimBase, TIMER_BOTH);
}

static void
_InitDrTimer(uint32_t u32TimerBase, void (*fnIntHandler)(void)) {

    uint32_t u32Peripheral, u32Interrupt;

    // get timer info
    _GetTimerInfo(u32TimerBase, &u32Peripheral, &u32Interrupt);

    // set static variables
    u32DrTimBase = u32TimerBase;
    
    // enable and reset timer module
    MAP_PRCMPeripheralClkEnable(u32Peripheral, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(u32Peripheral);

    // disable the timer and its interrupts
    MAP_TimerDisable(u32TimerBase, TIMER_BOTH);
    MAP_TimerIntDisable(u32TimerBase, TIMER_BOTH);

    // configure the timer
    MAP_TimerConfigure(u32TimerBase, (TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC));
    MAP_TimerPrescaleSet(u32TimerBase, TIMER_BOTH, 0);
    

    // configure the timer interrupts
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
//if any OS is used
    osi_InterruptRegister(u32Interrupt, fnIntHandler, INT_PRIORITY_LVL_1);
    osi_InterruptRegister(u32Interrupt + 1, fnIntHandler, INT_PRIORITY_LVL_1);
#else
    MAP_IntPrioritySet(u32Interrupt, INT_PRIORITY_LVL_1);
    MAP_IntPrioritySet(u32Interrupt + 1, INT_PRIORITY_LVL_1);
    MAP_TimerIntRegister(u32TimerBase, TIMER_BOTH, fnIntHandler);
#endif
}

static void
_DeinitDrTimer(void) {

    uint32_t u32Peripheral, u32Interrupt;

    // get timer info
    _GetTimerInfo(u32DrTimBase, &u32Peripheral, &u32Interrupt);
    
    // disable timer and its interrupts
    _DisableDrTimer();

    // unregister timer interrupts
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
//if any OS is used
    osi_InterruptDeRegister(u32Interrupt);
    osi_InterruptDeRegister(u32Interrupt + 1);
        
#else
    MAP_TimerIntUnregister(u32DrTimBase, TIMER_BOTH);
#endif

    // reset and stop the timer module
    MAP_PRCMPeripheralReset(u32Peripheral);
    MAP_PRCMPeripheralClkDisable(u32Peripheral, PRCM_RUN_MODE_CLK);
}

static int32_t
_GetPWMTimerInfo(uint32_t u32PinNum, uint32_t *u32Peripheral, uint32_t *u32TimerBase,
                 uint32_t *u32Timer, uint32_t *u32PinMode) {

    // get PWM timer info based on package pin number
    switch(u32PinNum) {
        case PIN_01:
            *u32TimerBase = TIMERA3_BASE;
            *u32Peripheral = PRCM_TIMERA3;
            *u32Timer = TIMER_A;
            *u32PinMode = PIN_MODE_3;
            break;
        case PIN_02:
            *u32TimerBase = TIMERA3_BASE;
            *u32Peripheral = PRCM_TIMERA3;
            *u32Timer = TIMER_B;
            *u32PinMode = PIN_MODE_3;
            break;
        case PIN_17:
            *u32TimerBase = TIMERA0_BASE;
            *u32Peripheral = PRCM_TIMERA0;
            *u32Timer = TIMER_A;
            *u32PinMode = PIN_MODE_5;
            break;
        case PIN_19:
            *u32TimerBase = TIMERA1_BASE;
            *u32Peripheral = PRCM_TIMERA1;
            *u32Timer = TIMER_B;
            *u32PinMode = PIN_MODE_8;
            break;
        case PIN_21:
            *u32TimerBase = TIMERA1_BASE;
            *u32Peripheral = PRCM_TIMERA1;
            *u32Timer = TIMER_A;
            *u32PinMode = PIN_MODE_9;
            break;
        case PIN_64:
            *u32TimerBase = TIMERA2_BASE;
            *u32Peripheral = PRCM_TIMERA2;
            *u32Timer = TIMER_B;
            *u32PinMode = PIN_MODE_3;
            break;
        default:
            return -1;
    }
    return 0;
}

static void
_EnablePWMTimer(void) {

    // set the timer reload and compare values
    MAP_TimerLoadSet(u32PWMTimBase, u32PWMTim, 0xFFFF);
    MAP_TimerMatchSet(u32PWMTimBase, u32PWMTim, 0xFFFF);

    // enable the timer
    MAP_TimerEnable(u32PWMTimBase, u32PWMTim);
}

static void
_DisablePWMTimer(void) {
    // disable the timer
    MAP_TimerDisable(u32PWMTimBase, u32PWMTim);
}

static void
_InitPWMTimer(uint32_t u32PinNum) {

    uint32_t u32Peripheral, u32TimerBase, u32Timer, u32PinMode;

    // get timer info
    _GetPWMTimerInfo(u32PinNum, &u32Peripheral, &u32TimerBase, &u32Timer, &u32PinMode);

    u32PWMTimBase = u32TimerBase;
    u32PWMTim = u32Timer;

    // enable and reset the timer module
    MAP_PRCMPeripheralClkEnable(u32Peripheral, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(u32Peripheral);

    // disable the timer
    MAP_TimerDisable(u32TimerBase, u32Timer);

    // configure the timer to generate PWM signal
    MAP_TimerConfigure(u32TimerBase, (TIMER_CFG_SPLIT_PAIR |
                       (u32Timer == TIMER_A ? TIMER_CFG_A_PWM : TIMER_CFG_B_PWM)));
    MAP_TimerPrescaleSet(u32TimerBase, u32Timer, 0);

    MAP_TimerControlLevel(u32TimerBase, u32Timer, 1);

    // set the load and compare register values
    if (u32Timer == TIMER_A) {
        u32PWMTimLoadReg = u32TimerBase + TIMER_O_TAILR;
        u32PWMTimSwitchReg = u32TimerBase + TIMER_O_TAMATCHR;
    } else {
        u32PWMTimLoadReg = u32TimerBase + TIMER_O_TBILR;
        u32PWMTimSwitchReg = u32TimerBase + TIMER_O_TBMATCHR;
    }
    
    // initialize PWM output pin
    MAP_PinTypeTimer(u32PinNum, u32PinMode);
}

static void
_DeinitPWMTimer(void) {
    
    uint32_t u32Peripheral, u32Interrupt;

    // get timer info
    _GetTimerInfo(u32PWMTimBase, &u32Peripheral, &u32Interrupt);

    // disable timer
    _DisablePWMTimer();

    // stop timer preipheral clock
    MAP_PRCMPeripheralReset(u32Peripheral);
    MAP_PRCMPeripheralClkDisable(u32Peripheral, PRCM_RUN_MODE_CLK);
}

static void
_InitSoGPIO(uint32_t u32GPIONum, uint32_t u32PinNum) {

    uint32_t u32Base, u32Peripheral;
    uint8_t u8Offset;

    // get GPIO info
    _GetGPIOInfo(u32GPIONum, &u32Base, &u8Offset, &u32Peripheral);
    
    // set static variables
    u32GPIOBase = u32Base;
    u8GPIOPin = u8Offset;
    u32GPIOPinReg = u32Base + GPIO_O_GPIO_DATA + (u8Offset << 2);

    // enable the GPIO port
    MAP_PRCMPeripheralClkEnable(u32Peripheral, PRCM_RUN_MODE_CLK);

    // configure pin for GPIOOutput
    MAP_PinTypeGPIO(u32PinNum, PIN_MODE_0, 0);
    MAP_GPIODirModeSet(u32Base, u8Offset, GPIO_DIR_MODE_OUT);
}

static void
_OOK_DrTimerIntHandler(void) {

    // clear timeout interrupt
    HWREG(u32DrTimBase + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT;

	if (u32BitCntr < u32DataLen) {
        // set output pin to next bit value
		HWREG(u32GPIOPinReg) = (u8DataOut[u32BitCntr / 8] & (1 << (7 - u32BitCntr % 8))) ?
                                u8GPIOPin : 0;
		u32BitCntr++;
	} else {
        // stop modulation
        _DisableDrTimer();
        HWREG(u32GPIOPinReg) = 0;
		u8ModReady = 1;
        MOD_IF_u8ModReady = 1;
        if (fnCb != NULL) {
            fnCb(vCbArgs);
        }
	}
}

static void
_BFSK_DrTimerIntHandler(void) {
    // clear timeout interrupt
    HWREG(u32DrTimBase + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT;
    
    if (u32BitCntr < u32DataLen) {
        // set PWM frequency based on current bit value
        if  (u8DataOut[u32BitCntr / 8] & (1 << (7 - u32BitCntr % 8))) {
            HWREG(u32PWMTimLoadReg) = u32PWMTimRldInt2;
            HWREG(u32PWMTimSwitchReg) = u32PWMTimSwtInt2;
        } else {
            HWREG(u32PWMTimLoadReg) = u32PWMTimRldInt1;
            HWREG(u32PWMTimSwitchReg) = u32PWMTimSwtInt1;
        }
		u32BitCntr++;
	} else {
        // stop modulation
        _DisableDrTimer();
        _DisablePWMTimer();
		u8ModReady = 1;
        MOD_IF_u8ModReady = 1;
        if (fnCb != NULL) {
            fnCb(vCbArgs);
        }
	}    
}

static void
_PPM_DrTimerIntHandler(void) {
    // clear timeout interrupt
    HWREG(u32DrTimBase + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT;

    // set the output pin to the current output value
    HWREG(u32GPIOPinReg) = u8PPMNextValue;

    if (u32PPMSlotCntr < u32PPMSlotCntrMax) {

        // increment the slot counter and get the current slot
        u8PPMCurrentSlot = ++u32PPMSlotCntr % u16PPMSlotCnt;

        if (u8PPMCurrentSlot == 0) {
            // `u8PPMBitSymbol` bits have been sent, increment the bit counter
            u32BitCntr += u8PPMBitSymbol;
            if (u32BitCntr >= u32DataLen) {
                // all bits are sent
                u8PPMNextValue = 0;
                return;
            }
        }

        // set `u8PPMNextValue` according to next `u8PPMBitSymbol` bits value
        u8PPMNextValue = (u8PPMCurrentSlot == (uint8_t)((u8DataOut[u32BitCntr / 8] >>
                                (8 - (u32BitCntr % 8 + u8PPMBitSymbol))) & u8PPMMask)) ?
                                u8GPIOPin : 0;
    } else {
        // stop modulation
        _DisableDrTimer();
		u8ModReady = 1;
        HWREG(u32GPIOPinReg) = 0;
        MOD_IF_u8ModReady = 1;
        if (fnCb != NULL) {
            fnCb(vCbArgs);
        }
    }
}

int32_t
MOD_IF_InitModulation_OOK(uint32_t u32DrTimerBase, uint32_t u32SoGPIONum, uint32_t u32SoPinNum) {

    ASSERT_ERROR(u8ModReady || !u8ModInit, MOD_TRY_AGAIN);
    // assert inputs
    ASSERT_INPUT(u32DrTimerBase >= TIMERA0_BASE &&
                 u32DrTimerBase <= TIMERA3_BASE);
    ASSERT_INPUT(u32SoGPIONum <= 31);
    ASSERT_INPUT(u32SoPinNum >= PIN_01 &&
                 u32SoPinNum <= PIN_64);

    u8ModScheme = MOD_OOK;
    MOD_IF_u8ModScheme = MOD_OOK;

    // initialize the data rate timer
    _InitDrTimer(u32DrTimerBase, _OOK_DrTimerIntHandler);

    // initialize the signal output GPIO
    _InitSoGPIO(u32SoGPIONum, u32SoPinNum);
    
    u8ModReady = 1;
    MOD_IF_u8ModReady = 1;

    u8ModInit = 1;
    // success
    return 0;
}

int32_t
MOD_IF_InitModulation_BFSK(uint32_t u32DrTimerBase, uint32_t u32PWMPinNum, uint32_t u32Freq1,
                           uint32_t u32Freq2, uint8_t u8DutyCycle) {

    ASSERT_ERROR(u8ModReady || !u8ModInit, MOD_TRY_AGAIN);

    // assert inputs
    ASSERT_INPUT(u32DrTimerBase >= TIMERA0_BASE && u32DrTimerBase <= TIMERA3_BASE);
    ASSERT_INPUT(u32PWMPinNum == PIN_01 || u32PWMPinNum == PIN_02 || u32PWMPinNum == PIN_17 ||
                    u32PWMPinNum == PIN_19 || u32PWMPinNum == PIN_21 || u32PWMPinNum == PIN_64);
    ASSERT_INPUT(u8DutyCycle >= 0 && u8DutyCycle <= 100);

    // set static variables
    u8ModScheme = MOD_BFSK;
    MOD_IF_u8ModScheme = MOD_BFSK;

    u32PWMTimRldInt1 = (uint32_t)(MOD_CLK_FREQ / (float)u32Freq1 - 0.5);
    u32PWMTimRldInt2 = (uint32_t)(MOD_CLK_FREQ / (float)u32Freq2 - 0.5);

    u32PWMTimSwtInt1 = (uint32_t)(u32PWMTimRldInt1 * u8DutyCycle / 100 + 0.000001);
    u32PWMTimSwtInt2 = (uint32_t)(u32PWMTimRldInt2 * u8DutyCycle / 100 + 0.000001);

    // initialize the data rate timer
    _InitDrTimer(u32DrTimerBase, _BFSK_DrTimerIntHandler);

    // initialize the PWM timer
    _InitPWMTimer(u32PWMPinNum);
    
    u8ModReady = 1;
    MOD_IF_u8ModReady = 1;

    u8ModInit = 1;
    
    // success
    return 0;
}

int32_t
MOD_IF_InitModulation_PPM(uint32_t u32DrTimerBase, uint32_t u32SoGPIONum, uint32_t u32SoPinNum,
                          uint8_t u8BitSymbol) {

    ASSERT_ERROR(u8ModReady || !u8ModInit, MOD_TRY_AGAIN);

    ASSERT_INPUT(u32DrTimerBase >= TIMERA0_BASE && u32DrTimerBase <= TIMERA3_BASE);
    ASSERT_INPUT(u32SoGPIONum <= 31);
    ASSERT_INPUT(u32SoPinNum >= PIN_01 && u32SoPinNum <= PIN_64);
    ASSERT_INPUT(u8BitSymbol == 1 || u8BitSymbol == 2 || u8BitSymbol == 4 || u8BitSymbol == 8);

    u8ModScheme = MOD_PPM;
    MOD_IF_u8ModScheme = MOD_PPM;

    u8PPMBitSymbol = u8BitSymbol;
    u16PPMSlotCnt = 1 << u8BitSymbol;
    u8PPMMask = 0xFF >> 8 - u8BitSymbol;
    
    // initialize the data rate timer
    _InitDrTimer(u32DrTimerBase, _PPM_DrTimerIntHandler);

    // initialize the signal output GPIO
    _InitSoGPIO(u32SoGPIONum, u32SoPinNum);
    
    u8ModReady = 1;
    MOD_IF_u8ModReady = 1;

    u8ModInit = 1;
    
    // success
    return 0;
}

int32_t
MOD_IF_DeinitModulation(void) {

    if (u8ModInit) {
        
        ASSERT_ERROR(u8ModReady, MOD_TRY_AGAIN);

        if (u8ModScheme == MOD_BFSK) {
            _DeinitPWMTimer();
        }
        _DeinitDrTimer();
        u8ModReady = 0;
        MOD_IF_u8ModReady = 0;
        u8ModScheme = 0;
        MOD_IF_u8ModScheme = 0;
        u8ModInit = 0;
    }
    return 0;
}

int32_t
MOD_IF_Modulate(uint8_t *u8Data, uint16_t u16Len, uint32_t u32BitRate, uint8_t u8Blocking,
                MOD_fnCallback_t fnCallback, void *vCallbackArgs) {
    
    // assert inputs
    ASSERT_INPUT(u8Data != NULL);
    ASSERT_INPUT(u16Len > 0);
    ASSERT_INPUT(u32BitRate > 0);
    
    // assert modulation status
    ASSERT_ERROR(u8Blocking || u8ModReady, MOD_TRY_AGAIN);
    // wait for modulator to be ready
    while (!u8ModReady);
    // set static variables
    u8ModReady = 0;
    MOD_IF_u8ModReady = 0;
    
#ifdef MOD_STORE_DATA
    // assert data length
    ASSERT_INPUT(u16Len <= MOD_BUFFER_LEN);

    // copy data into local buffer and set data pointer
    memcpy(u8Buffer, u8Data, u16Len);
    u8DataOut  = u8Buffer;
#else
    // set data pointer
    u8DataOut  = u8Data;
#endif
    
    u32DataLen = u16Len * 8;
    u32BitCntr = 0;

    fnCb = fnCallback;
    vCbArgs = vCallbackArgs;
    
    switch(u8ModScheme) {
        case MOD_BFSK:
            u32DrTimInterval = (uint32_t)(MOD_CLK_FREQ / (float)u32BitRate - 0.5);
            // enable the PWM timer
            _EnablePWMTimer();
            break;
        case MOD_OOK:
            u32DrTimInterval = (uint32_t)(MOD_CLK_FREQ / (float)u32BitRate - 0.5);
            break;
        case MOD_PPM:
            u32PPMSlotCntr = 0;
            u8PPMCurrentSlot = 0;
            u32PPMSlotCntrMax = u16PPMSlotCnt * (u16Len * 8 / u8PPMBitSymbol);
            u8PPMNextValue = u8DataOut[0] & u8PPMMask == 0 ? u8GPIOPin : 0;
            u32DrTimInterval = (uint32_t)(u8PPMBitSymbol * MOD_CLK_FREQ /
                               (float)(u32BitRate * u16PPMSlotCnt) - 0.5);
            break;
        default:
            return MOD_INIT_ERROR;
    }
    // enable the data rate timer
    _EnableDrTimer();
    
    return 0;
}
