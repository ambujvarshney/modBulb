// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_mcspi.h"
#include "hw_gpio.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "interrupt.h"
#include "gpio.h"
#include "pin.h"
#include "spi_if.h"

#include "common.h"

static uint8_t  u8AutoChipSel;
static uint32_t *u32Base;
static uint8_t  *u8Pin;

static int32_t
_GetChipSelPins(uint8_t *u8ChipSelGPIOPins, uint8_t u8CSNum) {
    uint32_t i;
    uint32_t u32Reg[] = {GPIOA0_BASE, GPIOA1_BASE, GPIOA2_BASE, GPIOA3_BASE, GPIOA4_BASE};
    
    // allocate memory for pins
    u32Base = realloc(u32Base, u8CSNum * sizeof(uint32_t));
    u8Pin  = realloc(u8Pin, u8CSNum * sizeof(uint8_t));
    if (u32Base == 0 || u8Pin == 0) {
        free(u32Base);
        free(u8Pin);
        return -1;
    }

    for (i = 0; i < u8CSNum; i++) {
        u8Pin[i]  = 1 << (u8ChipSelGPIOPins[i] % 8);
        u32Base[i] = u32Reg[u8ChipSelGPIOPins[i] / 8];
    }
    return 0;
}

int32_t
SPI_IF_Init(uint32_t u32BitRate, uint32_t u32Mode, uint32_t u32SubMode, uint32_t u32Config,
            uint8_t *u8ChipSelGPIOPins, uint8_t u8CSNum) {

    uint32_t i;
    
    // init chip select pins
    if (u32Config & SPI_3PIN_MODE) {
        u8AutoChipSel = 0;
        if (u8CSNum > 0) {
            // get port and pin values of GPIO pins
            if (_GetChipSelPins(u8ChipSelGPIOPins, u8CSNum) < 0) {
                return -1;
            }
            // set chip select pin values based on active state
            for (i = 0; i < u8CSNum; i++) {
                MAP_GPIOPinWrite(u32Base[i], u8Pin[i], u32Config & SPI_CS_ACTIVELOW ? u8Pin[i] : 0);
            }
        } else {
            return -1;
        }
    } else {
        u8AutoChipSel = 1;
    }

    // enable clock and reset peripheral
    MAP_PRCMPeripheralClkEnable(SPI_PRCM, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(SPI_PRCM);

    // reset SPI
    MAP_SPIReset(SPI_BASE);
    
    // enable SPI pins
    MAP_PinTypeSPI(PIN_05, PIN_MODE_7);
    MAP_PinTypeSPI(PIN_06, PIN_MODE_7);
    MAP_PinTypeSPI(PIN_07, PIN_MODE_7);
    if (u8AutoChipSel) {
        MAP_PinTypeSPI(PIN_08, PIN_MODE_7);
    }

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(SPI_BASE, MAP_PRCMPeripheralClockGet(SPI_PRCM), u32BitRate, u32Mode, 
                           u32SubMode, u32Config);

    // enable SPI
    MAP_SPIEnable(SPI_BASE);

    return 0;
}

int32_t
SPI_IF_Send(void *vBuffer, uint32_t u32BufferLen, uint8_t u8CSIdx) {

    // enable chip sel pin
    if (!u8AutoChipSel) {
        HWREG(u32Base[u8CSIdx] + (GPIO_O_GPIO_DATA + (u8Pin[u8CSIdx] << 2))) ^= u8Pin[u8CSIdx];
    }

    // send buffer
    UART_PRINT("ret = %d\r\n",MAP_SPITransfer(SPI_BASE, vBuffer, 0, u32BufferLen, SPI_CS_ENABLE | SPI_CS_DISABLE));

    // disable chip sel pin
    if (!u8AutoChipSel) {
        HWREG(u32Base[u8CSIdx] + (GPIO_O_GPIO_DATA + (u8Pin[u8CSIdx] << 2))) ^= u8Pin[u8CSIdx];
    }
    
    return 0;
}

int32_t
SPI_IF_Deinit(void) {
    free(u32Base);
    free(u8Pin);
    u32Base = 0;
    u8Pin = 0;
    MAP_SPIDisable(SPI_BASE);
    MAP_PRCMPeripheralReset(SPI_PRCM);
    MAP_PRCMPeripheralClkDisable(SPI_PRCM, PRCM_RUN_MODE_CLK);
    return 0;
}
