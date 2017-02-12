#ifndef SPI_IF_
#define SPI_IF_

#define SPI_BASE            GSPI_BASE
#define SPI_PRCM            PRCM_GSPI

int32_t
SPI_IF_Init(uint32_t u32BitRate, uint32_t u32Mode, uint32_t u32SubMode, uint32_t u32Config,
            uint8_t *u8ChipSelGPIOPins, uint8_t u8CSNum);

int32_t
SPI_IF_Send(void *vBuffer, uint32_t u32BufferLen, uint8_t u8CSIdx);

int32_t
SPI_IF_Deinit(void);


#endif
