#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static uint16_t
_CRC16Add(uint8_t u8Byte, uint16_t u16Acc) {
  u16Acc ^= u8Byte;
  u16Acc  = (u16Acc >> 8) | (u16Acc << 8);
  u16Acc ^= (u16Acc & 0xff00) << 4;
  u16Acc ^= (u16Acc >> 8) >> 4;
  u16Acc ^= (u16Acc & 0xff00) >> 5;
  return u16Acc;
}

static uint16_t
_ComputeChecksum(uint8_t *u8pBuffer, uint16_t u16Len)
{
    uint16_t i = 0;
    uint16_t u16Acc = 0;
    for(; i < u16Len; i++) {
        u16Acc = _CRC16Add(u8pBuffer[i], u16Acc);
    }
    return u16Acc;
}

int main(int argc, char **argv)
{
    char hexstring[strlen(argv[1])], *pos = hexstring;
    memcpy(hexstring, argv[1], strlen(argv[1]));
    unsigned char val[strlen(argv[1])/2+2];
    val[0] = strlen(argv[1]) + 4 >> 9;
    val[1] = strlen(argv[1]) + 4 >> 1;
    size_t count = 2;

     /* WARNING: no sanitization or error-checking whatsoever */
    for(; count < sizeof(val)/sizeof(val[0]); count++) {
        sscanf(pos, "%2hhx", &val[count]);
        pos += 2;
    }
    
    
    
    int x = _ComputeChecksum(val, sizeof(val));
    
    for(count = 0; count < sizeof(val)/sizeof(val[0]); count++)
        printf("%02x", val[count]);
    printf("%04x\n", x);
    printf("\n");

    return(0);
}
