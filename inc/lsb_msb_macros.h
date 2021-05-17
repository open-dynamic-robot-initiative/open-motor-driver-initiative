#ifndef __LSB_MSB_MACROS_H__
#define __LSB_MSB_MACROS_H__

// 16 bits macros
#define LSB_16(x)             ((x) & 0xFF)
#define MSB_16(x)             (((x) >> 8) & 0xFF)
#define FUS_16_TO_32(x, y)    (((uint32_t)(x) << 16) | (uint32_t)(y))

// 32 bits macros
#define LSB_32(x)             ((x) & 0xFFFF)
#define MSB_32(x)             (((x) >> 16) & 0xFFFF)


#endif
