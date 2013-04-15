#ifndef UCORE_LWIP_CC_H_
#define UCORE_LWIP_CC_H_

#ifdef __KERNEL__
#include <types.h>
#include <assert.h>
#include <stdio.h>
#endif
 
typedef unsigned char u8_t;
typedef signed char s8_t;
typedef unsigned short u16_t;
typedef signed short s16_t;
typedef unsigned int u32_t;
typedef signed short s32_t;
typedef unsigned int mem_ptr_t;

#define U16_F "uh"
#define S16_F "dh"
#define X16_F "04xh"
#define U32_F "ul"
#define S32_F "dl"
#define X32_F "08xl"

#define LWIP_PROVIDE_ERRNO

#define PACK_STRUCT_FIELD(x)    x
#define PACK_STRUCT_STRUCT      __attribute__((__packed__))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

#define DBG_OFF 0x00

#define LWIP_PLATFORM_DEBUG(x)  cprintf x
#define LWIP_PLATFORM_DIAG(x)  cprintf x
#define LWIP_PLATFORM_ASSERT(x) assert(x)

#endif // UCORE_LWIP_CC_H_
