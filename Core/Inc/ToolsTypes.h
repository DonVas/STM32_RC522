/*
 * ToolsTypes.h
 *
 *  Created on: Feb 21, 2021
 *      Author: vasil003
 */

#ifndef INC_TOOLSTYPES_H_
#define INC_TOOLSTYPES_H_

#include "SPI.h"


#define PROGMEM
#define PGM_P  const char *

#ifndef nullptr
#define nullptr NULL
#endif

#ifndef NULL
#define NULL        ((void*) 0)
#endif

typedef unsigned char       boolean;        /* for use with TRUE/FALSE        */
typedef unsigned char  		byte;
typedef signed char         sint8;          /*        -128 .. +127            */
typedef unsigned char       uint8;          /*           0 .. 255             */
typedef signed short        sint16;         /*      -32768 .. +32767          */
typedef unsigned short      uint16;         /*           0 .. 65535           */
typedef signed long         sint32;         /* -2147483648 .. +2147483647     */
typedef unsigned long       uint32;         /*           0 .. 4294967295      */
typedef void prog_void;
typedef char prog_char;
typedef unsigned char prog_uchar;
typedef int8_t prog_int8_t;
typedef uint8_t prog_uint8_t;
typedef int16_t prog_int16_t;
typedef uint16_t prog_uint16_t;
typedef int32_t prog_int32_t;
typedef uint32_t prog_uint32_t;

#define LOW 0
#define HIGH 1

/* TRUE, FALSE */
#ifndef TRUE
    #define TRUE ((boolean)1u)
    #define true ((boolean)1u)
#endif

#ifndef FALSE
    #define FALSE ((boolean)0u)
    #define false ((boolean)0u)
#endif

#ifndef   __STATIC_INLINE
    #define __STATIC_INLINE    static inline
#endif

/*Serial types*/
typedef enum
{
	STR = 0,
	HEX = 1,
	DEC = 2,
}strDef;

typedef void (*printFunc)(const char*,strDef);
typedef struct Serial {
	boolean     isConnected;
	printFunc	print;
	printFunc	println;
} serial;

/*SPI types*/
typedef HAL_StatusTypeDef (*TxSpiFunc)(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
typedef HAL_StatusTypeDef (*RxSpiFunc)(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
typedef HAL_StatusTypeDef (*TXRXSpiFunc)(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);

typedef struct SPI {
	TxSpiFunc	transfer;
	RxSpiFunc	recive;
	TXRXSpiFunc transmitRecive;
} spi;

#endif /* INC_TOOLSTYPES_H_ */
