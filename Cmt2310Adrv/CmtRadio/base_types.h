#ifndef __BASE_TYPES_H__
#define __BASE_TYPES_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#ifndef TRUE
/** Value is true (boolean_t type) */
#define TRUE        ((boolean_t) 1u)
#endif

#ifndef FALSE
/** Value is false (boolean_t type) */
#define FALSE       ((boolean_t) 0u)
#endif

#if defined (__ICCARM__)
#define __WEAKDEF            __WEAK __ATTRIBUTES
#elif defined (__CC_ARM)
#define __WEAKDEF            __weak
#else
//		#error	"unsupported compiler!!"
#endif

/** Returns the minimum value out of two values */
#define MIN( X, Y )  ((X) < (Y) ? (X) : (Y))

/** Returns the maximum value out of two values */
#define MAX( X, Y )  ((X) > (Y) ? (X) : (Y))

/** Returns the dimension of an array */
#define ARRAY_SZ( X )  (sizeof(X) / sizeof(X[0]))

#define ASSERT(x)       do{  assert((x)> 0u) ; }while(0);

/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/** logical datatype (only values are TRUE and FALSE) */
typedef uint8_t boolean_t;

/** single precision floating point number (4 byte) */
typedef float float32_t;

/** double precision floating point number (8 byte) */
typedef double float64_t;

/** ASCII character for string generation (8 bit) */
typedef char char_t;

/** function pointer type to void/void function */
typedef void (*func_ptr_t)(void);

/** function pointer type to void/uint8_t function */
typedef void (*func_ptr_arg1_t)(uint8_t);

typedef unsigned char byte;
typedef unsigned char BYTE;

typedef unsigned short word;
typedef unsigned short WORD;

typedef unsigned int lword;
typedef unsigned int LWORD;

typedef unsigned char u8;
typedef unsigned char uchar;
typedef uint16_t u16;
typedef uint32_t u32;

/** generic error codes */
typedef enum en_result
{
    Ok = 0u,  ///< No error
    Error = 1u,  ///< Non-specific error code
    ErrorAddressAlignment = 2u,  ///< Address alignment does not match
    ErrorAccessRights = 3u,  ///< Wrong mode (e.g. user/system) mode is set
    ErrorInvalidParameter = 4u,  ///< Provided parameter is not valid
    ErrorOperationInProgress = 5u,  ///< A conflicting or requested operation is still in progress
    ErrorInvalidMode = 6u,  ///< Operation not allowed in current mode
    ErrorUninitialized = 7u,  ///< Module (or part of it) was not initialized properly
    ErrorBufferFull = 8u,  ///< Circular buffer can not be written because the buffer is full
    ErrorTimeout = 9u,  ///< Time Out error occurred (e.g. I2C arbitration lost, Flash time-out, etc.)
    ErrorNotReady = 10u, ///< A requested final state is not reached
    OperationInProgress = 11u  ///< Indicator for operation in progress
} en_result_t;

#endif /* __BASE_TYPES_H__ */

