/*
 * Platformypes.h
 *
 *     Author : Hady Abdelhady
 */

#ifndef PLATFORM_TYPES_H_
#define PLATFORM_TYPES_H_
/*----------------------------------------------------------------------------*/
/*-------------------------------- Types -------------------------------------*/
/*----------------------------------------------------------------------------*/

// typedef bool                       boolean;
typedef signed char sint8;
typedef unsigned char uint8;
typedef signed short sint16;
typedef unsigned short uint16;
typedef signed int sint32;
typedef unsigned int uint32;
typedef signed long long int sint64;
typedef unsigned long long int uint64;
typedef volatile signed char vsint8;
typedef volatile unsigned char vuint8;
typedef volatile signed short vsint16;
typedef volatile unsigned short vuint16;
typedef volatile signed int vsint32;
typedef volatile unsigned int vuint32;
typedef volatile signed long long int vsint64;
typedef volatile unsigned long long int vuint64;

typedef float f32;
typedef double f64;

typedef void (*Ptr_Func)(void);

/*----------------------------------------------------------------------------*/
/*-------------------------------- Defines -----------------------------------*/
/*----------------------------------------------------------------------------*/

#define TRUE ((boolean)1)

#define FALSE ((boolean)0)

#define NULL ((void *)0)


#define AND &&
#define OR ||
#define EQUAL ==
#define NOT_EQUAL !=

#endif /* PLATFORMYPES_H_ */