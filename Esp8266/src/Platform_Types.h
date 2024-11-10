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
typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef signed int s32;
typedef unsigned int u32;
typedef signed long long int s64;
typedef unsigned long long int u64;
typedef volatile signed char vs8;
typedef volatile unsigned char vu8;
typedef volatile signed short vs16;
typedef volatile unsigned short vu16;
typedef volatile signed int vs32;
typedef volatile unsigned int vu32;
typedef volatile signed long long int vs64;
typedef volatile unsigned long long int vu64;

typedef float f32;
typedef double f64;

typedef void (*Ptr_Func)(void);

/*----------------------------------------------------------------------------*/
/*-------------------------------- Defines -----------------------------------*/
/*----------------------------------------------------------------------------*/

#define TRUE ((boolean)1)

#define FALSE ((boolean)0)

#define NULL_PTR ((void *)0)


#define AND &&
#define OR ||
#define EQUAL ==
#define NOT_EQUAL !=

#endif /* PLATFORMYPES_H_ */