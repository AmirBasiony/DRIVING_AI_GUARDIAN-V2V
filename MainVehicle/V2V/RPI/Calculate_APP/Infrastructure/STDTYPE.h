/*
 * STDTYPE.h
 *
 * Author: Amir Elbasiony
 */

#ifndef STDTYPE_H_
#define STDTYPE_H_

// Standard integer types for better code readability

typedef  char sint8;
typedef unsigned char uint8;

typedef signed short sint16;
typedef unsigned short uint16;

typedef int sint32;  //typedef long int sint32;          on 32-bit systems[Raspberry pi]  
typedef unsigned int uint32;//typedef unsigned long int uint32; on 32-bit systems[Raspberry pi] 

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
typedef long double f128;

typedef volatile float vf32;
typedef volatile double vf64;
typedef volatile long double vf128;

// Pointer to a function that takes nothing and returns void
typedef void (*PTR_ToVoidFun_t)(void);
// Boolean type enumeration for true/false values
typedef enum { False = 0, True } Bool_t;
// Enumeration for standard function return status
typedef enum { E_OK = 0, E_NOK } E_STATUS_t;

// Definition of NULL as a void pointer to zero, for null pointer constants
// #define NULL (void*)0
// Macros for logical operations
#define AND &&
#define OR ||
#define EQUAL ==
#define NOT_EQUAL !=

#endif /* STDTYPE_H_ */
