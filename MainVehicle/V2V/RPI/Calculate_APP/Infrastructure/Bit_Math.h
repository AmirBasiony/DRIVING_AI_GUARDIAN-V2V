 /*
 * Bit_Math.h
 *
 * Author: Amir Elbasiony
 */
 
#ifndef UTILS_H_
#define UTILS_H_

// Macro definitions for common bit manipulation operations

// Set a specific bit in a byte
#define SET_BIT(BYTE,BIT_NO) (BYTE |= (1<<BIT_NO))

// Clear a specific bit in a byte
#define CLR_BIT(BYTE,BIT_NO) (BYTE &= ~(1<<BIT_NO))

// Get the value of a specific bit in a byte
#define GET_BIT(BYTE,BIT_NO) ((BYTE >> BIT_NO) & 1) // Alternative: (((BYTE & (1<<BIT_NO)) >> BIT_NO) & 1)

// Toggle the value of a specific bit in a byte
#define TOGGLE_BIT(BYTE,BIT_NO) (BYTE ^= (1<<BIT_NO))

#endif /* UTILS_H_ */
