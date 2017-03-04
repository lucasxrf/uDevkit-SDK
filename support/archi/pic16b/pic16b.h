/**
 * @file pic16b.h
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2017
 *
 * @date March 12, 2017, 11:35 AM
 *
 * @brief Architecture low level definitions for PIC 16 bits
 */

#ifndef PIC16B_H
#define PIC16B_H

#include <xc.h>

#define nop() __builtin_nop()
#define enable_interrupt() INTCON2bits.GIE=1
#define disable_interrupt() INTCON2bits.GIE=0

#endif // PIC16B_H
