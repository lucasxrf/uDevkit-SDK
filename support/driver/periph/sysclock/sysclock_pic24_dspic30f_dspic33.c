/**
 * @file sysclock_pic24_dspic30f_dspic33.c
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2016-2017
 *
 * @date April 11, 2016, 05:12 PM
 *
 * @brief System clock support for rtprog
 */

#include "sysclock_pic24_dspic30f_dspic33.h"

#include <stdint.h>

#include <xc.h>

#include "board.h"

uint32_t sysfreq;

/**
 * @brief Sets the system clock of the CPU, the system clock may be different of CPU
 * clock and peripherical clock
 * @param fosc desirate system frequency in Hz
 * @return 0 if ok, -1 in case of error
 */
int sysclock_setClock(uint32_t fosc)
{
    return sysclock_setClockWPLL(fosc);
}

/**
 * @brief Internal function to set clock with PLL from XTAL or FRC
 * @param fosc desirate system frequency in Hz
 * @return 0 if ok, -1 in case of error
 */
int sysclock_setClockWPLL(uint32_t fosc)
{
    uint32_t fin, fplli, fsys;
    uint16_t multiplier;
    uint16_t prediv, postdiv;

    uint8_t frc_mode;

    if (fosc > SYSCLOCK_FOSC_MAX)
        return -1; // cannot generate fosc > SYSCLOCK_FOSC_MAX

    if (fosc < SYSCLOCK_FSYS_MIN / 8)
        return -1; // cannot generate fosc < SYSCLOCK_FSYS_MIN / 8

#ifndef SYSCLOCK_XTAL
    OSCTUN = 21; // ==> Fin = 8 MHz Internal clock
    fin = 8000000;
    frc_mode = 1;
#else
    fin = SYSCLOCK_XTAL;
    frc_mode = 0;
#endif

    // calculate post-diviser and fsys
    postdiv = 2;
    fsys = fosc << 1;
    if (fsys < SYSCLOCK_FSYS_MIN)
    {
        postdiv = 4;
        fsys = fosc << 2;
    }
    if (fsys < SYSCLOCK_FSYS_MIN)
    {
        postdiv = 8;
        fsys = fosc << 3;
    }

    // calculate pre-diviser to ensure Fplli < SYSCLOCK_FPLLI_MAX
    prediv = (fin / (SYSCLOCK_FPLLI_MAX + 1)) + 1;
    if (prediv < SYSCLOCK_N1_MIN)
        prediv = SYSCLOCK_N1_MIN;
    fplli = fin / prediv;

    // calculate multiplier
    multiplier = fsys / fplli;

    // set post-diviser
    if (postdiv == 2)
        CLKDIVbits.PLLPOST = 0b00; // PLL post div = 2
    if (postdiv == 4)
        CLKDIVbits.PLLPOST = 0b01; // PLL post div = 4
    if (postdiv == 8)
        CLKDIVbits.PLLPOST = 0b11; // PLL post div = 8

    // set pre-diviser
    CLKDIVbits.PLLPRE = prediv - 2; // PLL pre div = 1

    //                         (PLLFBD + 2)
    // Fosc = Fin * ----------------------------------
    //              ((PLLPRE + 2) * 2 * (PLLPOST + 1))
    PLLFBD = multiplier - 2;

    if (frc_mode == 1)
    {
        __builtin_write_OSCCONH(0x01); // frc input
    	__builtin_write_OSCCONL(OSCCON | 0x01);
    }
    else
    {
        __builtin_write_OSCCONH(0x03); // primariry osc input
    	__builtin_write_OSCCONL(OSCCON | 0x01);
	    // Wait for Clock switch to occur
	    while (OSCCONbits.COSC != 0b011);
    }

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1);

    sysfreq = fplli * multiplier / postdiv; // Complete this

    return 0;
}

/**
 * @brief Gets system frequency in Hz
 * @return system frequency in Hz
 */
uint32_t sysclock_getClock()
{
    return sysfreq;
}

/**
 * @brief Gets peripherical clock frequency in Hz
 * @return system frequency in Hz
 */
uint32_t sysclock_getPeriphClock()
{
    return sysfreq >> 1;
}

/**
 * @brief Gets CPU clock frequency in Hz
 * @return system frequency in Hz
 */
uint32_t sysclock_getCPUClock()
{
    return sysfreq >> 1;
}