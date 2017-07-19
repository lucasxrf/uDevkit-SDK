/**
 * @file sysclock_pic32mm.c
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2017
 *
 * @date July 11, 2017, 18:30 PM
 *
 * @brief System clock support for rtprog for PIC32MM family
 *
 * Implementation based on Microchip document DS60001329B :
 *  http://ww1.microchip.com/downloads/en/DeviceDoc/60001329b.pdf
 */

#include "sysclock.h"

#include <archi.h>
#include "board.h"

uint32_t sysclock_sysfreq = 8000000;
uint32_t sysclock_sosc = 0;
uint32_t sysclock_posc = 0;
uint32_t sysclock_pll = 0;

/**
 * @brief Gets the actual frequency on a particular peripherical bus clock
 * @param busClock id of the bus clock
 * @return bus frequency in Hz
 */
uint32_t sysclock_getPeriphClock(SYSCLOCK_CLOCK busClock)
{
    if (busClock == SYSCLOCK_CLOCK_SYSCLK || busClock == SYSCLOCK_CLOCK_PBCLK)
        return sysclock_sysfreq;
    if (busClock == SYSCLOCK_CLOCK_REFCLK)
        return 1; // TODO implement me (refclock computation)
    return 1;
}

/**
 * @brief Change the divisor of the busClock given as argument. This can take up to 60
 * CPU cycles.
 * @param busClock id of the bus clock (SYSCLOCK_CLOCK_REFCLK)
 * @param div divisor to set
 * @return 0 if ok, -1 in case of error
 */
int sysclock_setPeriphClockDiv(SYSCLOCK_CLOCK busClock, uint8_t div)
{
    if (busClock != SYSCLOCK_CLOCK_REFCLK)  // bad index
        return -1;

    // TODO implement me

    return 0;
}

/**
 * @brief Return the actual frequency of the clock source
 * @param source clock id to request
 * @return SYSCLOCK_SOURCE enum corresponding to actual clock source
 */
uint32_t sysclock_getSourceClock(SYSCLOCK_SOURCE source)
{
    switch (source)
    {
    case SYSCLOCK_SRC_LPRC:
        return 32000;         // 32kHz LPRC
    case SYSCLOCK_SRC_SOSC:
        return sysclock_sosc; // external secondary oscilator
    case SYSCLOCK_SRC_POSC:
        return sysclock_posc; // external primary oscilator
    case SYSCLOCK_SRC_SPLL:
        return sysclock_pll;  // PLL out freq
    case SYSCLOCK_SRC_FRC:
        {
            uint16_t div = OSCCONbits.FRCDIV;
            if (div != 0b111)
                div = 1 << div;
            else
                div = 256;

            return 8000000 / div; // 8MHz FRC // TODO integrate OSCTUNE
        }
    }
    return 0;
}

/**
 * @brief Return the actual clock source for system clock
 * @return SYSCLOCK_SOURCE enum corresponding to actual clock source
 */
SYSCLOCK_SOURCE sysclock_source()
{
    SYSCLOCK_SOURCE source = (SYSCLOCK_SOURCE)OSCCONbits.COSC;
    return source;
}

/**
 * @brief Switch the source clock of sysclock to another one and wait for the change effective
 * @param source id to switch to
 * @return 0 if ok, -1 in case of error
 */
int sysclock_switchSourceTo(SYSCLOCK_SOURCE source)
{
    if (OSCCONbits.CLKLOCK == 1)
        return -1; // Clocks and PLL are locked, source cannot be changed

    // disable interrupts
    disable_interrupt();

    // unlock clock config (OSCCON is write protected)
    unlockClockConfig();

    // select the new source
    OSCCONbits.NOSC = source;

    // trigger change
    OSCCONSET = _OSCCON_OSWEN_MASK;
    nop();
    nop();

    // relock clock config
    lockClockConfig();

    while (OSCCONbits.OSWEN == 1)
        nop();

    // enable interrupts
    enable_interrupt();

    if (sysclock_source() != source)
        return -3; // Error when switch clock source

    return 0;
}

/**
 * @brief Sets the system clock of the CPU, the system clock may be different of CPU
 * clock and peripherical clock
 * @param fosc desirate system frequency in Hz
 * @return 0 if ok, -1 in case of error
 */
int sysclock_setClock(uint32_t fosc)
{
    //return sysclock_setClockWPLL(fosc);
    sysclock_sysfreq = fosc;
    return 0;
}

/**
 * @brief Internal function to set clock with PLL from XTAL or FRC
 * @param fosc desirate system frequency in Hz
 * @param src input source clock of PLL (SYSCLOCK_SRC_FRC or SYSCLOCK_SRC_POSC)
 * @return 0 if ok, -1 in case of error
 */
int sysclock_setPLLClock(uint32_t fosc, uint8_t src)
{
    // TODO implement me
    return 0;
}

/**
 * @brief Gets system frequency in Hz
 * @return system frequency in Hz
 */
uint32_t sysclock_getClock()
{
    return sysclock_sysfreq;
}

/**
 * @brief Gets CPU clock frequency in Hz
 * @return cpu frequency in Hz
 */
uint32_t sysclock_getCPUClock()
{
    return sysclock_sysfreq;
}

