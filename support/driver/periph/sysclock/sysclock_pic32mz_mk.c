/**
 * @file sysclock_pic32mz_mk.h
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2017
 *
 * @date March 01, 2016, 22:10 PM
 *
 * @brief System clock support for rtprog for PIC32MZ family (DA, EC and EF)
 * PIC32MK and PIC32MM
 *
 * Implementation based on Microchip document DS60001250B :
 *  http://ww1.microchip.com/downloads/en/DeviceDoc/60001250B.pdf
 */

#include "sysclock.h"

#include <archi.h>
#include "board.h"

uint32_t sysclock_sysfreq = 8000000;
uint32_t sysclock_sosc = 0;
uint32_t sysclock_posc = 0;
uint32_t sysclock_pll = 0;
#if defined(ARCHI_pic32mk)
uint32_t sysclock_upll = 0;
#endif

/**
 * @brief Gets the actual frequency on a particular peripherical bus clock
 * @param busClock id of the bus clock (1 to 8 for PBCLK1 to PBCLK8), 0 for sysclock
 * @return bus frequency in Hz
 */
uint32_t sysclock_getPeriphClock(SYSCLOCK_CLOCK busClock)
{
    volatile uint32_t* divisorAddr;
    uint8_t divisor;
    if (busClock == SYSCLOCK_CLOCK_SYSCLK)
        return sysclock_sysfreq;
    if (busClock > SYSCLOCK_CLOCK_PBCLK8)
        return 1; // error, not return 0 to avoid divide by zero
    divisorAddr = &PB1DIV + (((uint8_t)busClock - 1) << 2);
    divisor = ((*divisorAddr) & 0x0000007F) + 1;
    return sysclock_sysfreq / divisor;
}

/**
 * @brief Change the divisor of the busClock given as argument. This can take up to 60
 * CPU cycles.
 * @param busClock id of the bus clock (1 to 8 for PBCLK1 to PBCLK8)
 * @param div divisor to set
 * @return 0 if ok, -1 in case of error
 */
int sysclock_setPeriphClockDiv(SYSCLOCK_CLOCK busClock, uint8_t div)
{
    volatile uint32_t* divisorAddr;

    if (OSCCONbits.CLKLOCK == 1)
        return -1; // Clocks and PLL are locked, source cannot be changed
    if (busClock == SYSCLOCK_CLOCK_SYSCLK) // cannot change sysclock
        return -1;
    if (busClock > SYSCLOCK_CLOCK_PBCLK8)  // bad index
        return -1;
    if (div == 0 || div > 128)
        return -1; // bad divisor value

    // get divisor bus value
    divisorAddr = &PB1DIV + (((uint8_t)busClock - 1) << 2);

    // wait for divisor can be changed
    while((*divisorAddr & _PB1DIV_PBDIVRDY_MASK) == 0)
        nop();

    // critical section, protected by lock on clock config
    unlockClockConfig();
    *divisorAddr = (*divisorAddr & 0xFFFFFF80) + (div - 1);
    lockClockConfig();

    // wait for divisor setted
    while ((*divisorAddr & _PB1DIV_PBDIVRDY_MASK) == 0)
        nop();

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
#if defined(ARCHI_pic32mzda) || defined(ARCHI_pic32mzec) || defined(ARCHI_pic32mzef)
    case SYSCLOCK_SRC_BFRC:
        return 8000000;         // 8MHz BFRC hardware automatic selection
    case SYSCLOCK_SRC_FRC2:
#endif
    case SYSCLOCK_SRC_FRC:
        {
            uint16_t div = OSCCONbits.FRCDIV;
            if (div != 0b111)
                div = 1 << div;
            else
                div = 256;

            return 8000000 / div; // 8MHz FRC // TODO integrate OSCTUNE
        }
    case SYSCLOCK_SRC_LPRC:
        return 32000;         // 32kHz LPRC
    case SYSCLOCK_SRC_SOSC:
        return sysclock_sosc; // external secondary oscilator
    case SYSCLOCK_SRC_POSC:
        return sysclock_posc; // external primary oscilator
    case SYSCLOCK_SRC_SPLL:
        return sysclock_pll;  // PLL out freq
#if defined(ARCHI_pic32mk)
    case SYSCLOCK_SRC_UPLL:
        return sysclock_upll; // USB PLL out freq
#endif
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
#ifdef SYSCLOCK_SRC_FRC2
    if (source == SYSCLOCK_SRC_FRC2)
        return SYSCLOCK_SRC_FRC;
#endif
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

#ifdef SYSCLOCK_SRC_BFRC
    if (source == SYSCLOCK_SRC_BFRC)
        return -2; // cannot switch to backup FRC
#endif

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
    uint32_t fin, fpllo, fsys;
    uint16_t prediv, multiplier, postdiv;

    if (sysclock_source() == SYSCLOCK_SRC_SPLL)
        return -1; // cannot change PLL when it is used

    if (fosc > SYSCLOCK_FOSC_MAX)
        return -1; // cannot generate fosc > SYSCLOCK_FOSC_MAX

#ifdef SYSCLOCK_SRC_FRC2
    if (src == SYSCLOCK_SRC_FRC2 || src == SYSCLOCK_SRC_FRC)
#else
    if (src == SYSCLOCK_SRC_FRC)
#endif
    {
        fin = 8000000;
        SPLLCONbits.PLLICLK = 1; // FRC as input
    }
    else if (src == SYSCLOCK_SRC_POSC)
    {
        fin = SYSCLOCK_XTAL;
        SPLLCONbits.PLLICLK = 0; // POSC as input
    }
    else
    {
        return -1; // source can be only FRC or POSC
    }

    // post div
    postdiv = 32;
    while (fosc * postdiv > SYSCLOCK_FVCO_MAX && postdiv >= 2)
        postdiv = postdiv >> 1;
    if (postdiv < 2)
        return -1;
    fpllo = fosc * postdiv;

    // multiplier

    // pre divisor
    for (prediv = 1; prediv <= 8; prediv++)
    {
        multiplier = fpllo / (fin / prediv);
    }

    // pll range

    // calculate post-diviser and fsys
    /*postdiv = 2;
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
    while (OSCCONbits.LOCK != 1);*/

    sysclock_pll = 0;

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

