/**
 * @file swarmtips.h
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2016
 *
 * @date August 13, 2016, 12:04 PM
 *
 * @brief Definitions for Swarmtips platform from Robotips
 */

#include "swarmtips.h"

int init_io()
{
#ifndef SIMULATOR
    // analog inputs
    ANSELB = 0x0000;    // all analog inputs of port B as digital buffer
    ANSELD = 0x0000;    // all analog inputs of port D as digital buffer
    ANSELE = 0x0000;    // all analog inputs of port E as digital buffer
    ANSELG = 0x0000;    // all analog inputs of port G as digital buffer

    ANSELBbits.ANSB8 = 1;       // BOARD_VOLT_IN as analog

    // digitals outputs
    TRISBbits.TRISB3 = 0;       // LED pin as output
    TRISDbits.TRISD11 = 0;      // LED2 pin as output

    TRISEbits.TRISE4 = 0;       // M1A pin as output
    TRISEbits.TRISE6 = 0;       // M1B pin as output

    TRISEbits.TRISE1 = 0;       // M2A pin as output
    TRISEbits.TRISE3 = 0;       // M2B pin as output

    // remappable pins
    // Unlock configuration pin
    OSCCONL = 0x46; OSCCONL = 0x57; OSCCONbits.IOLOCK = 0;

        // UART1 pins (wifi)
        _U1RXR = 47; // RX1 ==> RPI47
        _RP68R = _RPOUT_U1TX; // TX1 ==> RP68

        // OC PWM motors (motor 1-4)
        _RP85R = _RPOUT_OC1; // OC1 ==> RP85
        _RP87R = _RPOUT_OC2; // OC2 ==> RP87
        _RP80R = _RPOUT_OC3; // OC3 ==> RP80
        _RP82R = _RPOUT_OC4; // OC4 ==> RP82

    // Lock configuration pin
    OSCCONL = 0x46; OSCCONL = 0x57; OSCCONbits.IOLOCK = 1;
#endif
    return 0;
}

int init_board()
{
    init_io();

    return 0;
}