/**
 * @file gpio_pic32.c
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2016-2017
 *
 * @date June 6, 2017, 11:02 AM
 *
 * @brief GPIO support driver for PIC32MM, PIC32MK, PIC32MX,
 * PIC32MZDA, PIC32MZEC and PIC32MZEF
 *
 * Implementation based on Microchip document DS60001120F :
 *  http://ww1.microchip.com/downloads/en/DeviceDoc/60001120F.pdf
 */

#include "gpio.h"

#include <archi.h>

/**
 * @brief Set a bit of a GPIO to 1 (HIGHT)
 * @param device GPIO pin
 */
void gpio_setBit(rt_dev_t device)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;
    uint16_t bit = 1 << (device & GPIO_MAX_PORTWIDTHMASK);

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        LATASET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        LATBSET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        LATCSET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        LATDSET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        LATESET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        LATFSET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        LATGSET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        LATHSET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        LATISET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        LATJSET = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        LATKSET = bit;
        break;
#endif
    }
}

/**
 * @brief Set a bit of a GPIO to 0 (LOW)
 * @param device GPIO pin
 */
void gpio_clearBit(rt_dev_t device)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;
    uint16_t bit = 1 << (device & GPIO_MAX_PORTWIDTHMASK);

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        LATACLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        LATBCLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        LATCCLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        LATDCLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        LATECLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        LATFCLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        LATGCLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        LATHCLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        LATICLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        LATJCLR = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        LATKCLR = bit;
        break;
#endif
    }
}

/**
 * @brief Toggle (invert) a bit of a GPIO from 1 to 0 or from 0 to 1
 * @param device GPIO pin
 */
void gpio_toggleBit(rt_dev_t device)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;
    uint16_t bit = 1 << (device & GPIO_MAX_PORTWIDTHMASK);

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        LATAINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        LATBINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        LATCINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        LATDINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        LATEINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        LATFINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        LATGINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        LATHINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        LATIINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        LATJINV = bit;
        break;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        LATKINV = bit;
        break;
#endif
    }
}

/**
 * @brief Write a specific value of a bit of a GPIO
 * @param device GPIO pin
 * @param value value to set (GPIO_LOW / GPIO_HIGH / GPIO_TOGGLE)
 */
void gpio_writeBit(rt_dev_t device, GPIO_VALUE value)
{
    switch(value)
    {
    case GPIO_LOW:
        gpio_clearBit(device);
        break;
    case GPIO_HIGH:
        gpio_setBit(device);
        break;
    case GPIO_TOGGLE:
        gpio_toggleBit(device);
        break;
    }
}

/**
 * @brief Read a bit of a GPIO
 * @param device GPIO pin
 * @return curent value of the bit(GPIO_LOW / GPIO_HIGH)
 */
GPIO_VALUE gpio_readBit(rt_dev_t device)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;
    uint16_t mbit = 1 << (device & GPIO_MAX_PORTWIDTHMASK);
    uint16_t value = 0;

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        value = PORTA;
        break;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        value = PORTB;
        break;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        value = PORTC;
        break;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        value = PORTD;
        break;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        value = PORTE;
        break;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        value = PORTF;
        break;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        value = PORTG;
        break;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        value = PORTH;
        break;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        value = PORTI;
        break;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        value = PORTJ;
        break;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        value = PORTK;
        break;
#endif
    }

    if((value & mbit) == 0)
        return GPIO_LOW;
    else
        return GPIO_HIGH;
}

/**
 * @brief Set GPIO bit configuration (input / output)
 * @param device GPIO pin
 * @param config pin configuration to set (GPIO_OUTPUT / GPIO_INPUT)
 * @return 0 if ok, -1 in case of error
 */
int gpio_setBitConfig(rt_dev_t device, uint16_t config)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;
    uint16_t bit = 1 << (device & GPIO_MAX_PORTWIDTHMASK);

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        if(config & GPIO_OUTPUT)
            TRISACLR = bit; // set as output
        else
            TRISASET = bit; // set as input
        if(config & GPIO_OPENDRAIN)
            ODCACLR = bit; // disable open drain
        else
            ODCASET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUACLR = bit; // disable pull-up
        else
            CNPUASET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDACLR = bit; // disable pull-down
        else
            CNPDASET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        if(config & GPIO_OUTPUT)
            TRISBCLR = bit; // set as output
        else
            TRISBSET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCBCLR = bit; // disable open drain
        else
            ODCBSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUBCLR = bit; // disable pull-up
        else
            CNPUBSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDBCLR = bit; // disable pull-down
        else
            CNPDBSET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        if(config & GPIO_OUTPUT)
            TRISCCLR = bit; // set as output
        else
            TRISCSET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCCCLR = bit; // disable open drain
        else
            ODCCSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUCCLR = bit; // disable pull-up
        else
            CNPUCSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDCCLR = bit; // disable pull-down
        else
            CNPDCSET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        if(config & GPIO_OUTPUT)
            TRISDCLR = bit; // set as output
        else
            TRISDSET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCDCLR = bit; // disable open drain
        else
            ODCDSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUDCLR = bit; // disable pull-up
        else
            CNPUDSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDDCLR = bit; // disable pull-down
        else
            CNPDDSET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        if(config & GPIO_OUTPUT)
            TRISECLR = bit; // set as output
        else
            TRISESET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCECLR = bit; // disable open drain
        else
            ODCESET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUECLR = bit; // disable pull-up
        else
            CNPUESET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDECLR = bit; // disable pull-down
        else
            CNPDESET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        if(config & GPIO_OUTPUT)
            TRISFCLR = bit; // set as output
        else
            TRISFSET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCFCLR = bit; // disable open drain
        else
            ODCFSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUFCLR = bit; // disable pull-up
        else
            CNPUFSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDFCLR = bit; // disable pull-down
        else
            CNPDFSET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        if(config & GPIO_OUTPUT)
            TRISGCLR = bit; // set as output
        else
            TRISGSET = bit;  // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCGCLR = bit; // disable open drain
        else
            ODCGSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUGCLR = bit; // disable pull-up
        else
            CNPUGSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDGCLR = bit; // disable pull-down
        else
            CNPDGSET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        if(config & GPIO_OUTPUT)
            TRISHCLR = bit; // set as output
        else
            TRISHSET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCHCLR = bit; // disable open drain
        else
            ODCHSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUHCLR = bit; // disable pull-up
        else
            CNPUHSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDHCLR = bit; // disable pull-down
        else
            CNPDHSET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        if(config & GPIO_OUTPUT)
            TRISICLR = bit; // set as output
        else
            TRISISET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCICLR = bit; // disable open drain
        else
            ODCISET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUICLR = bit; // disable pull-up
        else
            CNPUISET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDICLR = bit; // disable pull-down
        else
            CNPDISET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        if(config & GPIO_OUTPUT)
            TRISJCLR = bit; // set as output
        else
            TRISJSET = bit; // set as input;
        if(config & GPIO_OPENDRAIN)
            ODCJCLR = bit; // disable open drain
        else
            ODCJSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUJCLR = bit; // disable pull-up
        else
            CNPUJSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDJCLR = bit; // disable pull-down
        else
            CNPDJSET = bit; // enable pull-down
        break;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        if(config & GPIO_OUTPUT)
            TRISKCLR = bit; // set as output
        else
            TRISKSET = bit; // set as input;
        if(config & GPIO_OPENDRKIN)
            ODCKCLR = bit; // disable open drain
        else
            ODCKSET = bit; // enable open drain
        if(config & GPIO_PULLUP)
            CNPUKCLR = bit; // disable pull-up
        else
            CNPUKSET = bit; // enable pull-up
        if(config & GPIO_PULLDOWN)
            CNPDKCLR = bit; // disable pull-down
        else
            CNPDKSET = bit; // enable pull-down
        break;
#endif
    default:
        return -1;
    }

    return 0;
}

/**
 * @brief Set GPIO port value (complete port)
 * @param device GPIO port
 * @param value to set
 */
void gpio_writePort(rt_dev_t device, port_type value)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        LATA = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        LATB = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        LATC = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        LATD = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        LATE = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        LATF = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        LATG = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        LATH = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        LATI = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        LATJ = value;
        break;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        LATK = value;
        break;
#endif
    }
}

/**
 * @brief Read GPIO port value (complete port)
 * @param device GPIO port
 * @return current value of port
 */
port_type gpio_readPort(rt_dev_t device)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        return PORTA;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        return PORTB;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        return PORTC;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        return PORTD;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        return PORTE;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        return PORTF;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        return PORTG;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        return PORTH;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        return PORTI;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        return PORTJ;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        return PORTK;
#endif
    }

    return 0;
}

/**
 * @brief Set GPIO port configuration (input / output)
 * @param device GPIO port
 * @param config port configuration to set (GPIO_OUTPUT / GPIO_INPUT)
 * @return 0 if ok, -1 in case of error
 */
int gpio_setPortConfig(rt_dev_t device, uint16_t config)
{
    uint8_t port = MINOR(device) >> GPIO_MAX_PORTWIDTHU;

    uint16_t trisValue;
    if(config & GPIO_OUTPUT)
        trisValue = 0x0000;
    else
        trisValue = 0xFFFF;

    switch(port)
    {
#ifdef GPIO_HAVE_PORTA
    case 0:
        TRISA = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTB
    case 1:
        TRISB = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTC
    case 2:
        TRISC = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTD
    case 3:
        TRISD = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTE
    case 4:
        TRISE = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTF
    case 5:
        TRISF = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTG
    case 6:
        TRISG = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTH
    case 7:
        TRISH = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTI
    case 8:
        TRISI = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTJ
    case 9:
        TRISJ = trisValue;
        break;
#endif
#ifdef GPIO_HAVE_PORTK
    case 10:
        TRISK = trisValue;
        break;
#endif
    default:
        return -1;
    }

    return 0;
}
