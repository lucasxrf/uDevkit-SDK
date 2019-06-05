/**
 * @file nvm_pic24_dspic30f_dspic33.c
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2016
 *
 * @date novembre 10, 2016, 18:01
 *
 * @brief NVM (non volatile memory) support drivers for dsPIC30F, dsPIC33FJ, dsPIC33EP, dsPIC33EV,
 * PIC24F, PIC24FJ, PIC24EP and PIC24HJ
 */

#include "nvm.h"
#include <libpic30.h>
#include <archi.h>

/**
 * @brief Reads a page of flash memory
 * @param address address of the page to read
 * @param ramBuffer array of read data
 * @param size number of words to read
 */
void nvm_read(uint32_t address, uint16_t ramBuffer[], size_t size)
{
    uint16_t offset, i;
    TBLPAG = address >> 16;
    offset = address;//& 0x7FF;
    for (i = 0; i < size; offset+=2)
    {
        //TODO recalculate tblpag and offset if offset overflows
        if (offset > 0xFFFF)
        {
            offset = 0;
            TBLPAG++;
        }
        ramBuffer[i++] = __builtin_tblrdh(offset);
        ramBuffer[i++] = __builtin_tblrdl(offset);
    }
}

/**
 * @brief Erases a page of flash memory
 * @param address address of the page to read
 */
void nvm_erase_page(uint32_t address)
{
    NVMADR = address & 0xF800;
    NVMADRU = address >> 16; // set target write address of general segment

    NVMCON = 0x4003;

    __builtin_disi(6);

    // Disable interrupts for NVM unlock
    __builtin_write_NVM(); // unlock, WR = 1

    while (NVMCONbits.WR == 1);
}

/**
 * @brief Writes two words in flash memory
 * @param address address of the page to read
 * @param data array of the data to write (three 8bits words)
 */
void nvm_write_double_word(uint32_t address, unsigned char data[])
{
    NVMCON = 0x4001;
    TBLPAG = 0xFA; // write latch upper address
    uint16_t offset;

    __builtin_tblwtl(0, (uint16_t)(data[1] << 8) + data[2]);
    __builtin_tblwth(0, (uint8_t)data[0]); // load write latches

    __builtin_tblwtl(2, (uint16_t)(data[4] << 8) + data[5]);
    __builtin_tblwth(2, (uint8_t)data[3]); // load write latches


    offset = address & 0x07FF;
    NVMADR = (address & 0xF800) + offset;
    NVMADRU = address >> 16; // set target write address of general segment

    __builtin_disi(6);
    __builtin_write_NVM();
    while (_WR == 1);
}

/**
 * @brief Writes two words in flash memory
 * @param address address of the page to read
 * @param data array of the data to write (three 8bits words)
 */
void nvm_write(uint32_t address, unsigned char *data, int16_t size)
{
    int32_t i = 0;
    int16_t normalSize = size;
    char newData[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if ((address % 4) != 0)
    {
        normalSize -= 3;
        i = 3;

        newData[3] = data[0];
        newData[4] = data[1];
        newData[5] = data[2];

        address = address & 0xFFFFFFC;

        nvm_write_double_word(address, newData);

        address += 4;
        data += 3;
    }

    while (i < normalSize)
    {
        nvm_write_double_word(address, data);
        address += 4;
        data += 6;
        i += 6;
    }

    if (i < (size - 1))
    {
        newData[0] = data[0];
        newData[1] = data[1];
        newData[2] = data[2];
        newData[3] = 0xFF;
        newData[4] = 0xFF;
        newData[5] = 0xFF;

        nvm_write_double_word(address, newData);
    }
}