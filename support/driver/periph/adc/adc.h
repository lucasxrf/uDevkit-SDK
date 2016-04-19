/**
 * @file adc.h
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2016
 *
 * @date April 19, 2016, 10:56 AM 
 * 
 * @brief ADC driver support
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

int adc_initchannel(uint8_t channel);
int adc_closechannel(uint8_t channel);

uint16_t adc_getValue(uint8_t channel);
int adc_setSamplingPeriod(uint16_t priodMs);

#include "board.h"
#if defined(ARCHI_dspic33ep) || defined(ARCHI_dspic33fj)
 #include "adc_dspic.h"
#else
 #error Unsuported ARCHI
#endif

#endif // ADC_H
