/**
 * Filename:    check_sum.c
 * 
 * Description: Calculation check_sum of packets
*/

#ifndef __CHECK_SUM_H__
#define __CHECK_SUM_H__

#include <stdint.h>
#include <stdbool.h>

uint8_t crc8(uint8_t *pcBlock, uint16_t len, const uint8_t poly, const uint8_t init);

unsigned char crc8_dallas(const unsigned char * data, const unsigned int size);


#endif  //__CHECK_SUM_H__
