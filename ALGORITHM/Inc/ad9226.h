/**
 * @file ad9226.h
 * @brief AD9226 external 12-bit parallel ADC driver (F407 port).
 */
#ifndef AD9226_H
#define AD9226_H

#include <stdint.h>

#define AD9226_BUF_SIZE 4096

void AD9226_Init(void);
void AD9226_Start(void);

#endif
