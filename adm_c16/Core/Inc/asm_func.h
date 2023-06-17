#pragma once

#include <stdint.h>


void asm_svc (void);
void asm_zeros(uint32_t* vector, uint32_t longitud);
uint32_t asm_sum (uint32_t firstOperand, uint32_t secondOperand);
void asm_escalar32(uint32_t* vectorIn, uint32_t* vectorOut, uint32_t longitud, uint32_t escalar);
void asm_escalar16(uint16_t* vectorIn, uint16_t* vectorOut, uint32_t longitud, uint16_t escalar);
void asm_escalar12(uint16_t* vectorIn, uint16_t* vectorOut, uint32_t longitud, uint16_t escalar);
void asm_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
void asm_pack16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
int32_t asm_max(int32_t * vectorIn, uint32_t longitud);
void asm_downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
void asm_invertir (uint16_t * vector, uint32_t longitud);
void asm_eco(int16_t* vectorIn, int16_t* vectorOut, uint32_t longitud);
