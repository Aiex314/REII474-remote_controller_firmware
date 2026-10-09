/*
 * command_processor.h
 *
 *  Created on: Sep 22, 2026
 *      Author: hapro
 */

#ifndef SRC_COMMAND_PROCESSOR_H_
#define SRC_COMMAND_PROCESSOR_H_

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "main.h"


extern UART_HandleTypeDef* huart_dbg;

void processUART();

uint8_t getDebug();

uint8_t putString(uint8_t* s, uint8_t l);

#endif /* SRC_COMMAND_PROCESSOR_H_ */
