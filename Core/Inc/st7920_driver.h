/*
 * st7920_driver.h
 *
 *  Created on: Oct 1, 2026
 *      Author: hapro
 */

#ifndef INC_ST7920_DRIVER_H_
#define INC_ST7920_DRIVER_H_

#include "main.h"
#include <string.h>




// Basic commands
void sendLCDcmd(uint8_t cmd);
void sendLCDdata(uint8_t cmd);
void sendDDRAMaddress(uint8_t row, uint8_t col);

void LCD_init();

void LCDsendString(uint8_t* s, uint8_t len, uint8_t line, uint8_t col);

void LCDsetGraphicMode(uint8_t mode);
void LCDclearGraphics(uint8_t c);
void LCDclearBuffer(uint8_t c);
void LCDupdateFromBuffer();
void LCDdrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t col);
void LCDdrawBitmap(const uint8_t* bitmap, uint16_t len);
void LCDdrawBootScreen();

void LCDdrawChar(uint8_t c, uint8_t row, uint8_t col);
void LCDdrawString(uint8_t* s, uint8_t len, uint8_t row, uint8_t col);

#endif /* INC_ST7920_DRIVER_H_ */
