/*
 * st7920_driver.c
 *
 *  Created on: Oct 1, 2026
 *      Author: hapro
 */

#include "st7920_driver.h"

#include "boot_screen.h"
#include "font.h"


// Graphics or text mode
static uint8_t curMode = 0;
// Screen buffer
static uint8_t LCDscreenBuffer[1024];
static uint8_t LCDbufferDirty[1024];

// Microsecond timer and SPI
extern TIM_HandleTypeDef* lcd_us_timer;
extern SPI_HandleTypeDef* lcd_spi;


static void usDelay(uint16_t delay) {
  __HAL_TIM_SET_COUNTER(lcd_us_timer, 0);
  lcd_us_timer->Instance->CNT = 0;

  while (lcd_us_timer->Instance->CNT < delay);
}

void sendLCDcmd(uint8_t cmd) {
  // Setup the frame
  uint8_t frame[3] = {0b11111000, 0, 0}; // Header[RW=0, RS=0] -> Write to command register
  frame[1] =  cmd & 0xf0;
  frame[2] = (cmd & 0x0f) << 4;

  // Transmit the frame
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit(lcd_spi, frame, 3, 10);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

void sendLCDdata(uint8_t cmd) {
  // Setup the frame
  uint8_t frame[3] = {0b11111010, 0, 0}; // Header[RW=0, RS=1] -> Write data to LCD
  frame[1] =  cmd & 0xf0;
  frame[2] = (cmd << 4) & 0xf0;

  // Transmit the frame
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit(lcd_spi, frame, 3, 10);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

void sendLCDdoubleData(uint8_t hi, uint8_t lo) {
  // Setup the frame
  uint8_t frame[5] = {0b11111000, 0, 0, 0, 0};
  frame[1] = hi & 0xf0;
  frame[2] = (hi << 4) & 0xf0;
  frame[3] = lo & 0xf0;
  frame[4] = (lo << 4) & 0xf0;

  // Transmit the frame
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit(lcd_spi, frame, 5, 10);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

void sendDDRAMaddress(uint8_t row, uint8_t col) {
  uint8_t r = row%32;
  uint8_t c = col + (row >= 32 ? 8 : 0);

  // Setup the frame
  uint8_t frame[5] = {0b11111000, 0, 0, 0, 0}; // kill me
  frame[1] = (0x80 | r) & 0xf0;
  frame[2] = (r << 4) & 0xf0;
  frame[3] = (0x80 | c) & 0xf0;
  frame[4] = (c << 4) & 0xf0;

  // Transmit the frame
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit(lcd_spi, frame, 5, 10);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

void LCDinit() {
  // Wait for startup
  HAL_Delay(100); // gotta wait for the LCD to wake up

  // Set to 8-bit mode
  sendLCDcmd(0x30);
  HAL_Delay(1);
  // do it again?
  sendLCDcmd(0x30);
  HAL_Delay(1);

  // Clear screen
  sendLCDcmd(0x01);
  HAL_Delay(2);

  // Setup cursor
  sendLCDcmd(0x06); // Cursor moves right, no shifting
  HAL_Delay(1);
  sendLCDcmd(0x0C); // Display on, cursor on, no blink
  HAL_Delay(1);
  sendLCDcmd(0x02); // Cursor to home
  HAL_Delay(1);
}

void LCDsetGraphicMode(uint8_t mode) {
  if (curMode == mode || mode > 1) return;
  curMode = mode;

  // Go to graphic mode
  if (curMode == 1) {
    sendLCDcmd(0x30); // 8-bit, RE = 0
    usDelay(100);
    sendLCDcmd(0x34); // 8-bit, RE = 1, G = 0
    usDelay(100);
    sendLCDcmd(0x36); // 8-bit, RE = 1, G = 1
    usDelay(100);
  }
  // Go to text mode
  else {
    sendLCDcmd(0x30); // 8-bit, RE = 0, G = 0
    usDelay(100);
  }
}

void LCDclearGraphics(uint8_t l) {
  if (curMode == 0) return;

  // Send 1024 zeroes to clear the screen....
  for (uint8_t r=0; r<64; r++) {
    // Set address per row
    sendDDRAMaddress(r, 0);
    usDelay(10);

    // Send one row's worth of zeros
    for (uint8_t c=0; c<16; c++) {
      sendLCDdata(l);
      usDelay(10);
    }
  }
}

void LCDsendString(uint8_t* s, uint8_t len, uint8_t line, uint8_t col) {
  if (curMode != 0 ) return;

  if (line > 3) return;
  if (col > 15) return;

  uint8_t newDDRAMaddress = 0x80 | (line << 4) | col;

  // Set cursor.
  sendLCDcmd(newDDRAMaddress);
  usDelay(100);

  // Send characters one after another.
  for (uint8_t i=0; i<len; i++) {
    sendLCDdata(s[i]);
    usDelay(100);
  }
}



void LCDclearBuffer(uint8_t c) {
  memset(LCDscreenBuffer, c, 1024);
  memset(LCDbufferDirty, 0x01, 1024);
}

void LCDupdateFromBuffer() {
  if (curMode == 0) return;

  uint8_t addr_updated = 0;

  for (uint8_t r=0; r<64; r++) {
    // The ST7920 groups horizontal runs of pixels into 16-bits.
    // 128/8 gives 16 8-bit groups (as in the buffer), but we
    // check and send two of these at a time (because 16-bit runs).
    // That's why c<16; c+=2, and why we consider i and i+1.
    for (uint8_t c=0; c<16; c+=2) {
      // Get the array index
      uint16_t i = r*16 + c%16;

      // Test dirty flag
      if ((LCDbufferDirty[i] | LCDbufferDirty[i+1]) == 0) {
        // Once we break a run of transmitted bytes it is necessary
        // to resend the address once we start transmitting bytes again
        addr_updated = 0;
        continue;
      }
      // Clear dirty flag
      LCDbufferDirty[i] = LCDbufferDirty[i+1] = 0;

      // If the address counter is correct, just send the bytes
      // Otherwise, resend the address
      if (addr_updated == 0) {
        sendDDRAMaddress(r, c/2); // c/2 is again due to weirdness in ST7920
        usDelay(10);
      }

      // Send the bytes
      sendLCDdata(LCDscreenBuffer[i]);
      sendLCDdata(LCDscreenBuffer[i+1]);
      usDelay(10);
    }

    // After each row the address resets to the beginning of the SAME row
    // Need to resend address at least once per row...
    addr_updated = 0;
  }
}

// Not the strangest code I've written.
// These generate the left and right column of pixels
#define LEFT_BYTE(x) (1<<(8-(unsigned short)x%8))-1
#define RIGHT_BYTE(x) ((1<<((unsigned short)x%8))-1)<<(8-x)

void LCDdrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t col) {
  uint8_t startIndexX = x/8, endIndexX = (w+x)/8;
  uint8_t startIndexY = y,   endIndexY = y+h;

  uint8_t startParity = x%8, endParity = (w+x)%8;

  uint8_t leftByte =  LEFT_BYTE(startParity);
  uint8_t rightByte = RIGHT_BYTE(endParity);
  if (startIndexX == endIndexX) leftByte &= rightByte;

  for (uint8_t i_x = startIndexX; i_x <= endIndexX; i_x++) {
    for (uint8_t i_y = startIndexY; i_y < endIndexY; i_y++) {
      if (i_x == endIndexX && endParity == 0) continue;

      uint16_t index = i_y*16 + i_x;
      if (col != 0) {
        if (i_x == startIndexX)
          LCDscreenBuffer[index] |= leftByte;
        else if (i_x == endIndexX && endParity != 0)
          LCDscreenBuffer[index] |= rightByte;
        else
          LCDscreenBuffer[index] = 0xff;
      }
      else {
        if (i_x == startIndexX)
          LCDscreenBuffer[index] &= ~leftByte;
        else if (i_x == endIndexX && endParity != 0)
          LCDscreenBuffer[index] &= ~rightByte;
        else
          LCDscreenBuffer[index] = 0x00;
      }

      LCDbufferDirty[index] = 0x01;
    }
  }
}

#define XOR(a, b) ((a & ~b) | (b & ~a))

void LCDxorRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  uint8_t startIndexX = x/8, endIndexX = (w+x)/8;
  uint8_t startIndexY = y,   endIndexY = y+h;

  uint8_t startParity = x%8, endParity = (w+x)%8;

  uint8_t leftByte =  LEFT_BYTE(startParity);
  uint8_t rightByte = RIGHT_BYTE(endParity);
  if (startIndexX == endIndexX) leftByte &= rightByte;

  for (uint8_t i_x = startIndexX; i_x <= endIndexX; i_x++) {
    for (uint8_t i_y = startIndexY; i_y < endIndexY; i_y++) {
      if (i_x == endIndexX && endParity == 0) continue;

      uint16_t index = i_y*16 + i_x;
      if (i_x == startIndexX)
        LCDscreenBuffer[index] = XOR(leftByte, LCDscreenBuffer[index]);
      else if (i_x == endIndexX && endParity != 0)
        LCDscreenBuffer[index] = XOR(rightByte, LCDscreenBuffer[index]);
      else
        LCDscreenBuffer[index] = XOR(LCDscreenBuffer[index], 0xff);

      LCDbufferDirty[index] = 0x01;
    }
  }
}

void LCDdrawBitmap(const uint8_t* bitmap, uint16_t len) {
  memcpy(LCDscreenBuffer, bitmap, len);
  memset(LCDbufferDirty, 1, len);
}

void LCDdrawBootScreen() {
  LCDdrawBitmap(boot_screen_bitmap, 1024);
}

void LCDdrawChar(uint8_t c, uint8_t row, uint8_t col) {
  uint16_t char_index = (127-c) * 8;

  for (uint8_t i=0; i<8; i++) {
    uint16_t line = font_array[7-i+char_index] << (8-(col%8));
    uint16_t mask = 0xfc << (8-(col%8));

    uint16_t buf_idx = (col/8) + (row+i)*16;

    LCDscreenBuffer[buf_idx] &= ~((mask >> 8)&0xff);
    LCDscreenBuffer[buf_idx] |=  (line&0xff00) >> 8;
    LCDbufferDirty[buf_idx]  = 1;

    if (col%8 > 2) {
      LCDscreenBuffer[buf_idx+1] &= ~(mask & 0xff);
      LCDscreenBuffer[buf_idx+1] |=  (line&0x00ff);
      LCDbufferDirty[buf_idx+1]  = 1;
    }

  }
}

void LCDdrawString(uint8_t* s, uint8_t len, uint8_t row, uint8_t col) {
  for (uint16_t i = 0; i < len; i++)
    LCDdrawChar(s[i], row, col+i*6);
}

void LCDdrawEmphString(uint8_t* s, uint8_t len, uint8_t row, uint8_t col) {
  LCDdrawRect(col, row, 6*len+1, 9, 0);
  LCDdrawString(s, len, row+1, col+1);
  LCDxorRect(col, row, 6*len+1, 9);
}

void LCDdrawStringDeemph(uint8_t* s, uint8_t len, uint8_t row, uint8_t col) {
  LCDdrawRect(col, row, 6*len+1, 9, 0);
  LCDdrawString(s, len, row+1, col+1);
}
