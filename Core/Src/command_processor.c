/*
 * command_processor.c
 *
 *  Created on: Sep 22, 2026
 *      Author: hapro
 */


#include "command_processor.h"


static uint8_t uart_rx_buf[256];
static uint8_t uart_rx_head = 0, uart_rx_tail = 0, uart_rx_num_chars = 0;

static uint8_t uart_tx_buf[256];
static uint8_t uart_tx_head = 0, uart_tx_tail = 0, uart_tx_num_chars = 0;

// These track whether a character is pending to be sent.
// If not, the TX and RX callbacks will never fire again,
// so any call to putChar or getChar must restart the uart
static uint8_t uart_tx_dead = 1;
static uint8_t uart_rx_dead = 1;


void command_processor_startup() {
  HAL_UART_Receive_IT(huart_dbg, &uart_rx_buf[++uart_rx_head], 1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  // If there's no space in the buffer, stop receiving characters
  if (++uart_rx_num_chars == 255) {
    uart_rx_dead = 1;
    return;
  }

  HAL_GPIO_TogglePin(BLINK_GPIO_Port, BLINK_Pin);

  // Else, receive a new character into the rx buffer
  HAL_UART_Receive_IT(huart, &uart_rx_buf[++uart_rx_head], 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  // If there's nothing left in the buffer, stop transmitting characters
  if (--uart_tx_num_chars == 0) {
    uart_tx_dead = 1;
    return;
  }

  // Else, transmit the next character
  HAL_UART_Transmit_IT(huart, &uart_tx_buf[++uart_tx_tail], 1);
}

uint8_t getChar(uint8_t* c) {
  // If the buffer was full, reception needs to be restarted.
  if (uart_rx_dead == 1) {
    uart_rx_dead = 0;
    HAL_UART_Receive_IT(huart_dbg, &uart_rx_buf[++uart_rx_head], 1);
  }

  // If there are no characters in the buffer, return nothing
  if (uart_rx_num_chars == 0) return 0;

  // Else, get the character at the buffer's tail
  *c = uart_rx_buf[++uart_rx_tail];

  // Decrement number of characters in buffer.
  --uart_rx_num_chars;


  return 1;
}

uint8_t putChar(uint8_t c) {
  // If the buffer was empty, we need to reinitiate transmission
  if (uart_tx_dead == 1) {
    uart_tx_dead = 0;
    HAL_UART_Transmit_IT(huart_dbg, &uart_tx_buf[++uart_tx_tail], 1);
  }

  // If there's no space in the buffer, return error
  if (uart_tx_num_chars == 255) return 0;

  // Else, put the character in the buffer
  uart_tx_buf[++uart_tx_head] = c;

  // Increment the buffer size
  ++uart_tx_num_chars;

  return 1; // success
}

uint8_t putString(uint8_t* s, uint8_t l) {
  for (int i=0; i<l; i++)
    if (putChar(s[i]) == 0) return i;

  return l;
}

void printHex(uint8_t c) {
  const char* hex = "0123456789ABCDEF";

  putString((uint8_t*)"0x", 2);
  putChar(hex[c/16]);
  putChar(hex[c%16]);
  putChar(' ');
}

uint8_t hexToInt(uint8_t* h) {
  uint8_t res = 0;

  for (uint8_t i = 0; i < 2; ++i) {
    res <<= 4;

    uint8_t c = h[i];
    uint8_t c_lp = c & 0x0f;
    if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) res += 9 + c_lp;
    else if (c >= '0' && c <= '9') res += c_lp;
  }

  return res;
}

static uint8_t cmd_buf[16];
static uint8_t cmd_idx = 0;
static uint8_t debug_enabled = 0;

void processCMD() {
  if (!strcmp("debug", (char*)cmd_buf)) {
    debug_enabled = 1;
    uint8_t r = putString((uint8_t*)"Debug enabled\r\n", 15);

    if (r == 15) HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

    return;
  }

  if (debug_enabled != 0) {
    putString((uint8_t*)"\r\n", 2);

    if (!strcmp("hapromeen", (char*)cmd_buf)) {
      putString((uint8_t*)"Hi!", 3);
    }

    putString((uint8_t*)"\r\n", 2);
  }
}

void processUART() {
  uint8_t c;

  while (getChar(&c)) {
    if (c == 0x7F  && cmd_idx > 0)
      --cmd_idx;
    else if (cmd_idx >= 16)
      continue;
    else if (c == 0x0D || c == 0x0A) {
      processCMD();
      cmd_idx = 0;

      continue;
    }
    else if (c == 0x1B)
      cmd_idx = 0;
    else
      cmd_buf[cmd_idx++] = c;


    if (debug_enabled)
      putChar(c);
  }
}

uint8_t getDebug() {
  return (debug_enabled != 0) ? 1 : 0;
}

void setDebug(uint8_t d) {
  debug_enabled = (d == 0) ? 0 : 1;
}
