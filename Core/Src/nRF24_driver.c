/*
 * nRF24.c
 *
 *  Created on: Aug 29, 2026
 *      Author: hapro
 */

#include "nRF24_driver.h"


extern SPI_HandleTypeDef* nrf_spi;

uint8_t spi_out_data[33]; // Maximum SPI frame size is 1B + 32B for W_TX_COMMAND with 32B of data


// Single byte register setting and reading functions
void writeNRF24register(uint8_t index, uint8_t value) {
  spi_out_data[0] = 0b00100000 | (0b00011111 & index);
  spi_out_data[1] = value;

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 2, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

uint8_t readNRF24register(uint8_t index) {
  spi_out_data[0] = 0b00011111 & index;

  uint8_t reg_data = 0x00;

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1, 10);
  HAL_SPI_Receive(nrf_spi, &reg_data, 1, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  return reg_data;
}

uint8_t readNRF24status() {
  uint8_t status_reg = 0x00;

  spi_out_data[0] = N24_NOP;

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(nrf_spi, spi_out_data, &status_reg, 1, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  return status_reg;
}


// Address setting functions. These don't check against the configured
// address width, although I'm not sure how you would...
void setNRF24TXaddr(const uint8_t* addr, uint8_t addr_len) {
  spi_out_data[0] = 0b00100000 | N24_TX_ADDR;  // Write to TX_ADDR register
  memcpy(&spi_out_data[1], addr, addr_len*sizeof(uint8_t));

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1+addr_len, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void setNRF24RXaddr(const uint8_t* addr, uint8_t addr_len, uint8_t pipe_no) {
  if (pipe_no > 5) return; // If pipe index out of bounds, just fail silently

  spi_out_data[0] = 0b00100000 | (pipe_no + N24_RX_ADDR_P0);  // Write to RX_ADDR register corresponding to the pipe
  memcpy(&spi_out_data[1], addr, addr_len*sizeof(uint8_t));

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1+addr_len, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}


// Payload transmission and reception functions 
void sendNRF24payload(const uint8_t* data, uint8_t data_len) {
  spi_out_data[0] = N24_W_TX_PAYLOAD; // Write payload command
  memcpy(&spi_out_data[1], data, data_len);
  
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1+data_len, 10);

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
  spi_out_data[0] = N24_NOP;
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1, 10);
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

void readNRF24payload(uint8_t* buffer, uint8_t data_len) {
  // Is it possible to somehow infer the data_len other than using R_RX_PL_WID?
  // The payload width field isn't necessarily set, so this would be unreliable anyways...

  spi_out_data[0] = N24_R_RX_PAYLOAD; // Read payload command

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1, 10);
  HAL_SPI_Receive(nrf_spi, buffer, data_len, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void writeNRF24ackPayload(const uint8_t* data, uint8_t data_len, uint8_t pipe_no) {
  if (pipe_no > 5) return;

  spi_out_data[0] = N24_W_ACK_PAYLOAD + pipe_no;
  memcpy(&spi_out_data[1], data, data_len);

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, data, 1+data_len, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}


// Miscellaneous things...
void toggleNRF24extraFeatures() {
  spi_out_data[0] = N24_ACTIVATE;
  spi_out_data[1] = 0x73;

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 2, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void flushNRF24buffers() {
  spi_out_data[0] = N24_FLUSH_RX;

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  spi_out_data[0] = N24_FLUSH_TX;

  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(nrf_spi, spi_out_data, 1, 10);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}



// This is used to get the same setup on the PRX and PTX
void NRF24init() {
  // Turn off module while configuring it.
  writeNRF24register(N24_CONFIG,    0b01111000); // Mask all interrupts, enable 1B CRC, keep power down, act as PTX
  readNRF24register(N24_CONFIG);

  // Setup extra features.
  toggleNRF24extraFeatures();
  writeNRF24register(N24_FEATURE,   0b00000111); // Enable all acknowledge payload features
  writeNRF24register(N24_DYNPD,     0b00000001); // Enable dynamic payload in pipe 0

  // Miscellaneous configuration settings.
  writeNRF24register(N24_EN_AA,     0b00000001); // Only enable auto acknowledge on pipe 0
  writeNRF24register(N24_EN_RXADDR, 0b00000001); // Only enable reception on pipe 0
  writeNRF24register(N24_SETUP_AW,  0b00000011); // Set 5 byte address width
  writeNRF24register(N24_SETUP_RETR,0b00010101); // 5 retransmissions, each spaced 500us apart
  writeNRF24register(N24_RF_CH,     0b00010010); // Set frequency channel to 2... this is the datasheet's default, so I'll trust it for now
  writeNRF24register(N24_RF_SETUP,  0b00000111); // 1Mbps tx rate, 0dBm tx gain, setup LNA gain (idk what that means)

  // Setup TX and RX addresses.
  const uint8_t device_addr[5] = {0b01101001, 0b10010110, 0b10010110, 0b01101001, 0b10101010};
  setNRF24TXaddr(device_addr, 5);    // Configure TX address
  setNRF24RXaddr(device_addr, 5, 0); // Configure RX address for pipe 0 to be the same as TX address

  // Flush FIFO buffers
  flushNRF24buffers();

  // Final configuration, and turn on the module
  writeNRF24register(N24_CONFIG, 0b01111010); // Power up the nRF24, 1B crc, act as PTX
  readNRF24register(N24_CONFIG);
}
