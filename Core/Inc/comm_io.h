#ifndef COMM_IO_H
#define COMM_IO_H

#include "main.h"
#include <stdint.h>

#define COMM_RX_BUFFER_SIZE 256
#define COMM_TX_BUFFER_SIZE 256

void comm_io_init(UART_HandleTypeDef* huart);
int comm_io_write(const uint8_t* buf, int len);
int comm_io_read(uint8_t* buf, int maxlen);
int comm_io_getchar(void);
void comm_io_task_1k(void);

void comm_io_on_rx_byte(uint8_t b);
void comm_io_on_tx_done(void);

#endif