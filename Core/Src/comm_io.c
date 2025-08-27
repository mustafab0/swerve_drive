#include "comm_io.h"
#include <string.h>

static UART_HandleTypeDef* uart_handle = NULL;

static uint8_t rx_buffer[COMM_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static uint8_t rx_byte = 0;

void comm_io_init(UART_HandleTypeDef* huart)
{
    uart_handle = huart;
    rx_head = 0;
    rx_tail = 0;
    
    HAL_UART_Receive_IT(uart_handle, &rx_byte, 1);
}

int comm_io_write(const uint8_t* buf, int len)
{
    if (!uart_handle || !buf || len <= 0) return 0;
    
    HAL_UART_Transmit(uart_handle, (uint8_t*)buf, len, 1000);
    
    return len;
}

int comm_io_read(uint8_t* buf, int maxlen)
{
    if (!buf || maxlen <= 0) return 0;
    
    int read_count = 0;
    
    __disable_irq();
    
    while (rx_head != rx_tail && read_count < maxlen)
    {
        buf[read_count++] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % COMM_RX_BUFFER_SIZE;
    }
    
    __enable_irq();
    
    return read_count;
}

int comm_io_getchar(void)
{
    int result = -1;
    
    __disable_irq();
    
    if (rx_head != rx_tail)
    {
        result = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % COMM_RX_BUFFER_SIZE;
    }
    
    __enable_irq();
    
    return result;
}

void comm_io_task_1k(void)
{
    // Nothing needed for polling version
}

void comm_io_on_rx_byte(uint8_t b)
{
    uint16_t next_head = (rx_head + 1) % COMM_RX_BUFFER_SIZE;
    
    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = b;
        rx_head = next_head;
    }
}

void comm_io_on_tx_done(void)
{
    // Nothing needed for polling version
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart_handle)
    {
        comm_io_on_rx_byte(rx_byte);
        HAL_UART_Receive_IT(uart_handle, &rx_byte, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Nothing needed for polling version
}