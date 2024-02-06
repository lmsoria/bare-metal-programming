#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "core/ring_buffer.h"
#include "core/uart.h"

#define UART_BAUDRATE (115200U)
#define RING_BUFFER_SIZE (128U)

static ring_buffer_t rb = {0};
static uint8_t data_buffer[RING_BUFFER_SIZE] = {0U};

void usart3_isr(void)
{
    const bool overrun_occurred = (usart_get_flag(USART3, USART_FLAG_ORE) == 1);
    const bool received_data = (usart_get_flag(USART3, USART_FLAG_RXNE) == 1);

    if(received_data || overrun_occurred) {
        if(!ring_buffer_write(&rb, (uint8_t)usart_recv(USART3))) {
            // Handle failure?
        }
    }
}

void uart_setup(void)
{
    ring_buffer_setup(&rb, data_buffer, RING_BUFFER_SIZE);

    rcc_periph_clock_enable(RCC_USART3);

    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_databits(USART3, 8);
    usart_set_baudrate(USART3, UART_BAUDRATE);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_stopbits(USART3, USART_STOPBITS_1);

    usart_enable_rx_interrupt(USART3);
    nvic_enable_irq(NVIC_USART3_IRQ);

    usart_enable(USART3);
}

void uart_write(uint8_t* data, const uint32_t length)
{
    for(uint32_t i = 0; i < length; i++) {
        uart_write_byte(data[i]);
    }
}

void uart_write_byte(uint8_t data) { usart_send_blocking(USART3, (uint16_t)data); }

uint32_t uart_read(uint8_t* data, const uint32_t length)
{
    if(length == 0) {
        return 0;
    }

    for(uint32_t bytes_read = 0; bytes_read < length; bytes_read++) {
        if(!ring_buffer_read(&rb, &data[bytes_read])) {
            return bytes_read;
        }
    }

    return length;
}

uint8_t uart_read_byte(void)
{
    uint8_t byte = 0;
    (void)uart_read(&byte, 1);
    return byte;
}

bool uart_data_available(void) { return !ring_buffer_empty(&rb); }
