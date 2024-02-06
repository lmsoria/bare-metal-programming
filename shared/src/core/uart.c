#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "core/uart.h"

static uint8_t data_buffer = 0;
static bool data_available = false;

void usart3_isr(void)
{
    const bool overrun_occurred = (usart_get_flag(USART3, USART_FLAG_ORE) == 1);
    const bool received_data = (usart_get_flag(USART3, USART_FLAG_RXNE) == 1);

    if(received_data || overrun_occurred) {
        data_buffer = usart_recv(USART3);
        data_available = true;
    }
}

void uart_setup(void)
{
    rcc_periph_clock_enable(RCC_USART3);

    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_databits(USART3, 8);
    usart_set_baudrate(USART3, 115200);
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
    if(length > 0 && data_available) {
        *data = data_buffer;
        data_available = false;
        return 1;
    }
    return 0;
}

uint8_t uart_read_byte(void)
{
    data_available = false;
    return data_buffer;
}

bool uart_data_available(void) { return data_available; }
