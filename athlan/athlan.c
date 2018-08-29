#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "usart.h"

#define LITTLE_BIT 10000000

int main(void) {
	usart3_setup();

	while (1) {
		for (int i = 0; i < LITTLE_BIT; i++) __asm__("nop");
		usart_send_blocking(USART3, 'H');
		usart_send_blocking(USART3, 'i');
		usart_send_blocking(USART3, '\r');
		usart_send_blocking(USART3, '\n');
	}
}
