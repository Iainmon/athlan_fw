#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "usart.h"

#define PORT_USART3_TX GPIOB
#define PIN_USART3_TX 10
#define PORT_USART3_RX GPIOB
#define PIN_USART3_RX 12

struct usart_setup_cfg
{
    uint32_t usart_base;
    enum rcc_periph_clken clken;
    enum rcc_periph_rst reset;
    uint32_t port_tx;
    uint8_t pin_tx;
    uint8_t alt_func_tx;
    uint32_t port_rx;
    uint8_t pin_rx;
    uint8_t alt_func_rx;
    uint8_t irqn;
    uint32_t baudrate;
    uint8_t databits;
    uint32_t stopbits;
    uint32_t mode;
    uint32_t parity;
    uint32_t flowcontrol;
};

struct usart_cfg {
    uint32_t usart;
};

static void usart_setup(struct usart_setup_cfg *cfg, enum rcc_periph_clken gpio_clock)
{
    uint16_t gpio_tx = (1 << cfg->pin_tx);
    uint16_t gpio_rx = (1 << cfg->pin_rx);
    rcc_periph_reset_pulse(cfg->reset);
    rcc_periph_clock_enable(gpio_clock);
    rcc_periph_clock_enable(cfg->clken);
    nvic_enable_irq(cfg->irqn);

    gpio_mode_setup(cfg->port_tx, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_tx);
    gpio_mode_setup(cfg->port_rx, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_rx);
    gpio_set_output_options(cfg->port_rx, GPIO_OTYPE_OD, GPIO_OSPEED_MED, gpio_rx);

    gpio_set_af(cfg->port_tx, cfg->alt_func_tx, gpio_tx);
    gpio_set_af(cfg->port_rx, cfg->alt_func_rx, gpio_rx);

    usart_set_baudrate(cfg->usart_base, cfg->baudrate);
    usart_set_databits(cfg->usart_base, cfg->databits);
    usart_set_stopbits(cfg->usart_base, cfg->stopbits);
    usart_set_mode(cfg->usart_base, cfg->mode);
    usart_set_parity(cfg->usart_base, cfg->parity);
    usart_set_flow_control(cfg->usart_base, cfg->flowcontrol);

    usart_enable_rx_interrupt(cfg->usart_base);
    usart_enable(cfg->usart_base);
}

struct usart_cfg usart3_cfg;

void usart3_setup(void)
{
    struct usart_setup_cfg setup_cfg;

    setup_cfg.usart_base = USART3;
    setup_cfg.clken = RCC_USART3;
    setup_cfg.reset = RST_USART3;
    setup_cfg.port_tx = PORT_USART3_TX;
    setup_cfg.pin_tx = PIN_USART3_TX;
    setup_cfg.alt_func_tx = GPIO_AF7;
    setup_cfg.port_rx = PORT_USART3_RX;
    setup_cfg.pin_rx = PIN_USART3_RX;
    setup_cfg.alt_func_rx = GPIO_AF8;
    setup_cfg.irqn = NVIC_USART3_IRQ;
    setup_cfg.baudrate = 115200;
    setup_cfg.databits = 8;
    setup_cfg.stopbits = USART_STOPBITS_1;
    setup_cfg.mode = USART_MODE_TX_RX;
    setup_cfg.parity = USART_PARITY_NONE;
    setup_cfg.flowcontrol = USART_FLOWCONTROL_NONE;

    usart_setup(&setup_cfg, RCC_GPIOB);

    usart3_cfg.usart = setup_cfg.usart_base;
}