#include "uart_transport.hpp"

#include "driver/gpio.h"
#include "esp_check.h"

namespace {
constexpr int kBaudRate = 9600;
}

void configure_uart(uart_port_t uart, int tx_gpio, int rx_gpio) {
    const uart_config_t uart_config{
        .baud_rate = kBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .rx_glitch_filt_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    int interrupt_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    interrupt_flags = ESP_INTR_FLAG_IRAM;
#endif
    ESP_ERROR_CHECK(uart_driver_install(uart, 1024, 0, 0, nullptr, interrupt_flags));
    ESP_ERROR_CHECK(uart_param_config(uart, &uart_config));
    ESP_ERROR_CHECK(
        uart_set_pin(uart, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(
        gpio_set_pull_mode(static_cast<gpio_num_t>(rx_gpio), GPIO_FLOATING));
}
