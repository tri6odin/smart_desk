#pragma once

#include "driver/uart.h"

void configure_uart(uart_port_t uart, int tx_gpio, int rx_gpio);
