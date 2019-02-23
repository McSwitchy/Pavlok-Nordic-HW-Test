#include "debug.h"
#include "boards.h"
#include "app_uart.h"
#include "nrf_drv_uart.h"
#include "nrf_delay.h"

char debugBuffer[UART_TX_BUF_SIZE];

void uart_event_handler(app_uart_evt_t * p_event) {
	// This function is required by APP_UART_FIFO_INIT, but we don't need to
  // handle any events here.
}

void init_uart(void) {
	// Initializes the UART for debugging. Baud rate is specified at 38400.
	uint32_t err_code;
	app_uart_comm_params_t const comm_params = {
			.rx_pin_no    = RX_PIN_NUMBER,
			.tx_pin_no    = TX_PIN_NUMBER,
			.rts_pin_no   = RTS_PIN_NUMBER,
			.cts_pin_no   = CTS_PIN_NUMBER,
			.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
			.use_parity   = false,
			.baud_rate    =  UART_BAUDRATE_BAUDRATE_Baud115200/*UART_BAUDRATE_BAUDRATE_Baud38400*/
	};
	APP_UART_FIFO_INIT(&comm_params,
										 UART_RX_BUF_SIZE,
										 UART_TX_BUF_SIZE,
										 uart_event_handler,
										 APP_IRQ_PRIORITY_LOWEST,
										 err_code);
	APP_ERROR_CHECK(err_code);

}

void debug_uart(void) {
	uint16_t i = 0;
	uint16_t len = strlen(debugBuffer);
	
	for (i=0; i<len; i++) {
		app_uart_put(debugBuffer[i]);
		nrf_delay_us(100);
	}
}
