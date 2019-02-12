/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"

#include <string.h>

// Settings
#define BAUDRATE					9600
#define SERIAL_RX_BUFFER_SIZE		1024

// Threads
static THD_FUNCTION(slcd3_process_thread, arg);
static THD_WORKING_AREA(slcd3_process_thread_wa, 4096);
static thread_t *process_tp = 0;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static volatile bool is_running = false;

// Externs
extern volatile float live_pedal_assist_multiplier;

// Private functions
static bool validate_checksum(unsigned char *b);
static void checksum_and_send(unsigned char *data, int len);

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chSysLockFromISR();
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

static void checksum_and_send(unsigned char *data, int len) {
	if (len != 12) {
		return;
	}

	// Wait for the previous transmission to finish.
	while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	uint8_t b[len];
	memcpy(b, data, len);

	//NOTE: b[0] not included inline with a note on endless-sphere.com f=2&t=73471#p1284722
	b[6] = b[1]^b[2]^b[3]^b[4]^b[5]^b[7]^b[8]^b[9]^b[10]^b[11];

	uartStartSend(&HW_UART_DEV, len, b);
}

static bool validate_checksum(unsigned char *b) {
	return (b[5] == (b[0]^b[1]^b[2]^b[3]^b[4]^b[6]^b[7]^b[8]^b[9]^b[10]^b[11]^b[12]));
}

void app_uart_slcd3_start(void) {
	//TODO: any intialization required?
	serial_rx_read_pos = 0;
	serial_rx_write_pos = 0;

	if (!is_running) {
		chThdCreateStatic(slcd3_process_thread_wa, sizeof(slcd3_process_thread_wa),
				NORMALPRIO, slcd3_process_thread, NULL);
		is_running = true;
	}

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
}

void app_uart_slcd3_stop(void) {
	uartStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

	// Notice that the processing thread is kept running in case this call is made from it.
}

void app_uart_slcd3_configure(uint32_t baudrate) {
	uart_cfg.speed = baudrate;

	if (is_running) {
		uartStart(&HW_UART_DEV, &uart_cfg);
	}
}

static THD_FUNCTION(slcd3_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("uart_slcd3 process");

	process_tp = chThdGetSelfX();

	uint8_t rx_frame[13];
	uint8_t tx_frame[] = { 0x41, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t rx_len = 0;

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {
			// sync to beginning of a frame
			rx_frame[rx_len++] = serial_rx_buffer[serial_rx_read_pos++];
			if (rx_len >= 13 || rx_frame[rx_len - 1] == 0xE) {
				if (rx_len == 13 && rx_frame[rx_len - 1] == 0xE) {
					// use data and send response if valid
					if (validate_checksum(rx_frame)) {
						//
						// process data
						//

						// light
						if (0x80 & rx_frame[1]) {
							//TODO: set light output
						}

						// assist level
						switch(0x7 & rx_frame[1])
						{
							case 0:
								live_pedal_assist_multiplier = 0.00;
								break;
							case 1:
								live_pedal_assist_multiplier = 0.20;
								break;
							case 2:
								live_pedal_assist_multiplier = 0.40;
								break;
							case 3:
								live_pedal_assist_multiplier = 0.60;
								break;
							case 4:
								live_pedal_assist_multiplier = 0.80;
								break;
							case 5:
							default:
								live_pedal_assist_multiplier = 1.00;
								break;
							case 6:
								//TODO: walk assist
								break;
						}

						//
						// send data
						//
						//TODO: battery level, throttle/cruise/assist mode, motor temperature

						// motor wattage calculated by LCD using battery voltage
						//NOTE: it's ok to reset average, because app_uartcomm isn't running when we are
						uint8_t batt_cur = (uint8_t)(4 * mc_interface_read_reset_avg_input_current());
						tx_frame[8] = batt_cur;

						// motor rpm in wheel rotation milliseconds
						uint16_t wheel_ms = (uint16_t)(1000 / mc_interface_get_rpm());
						tx_frame[3] = (uint8_t)(wheel_ms >> 8);
						tx_frame[4] = (uint8_t)(wheel_ms);

						checksum_and_send(tx_frame, sizeof(tx_frame));
					}
				}

				// sync
				rx_len = 0;
			}
	
			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}
