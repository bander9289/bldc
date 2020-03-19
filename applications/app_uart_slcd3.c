/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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
#define PACKET_HANDLER				1
#define PACKET_HANDLER_P			2

// Threads
static THD_FUNCTION(slcd3_process_thread, arg);
static THD_WORKING_AREA(slcd3_process_thread_wa, 4096);

// Variables
static volatile bool thread_is_running = false;
static volatile bool uart_is_running = false;

// Externs
extern volatile float live_pedal_assist_multiplier;

// Private functions
static bool validate_checksum(unsigned char *b);
static void checksum_and_send(unsigned char *data, int len);

static SerialConfig uart_cfg = {
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

#ifdef HW_UART_P_DEV
static volatile bool from_p_uart = false;
static volatile bool uart_p_is_running = false;
static SerialConfig uart_p_cfg = {
		HW_UART_P_BAUD,
		0,
		USART_CR2_LINEN,
		0
};
#endif

static void checksum_and_send(unsigned char *data, int len) {
	if (len != 12) {
		return;
	}

	uint8_t *b = data;

	//NOTE: b[0] not included inline with a note on endless-sphere.com f=2&t=73471#p1284722
	b[6] = b[1]^b[2]^b[3]^b[4]^b[5]^b[7]^b[8]^b[9]^b[10]^b[11];

#ifdef HW_UART_P_DEV
	if (from_p_uart) {
		if (uart_p_is_running) {
			sdWrite(&HW_UART_P_DEV, data, len);
		}
	} else {
		if (uart_is_running) {
			sdWrite(&HW_UART_DEV, data, len);
		}
	}
#else
	if (uart_is_running) {
		sdWrite(&HW_UART_DEV, data, len);
	}
#endif
}

static bool validate_checksum(unsigned char *b) {
	return (b[5] == (b[0]^b[1]^b[2]^b[3]^b[4]^b[6]^b[7]^b[8]^b[9]^b[10]^b[11]^b[12]));
}

void app_uart_slcd3_start(void) {
	//TODO: any intialization required?

	if (!thread_is_running) {
		chThdCreateStatic(slcd3_process_thread_wa, sizeof(slcd3_process_thread_wa),
				NORMALPRIO, slcd3_process_thread, NULL);
		thread_is_running = true;
	}

	sdStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	uart_is_running = true;
}

void app_uart_slcd3_start_permanent(void) {
#ifdef HW_UART_P_DEV
	packet_init(send_packet, process_packet, PACKET_HANDLER);
	packet_init(send_packet, process_packet, PACKET_HANDLER_P);

	if (!thread_is_running) {
		chThdCreateStatic(slcd3_process_thread_wa, sizeof(slcd3_process_thread_wa),
				NORMALPRIO, slcd3_process_thread, NULL);
		thread_is_running = true;
	}

	sdStart(&HW_UART_P_DEV, &uart_p_cfg);
	palSetPadMode(HW_UART_P_TX_PORT, HW_UART_P_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_P_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_P_RX_PORT, HW_UART_P_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_P_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	uart_p_is_running = true;
#endif
}

void app_uart_slcd3_stop(void) {
	sdStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
	uart_is_running = false;

	// Notice that the processing thread is kept running in case this call is made from it.
}

void app_uart_slcd3_configure(uint32_t baudrate) {
	if (baudrate == 0) {
		uart_cfg.speed = BAUDRATE;
	} else {
		uart_cfg.speed = baudrate;
	}

	if (thread_is_running) {
		sdStart(&HW_UART_DEV, &uart_cfg);
		uart_is_running = true;
	}
}

static THD_FUNCTION(slcd3_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("uart_slcd3 process");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&HW_UART_DEV.event, &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

#ifdef HW_UART_P_DEV
	event_listener_t elp;
	chEvtRegisterMaskWithFlags(&HW_UART_P_DEV.event, &elp, EVENT_MASK(0), CHN_INPUT_AVAILABLE);
#endif

	uint8_t rx_frame[13];
	uint8_t tx_frame[] = { 0x41, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t rx_len = 0;

	for(;;) {
		chEvtWaitAny(ALL_EVENTS);

		bool rx = true;
		while (rx) {
			rx = false;

			msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
			if (res != MSG_TIMEOUT) {
#ifdef HW_UART_P_DEV
				from_p_uart = false;
#endif
				// sync to beginning of a frame
				rx_frame[rx_len++] = (uint8_t)res;
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

				rx = true;
			}

#ifdef HW_UART_P_DEV
			res = sdGetTimeout(&HW_UART_P_DEV, TIME_IMMEDIATE);
			if (res != MSG_TIMEOUT) {
				from_p_uart = true;
				packet_process_byte(res, PACKET_HANDLER_P);
				rx = true;
			}
#endif
		}
	}
}
