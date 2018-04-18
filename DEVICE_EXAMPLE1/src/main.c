/**
 * \file
 *
 * \brief CDC Application Main functions
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include "conf_usb.h"
#include "ui.h"
#include "uart.h"
#include "conf_example.h"
#include "adc_configure.h"

static volatile bool main_b_cdc_enable = false;

void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);
void tc_callback(struct tc_module *const module_inst);

#define ADC_SAMPLES 8192
int16_t adc_result_buffer[ADC_SAMPLES];

volatile bool adc_read_done = false;
struct adc_module adc_instance;
struct usart_module console_instance;
float result;
uint16_t raw_result;
uint8_t status;
uint32_t counter;

struct adc_module			adc_instance;
struct tc_module			tc_instance;

static void configure_tc(void)
{
	
	struct tc_config s_configTc;

	tc_get_config_defaults(&s_configTc);

	s_configTc.counter_size = TC_COUNTER_SIZE_16BIT;
	s_configTc.clock_source = GCLK_GENERATOR_3;
	s_configTc.clock_prescaler = TC_CLOCK_PRESCALER_DIV16;
	s_configTc.reload_action = TC_CTRLA_PRESCSYNC_GCLK;

	tc_init(&tc_instance, TC4, &s_configTc);

	tc_enable(&tc_instance);
}

void configure_tc_callbacks(void)
{
	tc_register_callback(&tc_instance, tc_callback,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance, TC_CALLBACK_OVERFLOW);
}

void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);

	config_adc.gain_factor     = ADC_GAIN_FACTOR_1X;
	config_adc.clock_source    = GCLK_GENERATOR_3;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV4;
	config_adc.reference       = ADC_REFERENCE_INTVCC1;
	config_adc.differential_mode = true;
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN0;
	config_adc.negative_input = ADC_NEGATIVE_INPUT_PIN4;
	//config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN6;
	config_adc.resolution      = ADC_RESOLUTION_14BIT;

	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);

	//struct adc_config conf_adc;
	//
	//adc_get_config_defaults(&conf_adc);
	//
	//conf_adc.clock_source = GCLK_GENERATOR_0;
	//conf_adc.reference = ADC_REFERENCE_INTVCC1;
	//conf_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV4;
	//conf_adc.differential_mode = true;
	//conf_adc.positive_input = ADC_POSITIVE_INPUT_PIN0;
	//conf_adc.negative_input = ADC_NEGATIVE_INPUT_PIN4;
	//conf_adc.resolution	= ADC_RESOLUTION_12BIT;
	//conf_adc.reference_compensation_enable = true;
	////conf_adc.event_action = ADC_EVENT_ACTION_START_CONV;
	//conf_adc.gain_factor = ADC_GAIN_FACTOR_1X;
	//
	//adc_init(&adc_instance, ADC, &conf_adc);
	//
	//adc_enable(&adc_instance);
}

void configure_adc_callbacks(void)
{
	adc_register_callback(&adc_instance, adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}

void adc_complete_callback(struct adc_module *const module)
{
	int i;
	//adc_read_done = true;
	adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
	counter += ADC_SAMPLES;
	if(counter >= 8192)
	{
		
		/* Data received */
		ui_com_tx_start();
		for(i=0;i<ADC_SAMPLES;i++)
		{
			/* Transfer UART RX fifo to CDC TX */
			if (!udi_cdc_is_tx_ready()) {
				/* Fifo full */
				udi_cdc_signal_overrun();
				ui_com_overflow();
				} else {
				udi_cdc_putc(adc_result_buffer[i]);
			}
		}
		ui_com_tx_stop();
		
		counter = 0;
	}
}

void tc_callback(struct tc_module *const module_inst)
{
	module_inst->hw->COUNT16.COUNT.reg = 65535 - (500000 / 8192);		//EXECUTION_TIMER_PERIOD sec period
	
	adc_start_conversion(&adc_instance);
}

int main(void)
{
	int16_t c;
	
	struct tc_module       tc_instance;
	
	
	uint16_t adc_result;
	
	irq_initialize_vectors();
	cpu_irq_enable();

	// Initialize the sleep manager
	sleepmgr_init();
	
	/* Configuration of clock and board */
	system_init();
	
	configure_adc();
	configure_adc_callbacks();
	
	configure_tc();
	configure_tc_callbacks();
	
	system_interrupt_set_priority(ADC_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_1);
	system_interrupt_set_priority(TC4_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);
	
	system_interrupt_enable_global();
	
	ui_init();
	ui_powerdown();

	// Start USB stack to authorize VBus monitoring
	udc_start();

	adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);

	// The main loop manages only the power mode
	// because the USB management is done by interrupt
	while (true) {
		sleepmgr_enter_sleep();
		
		
		
	}
}

void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
		return;
	ui_process(udd_get_frame_number());
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	ui_powerdown();
}

void main_remotewakeup_lpm_disable(void)
{
	ui_wakeup_disable();
}

void main_remotewakeup_lpm_enable(void)
{
	ui_wakeup_enable();
}
#endif

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
	uart_open(port);
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	uart_close(port);
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		ui_com_open(port);
	}else{
		// Host terminal has close COM
		ui_com_close(port);
	}
}

/**
 * \mainpage ASF USB Device CDC
 *
 * \section intro Introduction
 * This example shows how to implement a USB Device CDC
 * on Atmel MCU with USB module.
 * The application note AVR4907 provides more information
 * about this implementation.
 *
 * \section desc Description of the Communication Device Class (CDC)
 * The Communication Device Class (CDC) is a general-purpose way to enable all
 * types of communications on the Universal Serial Bus (USB).
 * This class makes it possible to connect communication devices such as
 * digital telephones or analog modems, as well as networking devices
 * like ADSL or Cable modems.
 * While a CDC device enables the implementation of quite complex devices,
 * it can also be used as a very simple method for communication on the USB.
 * For example, a CDC device can appear as a virtual COM port, which greatly
 * simplifies application development on the host side.
 *
 * \section startup Startup
 * The example is a bridge between a USART from the main MCU
 * and the USB CDC interface.
 *
 * In this example, we will use a PC as a USB host:
 * it connects to the USB and to the USART board connector.
 * - Connect the USART peripheral to the USART interface of the board.
 * - Connect the application to a USB host (e.g. a PC)
 *   with a mini-B (embedded side) to A (PC host side) cable.
 * The application will behave as a virtual COM (see Windows Device Manager).
 * - Open a HyperTerminal on both COM ports (RS232 and Virtual COM)
 * - Select the same configuration for both COM ports up to 115200 baud.
 * - Type a character in one HyperTerminal and it will echo in the other.
 *
 * \note
 * On the first connection of the board on the PC,
 * the operating system will detect a new peripheral:
 * - This will open a new hardware installation window.
 * - Choose "No, not this time" to connect to Windows Update for this installation
 * - click "Next"
 * - When requested by Windows for a driver INF file, select the
 *   atmel_devices_cdc.inf file in the directory indicated in the Atmel Studio
 *   "Solution Explorer" window.
 * - click "Next"
 *
 * \copydoc UI
 *
 * \section example About example
 *
 * The example uses the following module groups:
 * - Basic modules:
 *   Startup, board, clock, interrupt, power management
 * - USB Device stack and CDC modules:
 *   <br>services/usb/
 *   <br>services/usb/udc/
 *   <br>services/usb/class/cdc/
 * - Specific implementation:
 *    - main.c,
 *      <br>initializes clock
 *      <br>initializes interrupt
 *      <br>manages UI
 *      <br>
 *    - uart_xmega.c,
 *      <br>implementation of RS232 bridge for XMEGA parts
 *    - uart_uc3.c,
 *      <br>implementation of RS232 bridge for UC3 parts
 *    - uart_sam.c,
 *      <br>implementation of RS232 bridge for SAM parts
 *    - specific implementation for each target "./examples/product_board/":
 *       - conf_foo.h   configuration of each module
 *       - ui.c        implement of user's interface (leds,buttons...)
 */
