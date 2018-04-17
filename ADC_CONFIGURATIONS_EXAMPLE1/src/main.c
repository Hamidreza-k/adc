/**
 * \file
 *
 * \brief SAMD21 ADC Configuration example
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
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
* \mainpage
 * \section intro Introduction
 * This example demonstrates how to use enable and configure different features of ADC module.
 *
 * \section files Main Files
 * - conf_board.h: Board Configuration
 * - conf_example.h: ADC Features Configuration
 * - conf_clocks.h: SAM D21 Clock configuration
 * - adc_configure.h: ADC configuration prototype declarations
 * - adc_configure.c: ADC configuration definitions
 * - adc_temp.h: ADC temperature sensor prototype declarations
 * - adc_temp.c: ADC temperature calculation and definitions
 * 

 
 * This example has been tested with the following setup:
 *   - SAMD21 Xplained Pro 
 *   - UART configuration is 115200 baudrate, no parity, data 8 bit.
 *
 * \section example description Description of the example
 * The example helps to configure the ADC module 
 * based on macro definition available in conf_example.h.
 * The Serial Console used to get input from user where necessary
 * display the ADC input voltage after the ADC configuration done.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/avr">Atmel AVR</A>.\n
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
 
#include <asf.h>
#include "conf_example.h"
#include "adc_configure.h"
#include "adc_temp.h"
#include "conf_qs_events_interrupt_hook.h"

void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);

#define ADC_SAMPLES 16
uint16_t adc_result_buffer[ADC_SAMPLES];

struct adc_module adc_instance;
volatile bool adc_read_done = false;

/* Structure for ADC module instance */
struct adc_module adc_instance;

/* Structure for USART module instance */
struct usart_module console_instance;

/*  To Store ADC output in voltage format */
float result;

/* To store raw_result of ADC output */
uint16_t raw_result;

/* To read the STATUS register of ADC */
uint8_t status;

static volatile uint32_t event_count = 0;

void event_counter(struct events_resource *resource);

struct tc_module			tc_instance;

void tc_callback(struct tc_module *const module_inst);

static void configure_event_channel(struct events_resource *resource)
{
	//! [setup_1]
	struct events_config config;
	//! [setup_1]

	//! [setup_2]
	events_get_config_defaults(&config);
	//! [setup_2]

	//! [setup_3]
	config.generator      = EVSYS_ID_GEN_TC4_OVF;
	config.edge_detect    = EVENTS_EDGE_DETECT_RISING;
	config.path           = EVENTS_PATH_SYNCHRONOUS;
	config.clock_source   = GCLK_GENERATOR_0;
	//! [setup_3]

	//! [setup_4]
	events_allocate(resource, &config);
	//! [setup_4]
}

static void configure_event_user(struct events_resource *resource)
{
	//! [setup_5]
	events_attach_user(resource, EVSYS_ID_USER_ADC_START);
	//! [setup_5]
}

static void configure_tc(void)
{
	
	struct tc_config s_configTc;

	tc_get_config_defaults(&s_configTc);

	s_configTc.counter_size = TC_COUNTER_SIZE_16BIT;
	s_configTc.clock_source = GCLK_GENERATOR_0;
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

static void configure_event_interrupt(struct events_resource *resource,	struct events_hook *hook)
{
	//! [setup_12]
	events_create_hook(hook, event_counter);
	//! [setup_12]

	//! [setup_13]
	events_add_hook(resource, hook);
	events_enable_interrupt_source(resource, EVENTS_INTERRUPT_DETECT);
	//! [setup_13]
}

void event_counter(struct events_resource *resource)
{
	if(events_is_interrupt_set(resource, EVENTS_INTERRUPT_DETECT)) {
		port_pin_toggle_output_level(LED_0_PIN);

		event_count++;
		events_ack_interrupt(resource, EVENTS_INTERRUPT_DETECT);
	}
}

void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);

	config_adc.gain_factor     = ADC_GAIN_FACTOR_1X;
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

uint32_t counter;

void adc_complete_callback(struct adc_module *const module)
{
	//adc_read_done = true;
	adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
	counter += 16;
	if(counter >= 8192)
	{
		counter = 0;
	}
}

void tc_callback(struct tc_module *const module_inst)
{	
	module_inst->hw->COUNT16.COUNT.reg = 65535 - (500000 / 8192 - 1);		//EXECUTION_TIMER_PERIOD sec period
	
	adc_start_conversion(&adc_instance);
}

int main(void)
{
	struct tc_module       tc_instance;
	struct events_resource example_event;
	struct events_hook     hook;
	uint16_t adc_result;
	/* Configuration of clock and board */
	system_init();
	
	configure_adc();
	configure_adc_callbacks();
	
	configure_tc();
	configure_tc_callbacks();
	
	system_interrupt_set_priority(ADC_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_1);
	system_interrupt_set_priority(TC4_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);
	
	system_interrupt_enable_global();

	//configure_event_channel(&example_event);
	//configure_event_user(&example_event);
	//configure_event_interrupt(&example_event, &hook);
	//configure_tc(&tc_instance);
	
	/* Serial Console configuration */
	//configure_console();
	
	/* Delay Module Initialization */
	//delay_init();
	
	//while (events_is_busy(&example_event)) {
		/* Wait for channel */
	//};
	
	adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
	
	//tc_start_counter(&tc_instance);
	
	//while (adc_read_done == false) {}
	
	while (true)
	{
	//adc_differential();
	
	///* Differential Mode configuration */
	//#if (ADC_MODE_DIFFERENTIAL == ENABLE)
	//adc_differential();
	//#endif
	//
	///* Hardware Averaging configuration */
	//#if (ADC_MODE_HW_AVERAGING == ENABLE)
	//adc_hardware_averaging();
	//#endif
	//
	///* Oversampling configuration */
	//#if (ADC_MODE_OVERSAMPLING == ENABLE)
	//adc_oversampling();
	//#endif
	//
	///* Window Monitoring configuration */
	//#if (ADC_MODE_WINDOW == ENABLE)
	//adc_window_monitor();
	//#endif
	//
	///* ADC calibration configuration */
	//#if (ADC_MODE_CALIBRATION == ENABLE)
	//adc_calibration();
	//#endif	
	//
	///* Temperature Sensor configuration */
	//#if (ADC_MODE_TEMP_SENSOR == ENABLE)
	//adc_temp_sensor(); 
	//#endif	
	//
	//delay_s(1);
	
	}
	
}

/**
* \brief ADC START and Read Result.
*
* This function starts the ADC and wait for the ADC
* conversation to be complete	and read the ADC result
* register and return the same to calling function.
*/

uint16_t adc_start_read_result(void)
{
	uint16_t adc_result = 0;
	
	adc_start_conversion(&adc_instance);
	while((adc_get_status(&adc_instance) & ADC_STATUS_RESULT_READY) != 1);
		
	adc_read(&adc_instance, &adc_result);
	
	return adc_result;
}


/**
* \brief Configure serial Console.
*
* This function configures and enable the SERCOM
* module with below Settings.

* GLCK for SERCOM	-> GCLK_GENERATOR_0 (8MHz)
* SERCOM instance	-> 3
* TXD				-> PA22
* RXD				-> PA23
* BAUDRATE			-> 115200
* STOP BIT			-> 1
* CHARACTER			-> 8
* PARITY			-> NONE
*
*/

void configure_console(void)
{
	
	struct usart_config conf_usart;
	
	usart_get_config_defaults(&conf_usart);
	
	conf_usart.baudrate = 115200;
	conf_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	
	conf_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	conf_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	conf_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	conf_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	
	stdio_serial_init(&console_instance, EDBG_CDC_MODULE, &conf_usart);
	
	usart_enable(&console_instance);
}

/**
* \brief ADC Differential mode.
*
* This function configures the ADC as differential mode 
* and converts the differential voltage applied on positive
* input and negative input. Prints the differential voltage
* on serial console and disable the ADC.
*/

void adc_differential(void)
{
	int16_t raw_result_signed;
	
	configure_adc_differential();
	
	raw_result = adc_start_read_result();
	
	raw_result_signed = (int16_t)raw_result;	
	result = ((float)raw_result_signed * (float)ADC_REFERENCE_INTVCC1_VALUE)/(float)ADC_11BIT_FULL_SCALE_VALUE;
	
	printf("\nDifferential Voltage on ADC Input = %fV", result);
	
	adc_disable(&adc_instance);
}


/**
* \brief ADC Hardware Averaging mode.
*
* This function configures the ADC as hardware averaging mode 
* and converts the ADC input applied on positive input then 
* prints the ADC input voltage on serial console after hardware 
* averaging done and disable the ADC
*/

void adc_hardware_averaging(void)
{
	configure_adc_averaging();
	
	raw_result = adc_start_read_result();
	
	result = ((float)raw_result * (float)ADC_REFERENCE_INTVCC1_VALUE)/(float)ADC_12BIT_FULL_SCALE_VALUE;
	
	printf("\nADC Input Voltage with Averaging = %fV", result);
	
	adc_disable(&adc_instance);
}


/**
* \brief ADC Oversampling mode.
*
* This function configures the ADC with normal mode and oversampling feature
* and converts the ADC input applied on positive input. Prints the ADC input 
* voltage for both mode on serial console to recognize the improvement.
*/

void adc_oversampling(void)
{
	configure_adc();
	
	raw_result = adc_start_read_result();
	
	result = ((float)raw_result * (float)ADC_REFERENCE_INTVCC1_VALUE)/(float)ADC_12BIT_FULL_SCALE_VALUE;
	
	printf("\nADC Input Voltage before Oversampling = %fV", result);
	
	adc_disable(&adc_instance);
	
	configure_adc_sampling();
	
	raw_result = adc_start_read_result();
	
	result = ((float)raw_result * (float)ADC_REFERENCE_INTVCC1_VALUE)/(float)ADC_16BIT_FULL_SCALE_VALUE;
	
	printf("\nADC Input Voltage after Oversampling = %fV", result);
	
	adc_disable(&adc_instance);
}

/**
* \brief ADC Window Monitor mode.
*
* This function configures the ADC as window monitor mode and predefine the
* voltage level for monitoring. If the ADC input exceeds the voltage level the
* LED0 will be in ON state in SAMD21 Xplained Pro.
*/

void adc_window_monitor(void)
{
	configure_adc_window_monitor();
	
	adc_start_conversion(&adc_instance);
	while((adc_get_status(&adc_instance) & ADC_STATUS_RESULT_READY) != 1);
	
	status = adc_get_status(&adc_instance);
	
	adc_read(&adc_instance, &raw_result);
	
	if (status & ADC_STATUS_WINDOW){
		port_pin_set_output_level(LED0_PIN, LOW);
		printf("\nLED0 is ON");
	}
	else{
		port_pin_set_output_level(LED0_PIN, HIGH);
		printf("\nLED0 is OFF");
	}
}

/**
* \brief ADC Calibration mode.
*
* This function enable/disable the ADC calibration based
* on the user input from serial console.

* For offset calibration user has to connect GND(0V) on ADC input.

* For Gain Calibration user has to connect 1.55V on ADC input.

* Serial console prints the ADC input voltage after calibration done.
*/

void adc_calibration(void)	
{
	uint8_t key;
	
	printf("\x0C\n-- Start of ADC Calibration Example --\n");

	configure_adc();
	
	printf("Commands:\n");
	printf("- key 'c' to enable correction\n");
	printf("- key 'd' to disable correction\n");
		
	key = getchar();
	
	delay_ms(500);
	
	if (key == 'c') {
		adc_correction_start();				
	}
	if (key == 'd') {
		adc_correction_stop();	
	}
	
	raw_result = adc_start_read_result();
	
	result = ((float)raw_result * (float)ADC_REFERENCE_INTVCC1_VALUE)/(float)ADC_12BIT_FULL_SCALE_VALUE;
	
	printf("\nADC Input voltage is %fV", result);	
	
	printf("\n-- End of ADC Calibration Example --\n");

}

/**
* \brief ADC internal TemperatureSensor mode.
*
* This function enables the internal temperature sensor for ADC input and displays 
* the current room temperature in serial console after the calculation done using ADC result.
*/

void adc_temp_sensor(void)
{
	float temp;
	
	system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE);
	
	configure_adc_temp();
	
	load_calibration_data();
	
	raw_result = adc_start_read_result();
	
	temp = calculate_temperature(raw_result);
	
	printf("\nThe current temperature is = %f degree Celsius", temp);
	
}
