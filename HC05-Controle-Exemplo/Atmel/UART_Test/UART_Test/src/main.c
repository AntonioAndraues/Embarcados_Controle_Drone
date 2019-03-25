/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */



/* Ligar pinos //bluetooth: RX - PB1  TX - PB0 // Mioware: sig - PB2 */
/* Ligar botoes //PA4 -> Botão vermelho , PA3 -> Botão verde , PA19 -> On/Off Botão. */


#include <asf.h>
#include <string.h>
#define LM35DZ_AFEC         AFEC0
#define LM35DZ_AFEC_ID      ID_AFEC0
#define LM35DZ_AFEC_CH      AFEC_CHANNEL_5 // Pin PB2
#define LM35DZ_AFEC_CH_IR   AFEC_INTERRUPT_EOC_5


// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL


#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

volatile long g_systimer = 0;
/** The conversion data is done flag */
volatile bool is_conversion_done = false;
/** The conversion data value */
volatile uint32_t g_ul_value = 0;

static void configure_console(void)
{
	usart_serial_options_t config;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


void SysTick_Handler() {
	g_systimer++;
}

/*
void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}
*/
void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	 // RX - PB0  TX - PB1 
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);	
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEDroneG", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN1234", 1000);
	usart_log("hc05_server_init", buffer_rx);
}
static void afec_temp_sensor_end_conversion(void)
{
	g_ul_value = afec_channel_get_value(LM35DZ_AFEC, LM35DZ_AFEC_CH);
	is_conversion_done = true;
}
void lm35dz_init(void) {
	pmc_enable_periph_clk(LM35DZ_AFEC_ID);
	afec_enable(LM35DZ_AFEC);
	struct afec_config afec_cfg;
	afec_get_config_defaults(&afec_cfg);
	afec_init(LM35DZ_AFEC, &afec_cfg);
	afec_set_trigger(LM35DZ_AFEC, AFEC_TRIG_SW);

	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(LM35DZ_AFEC, LM35DZ_AFEC_CH, &afec_ch_cfg);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_5, 0x200); // internal ADC offset is 0x200, it should cancel it and shift to 0
	afec_set_callback(LM35DZ_AFEC, LM35DZ_AFEC_CH_IR, afec_temp_sensor_end_conversion, 1);
	afec_channel_enable(LM35DZ_AFEC, LM35DZ_AFEC_CH);
	NVIC_SetPriority(AFEC0_IRQn, 10);
	
}
void lm35dz_enable_interrupt(void) {
	afec_enable_interrupt(LM35DZ_AFEC, LM35DZ_AFEC_CH_IR);
	NVIC_EnableIRQ(AFEC0_IRQn);
}
void lm35dz_convert(void) {
	if(!is_conversion_done) {
		afec_start_software_conversion(AFEC0);
	}
}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	lm35dz_init();
	
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	configure_console();
	lm35dz_enable_interrupt();
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	char button1 = '0';
	char button2 = '3';
	char button3 = '5';
	char muscle = '6';
	char eof = 'X';
	char buffer[1024];
	
	while(1) {
		if(pio_get(PIOA, PIO_INPUT, PIO_PA4) == 0) {
			button1 = '1';
		} else {
			button1 = '0';
		}
		if(pio_get(PIOA, PIO_INPUT, PIO_PA3) == 0) {
			button2 = '2';
		} else {
			button2 = '3';
		}
		if(pio_get(PIOA, PIO_INPUT, PIO_PA19) == 0) {
			button3 = '4';
			} else {
			button3 = '5';
		}
		if(g_ul_value > 2000) {
			muscle = '6';
		} else {
			muscle = '7';
		}
		//if(pio_get(PIOA))
		while(!usart_is_tx_ready(UART_COMM));
		usart_write(UART_COMM, button1);
		while(!usart_is_tx_ready(UART_COMM));
		usart_write(UART_COMM, button2);
		while(!usart_is_tx_ready(UART_COMM));
		usart_write(UART_COMM, button3);
		while(!usart_is_tx_ready(UART_COMM));
		usart_write(UART_COMM, muscle);
		while(!usart_is_tx_ready(UART_COMM));
		usart_write(UART_COMM, eof);
		delay_s(1);
		lm35dz_convert();
		//delay_ms(1);
		if(is_conversion_done){
			uint32_t g_voltage = g_ul_value * 3300 / 4096; // in mv
			printf("VoltagemMusculo : %d\r\n", g_ul_value);
			is_conversion_done = false;

		}
	}
}
