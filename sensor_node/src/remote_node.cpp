#include <Arduino.h>
#include <wiring_private.h>
extern "C" void __libc_init_array(void);

#define VARIANT_MCK (8000000ul) // this is core speed we will work at - 8MHz

// rtc requirement
#include "RTCZero.h"

RTCZero rtc;

// Change these values to set the current initial date
const byte seconds = 0;
const byte minutes = 00;
const byte hours = 0;
const byte day = 1;
const byte month = 1;
const unsigned int year = 21;

#include "global_counter.h"

GlobalCounter gcnt;

// dictionaries
/*
Enum with definition of possible system restart. User should be aware of what kind of reboot occurred.
1 - System reset
2 - External reset requested (some one pushedreset button)
3 - BOD 3.3V
4 - BOD 1.2V
5 - Normal power on reset - system is starting by its standard procedure
6 - WDT Reset
99 - Unknown / very very sad face :(
*/
typedef enum
{
	RBT_SYSTEM = 0,		/** (0) System reset */
	RBT_EXTERNAL_RESET, /** (1) External reset (reset pin) */
	RBT_BOD3V3,			/** (2) BOD 3.3V */
	RBT_BOD1V2,			/** (3) BOD 1.2V */
	RBT_STARTUP,		/** (4) Normaln power on reset - standard */
	RBT_WDT,			/** (6) WDT */
	RBT_UNKNOWN = 99
} startup_code_ref;

/*
Enum with definition of measurments code
*/
typedef enum
{
	MSGC_TEMPERATURE = 0x1,
	MSGC_HUMIDITY = 0x2,
	MSGC_TEMP_SENSOR_ERROR = 0x4,
	MSGC_BATTERY_VOLTAGE = 0x3,
	MSGC_ALL_MESSAGES_SUCCESS = 0x5,
	MSGC_ALL_SENDING_TRIALS = 0x6,
	MSGC_AVG_CYCLE_LENGTH = 0x7,
	MSGC_DELIVERY_RATIO = 0x8,
	MSGC_ID_1 = 0xB,
	MSGC_ID_2 = 0xC,
	MSGC_ID_3 = 0xD,
	MSGC_ID_4 = 0xE,
	MSGC_STARTUP_CODE = 0xF
} message_content_ref;

// define board pins according to arduino pins schema & board design
#define LED_BUILTIN				PIN_LED_RXL // Arduino: D25 / SAM D21: PB03
#define LED_BUILTIN_1 			PIN_LED_RXL
#define LED_BUILTIN_2 			PIN_LED_TXL // Arduino: D26 / SAM D21: PA27
#define PIN_RADIO_EN 			(6ul) // Arduino: D6 / SAM D21: PA20
#define PIN_ONEWIRE_DQ 			(8ul) // Arduino: D8 / SAM D21: PA06
#define PIN_BOOST_EN 			(7ul) // Arduino: D7 / SAM D21: PA21
#define PIN_SENSORS_EN 			(4ul) // Arduino: D4 / SAM D21: PA08
#define PIN_VBATT_EN 			(3ul) // Arduino: D3 / SAM D21: PA09
#define PIN_BATTERY_VOLTAGE 	(A5)  // Arduino: A5 / SAM D21: PB02

// system behavior
#define GCLK_UNUSED_FOR_EVER 	GCLK_CLKCTRL_GEN_GCLK7_Val // which GCLK will be never used - we can connect all devices to this one
#define GCLK_USED_FOR_RTC 		GCLK_CLKCTRL_GEN_GCLK2_Val	// clock used for RTC - required to wake up. always must be enabled
#define SERIAL_BAUDS			(38400ul) // connection speed for debug purposes
#ifndef GLOBAL_SLEEP_SECONDS
	#define GLOBAL_SLEEP_SECONDS (8ul)
#endif
uint_fast8_t is_debug_initialized = 0;
uint_fast8_t startup_reset_cause = 0;
uint_fast32_t mcu_unique_id[4]; // in this place we will keep unique identifier of our board
uint8_t my_node_id = 0;			// ID of this board - calculated from mcu_unique_id
#ifndef BOARD_VCC
	#define BOARD_VCC 			(3.017F) // required for ADC measurments (VCC is source for voltage divider)
#endif
#ifndef STATS_SENDER_CYCLES // after how many cycles performance statistics should be sent
	#define STATS_SENDER_CYCLES (100ul)
#endif

// including stuff required for communication over NRF24 module
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// NRF24 and communication layer settings
#define R_PIN_CE 				(9ul)
#define R_PIN_CSN				(38ul)
#define R_ADDR_TX 				(0x696900)
#define R_ADDR_RX_COMMON 		(0xAABB00) // common part of each node. To this constant we are adding custom node id value
#define R_ADDR_WIDTH 			(3ul)
#define R_CHANNEL 				(99ul)
#ifndef R_POWER_LVL
	#define R_POWER_LVL 		RF24_PA_MAX
#endif
#define R_DATA_RATE 			RF24_250KBPS
#define R_RETRIES_DELAY 		(8ul)  // see documentation for setRetires function
#define R_RETRIES_COUNT 		(3ul) // see documentation for setRetires function
#define MSG_SEP 				""
#define R_MSG_MAX_REPEAT 		(3ul) // how many time try of sending message should be taken - additional wrapping of write function
RF24 radio(R_PIN_CE, R_PIN_CSN);

// including stuff required for Sensirion SHT31 sensor
#include "Adafruit_SHT31.h"
#define SHT31_ENABLE_HEATER 	(false)

Adafruit_SHT31 sht31 = Adafruit_SHT31();

// END OF DECLARATIONS / real magic begins :)

// clocks configuration functions
// OSC setup and main clock setup - configure main system clock at 8MHz speed
void init_OSC8M()
{														  
	SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val; // no prescaller / it should be 8MHz
	SYSCTRL->OSC8M.bit.ONDEMAND = 0;
	SYSCTRL->OSC8M.bit.RUNSTDBY = 0;
	SYSCTRL->OSC8M.bit.ENABLE = 1;
	while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC8MRDY) == 0);
}

void init_GCLK_MAIN()
{
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(GCLK_CLKCTRL_GEN_GCLK0); // Generic Clock Generator 0

	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	/* Write Generic Clock Generator 0 configuration */
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(GCLK_CLKCTRL_GEN_GCLK0) | // Generic Clock Generator 0
						GCLK_GENCTRL_SRC_OSC8M |				  // source for main clock generator is OSC8M
						GCLK_GENCTRL_IDC |						  // Set 50/50 duty cycle
						GCLK_GENCTRL_GENEN;

	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void disable_general_clock(uint8_t gclk)
{
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(gclk);
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

// debuging function
template <typename... T>
void debug_me(const char *str, T... args)
{
#ifdef DEBUG_ENABLED
	if (is_debug_initialized == 1)
	{
		int len = snprintf(NULL, 0, str, args...);
		if (len)
		{
			char buffm[len] = {0};
			sprintf(buffm, str, args...);
			Serial.print(buffm);
		};
		Serial.println();
		delay(8);
	};
#endif
}

void enable_debug_mode()
{
#ifdef DEBUG_ENABLED
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
	Serial.begin(SERIAL_BAUDS);
	debug_me("----------------------------");
	debug_me("Debug enabled");
	is_debug_initialized = 1;
#endif
}

void disable_debug_mode()
{
#ifdef DEBUG_ENABLED
	Serial.end();
	is_debug_initialized = 0;
#endif
}

void get_unique_id()
{
	volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
	mcu_unique_id[0] = *ptr1;
	volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
	mcu_unique_id[1] = *ptr;
	ptr++;
	mcu_unique_id[2] = *ptr;
	ptr++;
	mcu_unique_id[3] = *ptr;

	debug_me("MCU unique ID: 0x%8x%8x%8x%8x", mcu_unique_id[0], mcu_unique_id[1], mcu_unique_id[2], mcu_unique_id[3]);

	my_node_id = mcu_unique_id[0] % 64 + mcu_unique_id[1] % 64 + mcu_unique_id[2] % 64 + mcu_unique_id[3] % 64;

	debug_me("My NODE_ID: %i / %x ", my_node_id, my_node_id);
}

startup_code_ref get_last_reset_cause()
{
	if (PM->RCAUSE.bit.SYST == 1)
	{
		debug_me("Reboot: Reset requested by system");
		return RBT_SYSTEM;
	};
	if (PM->RCAUSE.reg & PM_RCAUSE_WDT)
	{
		debug_me("Reboot: WDT");
		return RBT_WDT;
	};
	if (PM->RCAUSE.bit.EXT == 1)
	{
		debug_me("Reboot: External reset requested");
		return RBT_EXTERNAL_RESET;
	};
	if (PM->RCAUSE.bit.BOD33 == 1)
	{
		debug_me("Reboot: Reset brown out 3.3V");
		return RBT_BOD3V3;
	};
	if (PM->RCAUSE.bit.BOD12 == 1)
	{
		debug_me("Reboot: Reset brown out 1.2v");
		return RBT_BOD1V2;
	};
	if (PM->RCAUSE.bit.POR == 1)
	{
		debug_me("Reboot: Normal power on reset");
		return RBT_STARTUP;
	};
	debug_me("Reboot: unknown reason");
	return RBT_UNKNOWN;
}

//
// basic function to shorthand (wrap) enabling and disabling available equipment
void enable_equipment(const uint8_t pinNumber, const PinStatus state = HIGH, const PinMode mode = OUTPUT)
{
	debug_me("Enabling eq pin:%i state:%i mode:%i", pinNumber, state, mode);
	pinMode(pinNumber, mode);
	digitalWrite(pinNumber, state);
}

void disable_equipment(const uint8_t pinNumber, const PinStatus state = LOW, const PinMode mode = INPUT_PULLUP)
{
	debug_me("Disabling eq pin:%i state:%i mode:%i", pinNumber, state, mode);
	digitalWrite(pinNumber, state);
	pinMode(pinNumber, mode);
}

// making led blinking a bit easier
void say_hello_world(const int how_many_cycles = 2, const int delay_ms = 100, const int led_port = LED_BUILTIN)
{
	for (int i = 0; i < how_many_cycles; i++)
	{
		enable_equipment(led_port, LOW);
		delay(delay_ms);
		disable_equipment(led_port, HIGH);
		if (i + 1 < how_many_cycles)
		{
			delay(delay_ms);
		};
	};
}

// function responsible for generating messages to send
void prepare_msg(const int measure_id, const float measure_value, char *msg, const char *sprint_format = "%.2f")
{
	char buff[33] = {'\0'};
	strcpy(msg, "");

	snprintf(buff, sizeof(buff), "%x%s%x%s", my_node_id, MSG_SEP, measure_id, MSG_SEP);
	strcat(msg, buff);

	// third part: measure itself tadadam!
	snprintf(buff, sizeof(buff), sprint_format, measure_value);
	strcat(msg, buff);
	strcat(msg, MSG_SEP);

	// fourth part: message counter
	snprintf(buff, sizeof(buff), "%i", gcnt.get_cycles());
	strcat(msg, buff);

	debug_me("msg to send: %s", msg);
}

// whole part related with delivering - prevoisly generated message - over the air
bool send_msg(char *msg)
{
	bool msg_sending_result = false;
	uint_fast8_t sending_trial = 1;
	radio.stopListening();
	debug_me("Start sending");

	while (sending_trial <= R_MSG_MAX_REPEAT)
	{
		debug_me("Sending trial: %i", sending_trial);
		gcnt.increase_msg_trials();
		msg_sending_result = radio.write(msg, strlen(msg));
		if (msg_sending_result)
		{
			debug_me("Message sent with success!");
			gcnt.increase_msg_success();
			say_hello_world(1, 2, LED_BUILTIN_2);
			return true;
		};
		debug_me("Message sending failed");
		say_hello_world(1, 2, LED_BUILTIN_1);
		gcnt.increase_msg_retransmissions();
		if (sending_trial < R_MSG_MAX_REPEAT)
		{
			delay((sending_trial + 1) * R_RETRIES_DELAY * 4);
		};
		sending_trial++;
	};
	return false;
}

// in this place, whole unique id of SAM D21 is sent
void prepare_and_send_unique_id()
{
#ifdef DEBUG_ENABLED
	for (uint_fast8_t cnt = 0; cnt < 4; cnt++)
	{
		char buff[33] = {};
		char msg[33] = {};
		strcpy(msg, "");
		snprintf(buff, sizeof(buff), "%x%s%x%s", my_node_id, MSG_SEP, MSGC_ID_1 + cnt, MSG_SEP);
		strcat(msg, buff);
		snprintf(buff, sizeof(buff), "0x%x", mcu_unique_id[cnt]);
		strcat(msg, buff);
		debug_me(msg);
		send_msg(&msg[0]);
	};
#endif
}

// base radio configuration
void radio_init()
{
	bool radio_begin_status = radio.begin();
	uint_fast32_t rx_address = R_ADDR_RX_COMMON + my_node_id;
	if (!radio_begin_status)
	{ // in case of radio failure
		debug_me("radio hardware is not responding!!");
		say_hello_world(100, 200, LED_BUILTIN_1);
		// perform system reset - maybe this will help somehow?
		NVIC_SystemReset();
	};
	debug_me("Radio initiated with success");
	debug_me("NRF RX address: %x", rx_address);
	debug_me("NRF output power: %i", R_POWER_LVL);
	radio.enableDynamicPayloads();

	radio.setAutoAck(true);
	radio.setChannel(R_CHANNEL);
	radio.setAddressWidth(R_ADDR_WIDTH);
	radio.setPALevel(R_POWER_LVL);
	radio.setDataRate(R_DATA_RATE);
	radio.setRetries(R_RETRIES_DELAY, R_RETRIES_COUNT);
	radio.openReadingPipe(1, rx_address);
	radio.openWritingPipe(R_ADDR_TX);
	radio.stopListening();

#ifdef DEBUG_ENABLED
	radio.printDetails();
	radio.printPrettyDetails();
#endif
}

void radio_disable()
{
	radio.powerDown();
	// before removing VCC we should switch all communication pins into input to not fry NRF
	pinMode(22, INPUT_PULLUP);
	pinMode(38, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);
	pinMode(23, INPUT_PULLUP);
	pinMode(24, INPUT_PULLUP);
}

// END radio functions

// ADC functions
// init_adc based on: https://blog.thea.codes/reading-analog-values-with-the-samd-adc/ - thank you :)
void adc_init()
{
	/* Enable the APB clock for the ADC. */
	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

	ADC->CTRLA.bit.ENABLE = 0;
	while (GCLK->STATUS.bit.SYNCBUSY)
	{
	};
	/* Enable GCLK0 for the ADC */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
						GCLK_CLKCTRL_GEN_GCLK0 |
						GCLK_CLKCTRL_ID_ADC;

	uint32_t bias = (*((uint32_t *)ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
	uint32_t linearity = (*((uint32_t *)ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
	linearity |= ((*((uint32_t *)ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

	/* Wait for bus synchronization. */
	while (GCLK->STATUS.bit.SYNCBUSY)
	{
	};

	/* Write the calibration data. */
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
	/* Wait for bus synchronization. */
	while (ADC->STATUS.bit.SYNCBUSY)
	{
	};

	/* Voltage reference: 1/1.48 VDDANA -> for this board it is ~3.065V/1.48 = 2.07V */
	ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0;

	ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(14);

	/* Set the clock prescaler to 512, which will run the ADC at
    8 Mhz / 512 = 31.25 kHz.
    Set the resolution to 16bit. -> with averaging its effective 12bits
	For multisampling to work correctly the RESSEL part
    of CTRLB must be set to 16-bit, even if you're doing
    12-bit measurements. See section 33.6.7  in the
    datasheet.
    */
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV512_Val;
	ADC->CTRLB.reg |= ADC_CTRLB_RESSEL_16BIT;

	// Configure multisampling and averaging.
	// Note that ADJRES must be set according to table
	// 33-3 in the datasheet.
	ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_64 | ADC_AVGCTRL_ADJRES(4);

	/* Configure the input parameters.
    - GAIN
    - MUXNEG_GND means that the ADC should compare the input value to GND.
    - MUXPOST_PINX
    */
	ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
	ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
	ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN10_Val;

	/* Set A5 / PB02 as an input pin. */
	PORT->Group[PORTB].DIRCLR.reg = PORT_PB02;
	/* Enable the peripheral multiplexer for PA05. */
	PORT->Group[PORTB].PINCFG[2].bit.PMUXEN = 1;

	/* Set A5/PB02 to function B which is analog input. */
	PORT->Group[PORTB].PMUX[1].reg = PORT_PMUX_PMUXE_B;

	/* Wait for bus synchronization. */
	while (ADC->STATUS.bit.SYNCBUSY);
}

void adc_disable()
{
	ADC->CTRLA.bit.ENABLE = 0;
	while (ADC->STATUS.bit.SYNCBUSY);
}

uint_fast32_t get_adc_value()
{
	/* Enable the ADC. */
	/* Wait for bus synchronization. */
	ADC->CTRLA.bit.ENABLE = 1;

	while (ADC->STATUS.bit.SYNCBUSY);

	// first measure should be throw away
	/* Start the ADC using a software trigger. */
	ADC->SWTRIG.bit.START = true;

	/* Wait for the result ready flag to be set. */
	while (ADC->INTFLAG.bit.RESRDY == 0);

	/* Clear the flag. */
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

	/* Start the ADC using a software trigger. */
	ADC->SWTRIG.bit.START = true;

	/* Wait for the result ready flag to be set. */
	while (ADC->INTFLAG.bit.RESRDY == 0);

	/* Clear the flag. */
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

	/* Read the value. */
	uint_fast32_t result = ADC->RESULT.reg;
	return result;
}

float get_voltage(uint_fast32_t adc_reading)
{
	return 2 * adc_reading * ((float)BOARD_VCC / 1.48F) / 4096;
}

// END ADC functions

// SHT31 init function
bool sht31_enable()
{
	bool sht31_sensor_failure = false;

	if (!sht31.begin(0x44))
	{
		debug_me("Couldn't find SHT31");
		sht31_sensor_failure = true;
	};

	sht31.heater(SHT31_ENABLE_HEATER);

	if (sht31.isHeaterEnabled())
	{
		debug_me("SHT31 Heater enabled");
	}
	else
	{
		debug_me("SHT31 Heater disabled");
	};

	return sht31_sensor_failure;
}

void sht31_disable()
{
	// all measurments are taken - now we can switch off all sensors
	// disable SCL and SDA ports
	pinMode(20, INPUT_PULLUP);
	pinMode(21, INPUT_PULLUP);
}

// lets make some juice:)
void run_cycle()
{
	uint_fast32_t epoch_start, epoch_end, epoch_diff = 0;
	float battery_voltage = 0;
	float sht31_temperature = 0;
	float sht31_humidity = 0;
	bool sht31_sensor_failure = false;
	char msg[32] = {};

	epoch_start = millis();
#ifdef DEBUG_ENABLED
	debug_me("Cycle start");
	debug_me("Cycle: %i", gcnt.get_cycles());
	debug_me("Cycle timestamp start %i", epoch_start);
#endif

	// power up all sensors we should interact with
	enable_equipment(PIN_SENSORS_EN, LOW);
	// get some time to stabilize voltage
	delay(5);
	sht31_sensor_failure = sht31_enable();

	sht31_temperature = sht31.readTemperature();
	sht31_humidity = sht31.readHumidity();

	if (!isnan(sht31_temperature))
	{ // check if 'is not a number'
		debug_me("SHT31 Temp *C = %f", sht31_temperature);
	}
	else
	{
		debug_me("SHT31 Failed to read temperature");
		sht31_sensor_failure = true;
	};

	if (!isnan(sht31_humidity))
	{ // check if 'is not a number'
		debug_me("SHT31 Hum. % = %f", sht31_humidity);
	}
	else
	{
		debug_me("SHT31 Failed to read humidity");
		sht31_sensor_failure = true;
	};

	sht31_disable();
	disable_equipment(PIN_SENSORS_EN, HIGH);

	// get battery voltage
	debug_me("Start ADC module");

#ifndef NO_BATTERY_MEASURE
	// enable battery circuit
	enable_equipment(PIN_VBATT_EN, LOW);
	// wait some time to stabilize voltage
	delay(5);
	adc_init();
	battery_voltage = get_voltage(get_adc_value());
	adc_disable();
	disable_equipment(PIN_VBATT_EN, HIGH);
	debug_me("Battery voltage: %f", battery_voltage);
	// END get battery voltage
#endif

	// radio stuff
	enable_equipment(PIN_RADIO_EN, LOW);
	delay(10);
	radio_init();

	if (sht31_sensor_failure)
	{
		prepare_msg(MSGC_TEMP_SENSOR_ERROR, sht31_sensor_failure, &msg[0], "%i");
		send_msg(&msg[0]);
	}
	else
	{
		prepare_msg(MSGC_TEMPERATURE, sht31_temperature, &msg[0]);
		send_msg(&msg[0]);

		prepare_msg(MSGC_HUMIDITY, sht31_humidity, &msg[0]);
		send_msg(&msg[0]);
	};

	prepare_msg(MSGC_BATTERY_VOLTAGE, battery_voltage, &msg[0], "%.3f");
	send_msg(&msg[0]);

	// send basic stats about this board and startup
	if (gcnt.get_cycles() % STATS_SENDER_CYCLES == 0 or gcnt.get_cycles() == 1)
	{

		// all messages sent with success
		prepare_msg(MSGC_ALL_MESSAGES_SUCCESS, gcnt.get_msg_success(), &msg[0], "%.f");
		send_msg(&msg[0]);

		// all msg sending trials
		prepare_msg(MSGC_ALL_SENDING_TRIALS, gcnt.get_msg_trials(), &msg[0], "%.f");
		send_msg(&msg[0]);

		// avg cycle length
		prepare_msg(MSGC_AVG_CYCLE_LENGTH, (gcnt.get_cycles_length() / (float)gcnt.get_cycles()), &msg[0], "%.f");
		send_msg(&msg[0]);

		// delivery ratio
		if (gcnt.get_msg_trials() > 0)
		{
			prepare_msg(MSGC_DELIVERY_RATIO, (gcnt.get_msg_success() / (float)gcnt.get_msg_trials() * 100), &msg[0], "%03.f");
			send_msg(&msg[0]);
		};

		prepare_msg(MSGC_STARTUP_CODE, startup_reset_cause, &msg[0], "%.f");
		send_msg(&msg[0]);

		prepare_and_send_unique_id();
	};

	radio_disable();
	disable_equipment(PIN_RADIO_EN, HIGH);

	epoch_end = millis();
	epoch_diff = epoch_end - epoch_start;
	gcnt.increase_cycles_length(epoch_diff);

#ifdef DEBUG_ENABLED
	debug_me("MCU unique ID: 0x%8x%8x%8x%8x", mcu_unique_id[0], mcu_unique_id[1], mcu_unique_id[2], mcu_unique_id[3]);
	debug_me("My NODE_ID: %i / %x ", my_node_id, my_node_id);
	debug_me("Stats: Cycle epoch time:%i; diff: %i", epoch_end, epoch_diff);
	debug_me("Stats: All messages sent: %i", gcnt.get_msg_success());
	debug_me("Stats: All messages sending trials: %i", gcnt.get_msg_trials());
	debug_me("Stats: All messages retransmission: %i", gcnt.get_msg_retransmissions());
	if (gcnt.get_msg_trials() > 0)
	{
		float tmp_delivery_ratio = gcnt.get_msg_success() / (float)gcnt.get_msg_trials();
		debug_me("Stats: Delivery ratio: %.2f", tmp_delivery_ratio);
	};
	debug_me("Stats: All cycles time length: %i", gcnt.get_cycles_length());
	float tmp_avg_cycle_length = gcnt.get_cycles_length() / (float)gcnt.get_cycles();
	debug_me("Stats: AVG cycle time in millis: %.2f", tmp_avg_cycle_length);
	debug_me("Cycle end");
	delay(20);
#endif
}

// SAM D21 additional operational functions & power saving functions
void power_up()
{
	PM->APBBMASK.reg |= PM_APBBMASK_PORT;

	// required for I2C
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3;

	// required for SPI
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;

	SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val; // scale CPU UP

	enable_debug_mode();
	debug_me("Powering up");
	gcnt.increase_cycles();
}

// disabling power managers for most of peripherals
void power_manager_control()
{
	// disabled on AHB
	// - PM_AHBMASK_USB
	// - PM_AHBMASK_DMAC
	// - PM_AHBMASK_DSU
	PM->AHBMASK.reg = PM_AHBMASK_NVMCTRL | PM_AHBMASK_HPB2 | PM_AHBMASK_HPB1 | PM_AHBMASK_HPB0;

	// disabled on APBA
	// - PM_APBAMASK_EIC
	// - PM_APBAMASK_WDT
	// - PM_APBAMASK_PAC0
	PM->APBAMASK.reg = PM_APBAMASK_RTC | PM_APBAMASK_GCLK | PM_APBAMASK_PM | PM_APBAMASK_SYSCTRL;

	// on APBB and APBC we can disable all things. We will power up required components
	PM->APBBMASK.reg = 0x0;
	PM->APBCMASK.reg = 0x0;
}

void pins_power_down()
{
	// switch all port to input mode
	PORT->Group[0].DIR.reg = 0x0;
	PORT->Group[1].DIR.reg = 0x0;

	// change configuration to default on all pins - based on documentation, this is the most power save scenario
	for (unsigned int pin_n = 0; pin_n < 32; pin_n++)
	{
		PORT->Group[PORTA].PINCFG[pin_n].reg = 0x0;
		PORT->Group[PORTB].PINCFG[pin_n].reg = 0x0;
	};
}

// disable BOD - voltage is controlled by MCP16252 so we don't need additional controller for now. It also takes some power during deep sleep
void disable_BOD()
{
	SYSCTRL->BOD33.bit.ENABLE = 0;
}

//
void dummy_interrupt()
{
}

void go_sleep()
{
	debug_me("Going sleep");

	debug_me("Power down");
	disable_debug_mode();

	// disabling BOD
	disable_BOD();

	SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_3_Val; // scale CPU Down

	// disabling unused oscillators
	SYSCTRL->XOSC32K.bit.ENABLE = 0;
	SYSCTRL->XOSC.bit.ENABLE = 0;
	SYSCTRL->OSC32K.bit.ENABLE = 0;

	pins_power_down();

	power_manager_control();

	// disabling all unused clock controllers - just leaving enabled clock 0 - as a source for system and clock controller for RTC
	for (uint8_t cnt = 1; cnt < GCLK_GEN_NUM_MSB; cnt++)
	{
		if (cnt == GCLK_USED_FOR_RTC)
		{
			continue;
		};
		disable_general_clock(cnt);
	};

	// set next wake up time
	rtc.setAlarmEpoch(rtc.getEpoch() + GLOBAL_SLEEP_SECONDS);

	// method of solving bug related with hanging MCU during sleep - beside, systick isn't required during sleep
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI(); // here is the real sleep
	// Enable systick interrupt
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

// handling hardware failure through interrupt
void HardFault_Handler(void)
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	while (1)
	{
	};
}

// initialize MCU
// entry point: Reset_Handler in cortex_handlers
void SystemInit(void)
{

	__libc_init_array();

	asm(".global _printf_float"); // enable floating numbers handling

	init_OSC8M();
	init_GCLK_MAIN();
	SystemCoreClock = VARIANT_MCK;

	disable_BOD();

	SysTick_Config(SystemCoreClock / 1000);

	enable_debug_mode();
	debug_me("System start");
	say_hello_world(3, 100);

	rtc.begin();
	rtc.setTime(hours, minutes, seconds);
	rtc.setDate(day, month, year);
	rtc.enableAlarm(rtc.MATCH_HHMMSS);
	rtc.attachInterrupt(dummy_interrupt);

	get_unique_id();

	startup_reset_cause = get_last_reset_cause();

	for (;;)
	{
		power_up();
		run_cycle();
		go_sleep();
	}
}

// stub main function
// after using such declaration, you must remember to cover setup() and loop() functions by yourself
// this instruction is overwriting main from Arduino's main.cpp
int main(void)
{
	return 0;
}