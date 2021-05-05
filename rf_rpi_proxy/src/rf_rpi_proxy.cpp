#include <Arduino.h>

// define board pins according to arduino pins schema
#define LED_BUILTIN      (25ul)   // User controlled LED1 Arduino: D25 / Board: D25_RX_LED/PB03
#define LED_BUILTIN_1    LED_BUILTIN
#define LED_BULITIN_2    (26ul)   // User controlled LED2 Arduino: D26 / Board: D26_TX_LED/PA27
#define PIN_RADIO_EN     (6ul)	 // Pin to control radio power MOSFET / Arduino: D6 / Board: D6_NRF24_EN/PA20

// system behavior
#define SERIAL_BAUDS     (115200ul)	// UART Speed for debugging header (Arduino: Serial object)
#define RPI_SERIAL_BAUDS (460800ul) // UART Speed for communication with Raspberry Pi (Arduino: Serial1 object)

uint8_t is_debug_initialized = 0;

// headers for NRF24 library
#include <SPI.h>
#include "RF24.h"

// communication
#define R_PIN_CE         (9ul)		  // Arduino: D9 / Board: D9_NRF_CE/PA07
#define R_PIN_CSN        (38ul)		  // Arduino: D38 / Board: D38_SPI_SSCSN/PA13
#define R_ADDR_RX        (0x696900)	  // Address on which we must listen to get messages from remote nodes (on remote nodes this is TX address)
#define R_ADDR_WIDTH     (3ul)		  // We can limit frame by using shorter address
#define R_CHANNEL        (99ul)		  // Channel number from 2.4GHz range
#define R_POWER_LVL      RF24_PA_MAX  // Output power during sending
#define R_DATA_RATE      RF24_250KBPS // Data rate used over the air
#define R_MSG_DELAY      (5ul)		  // Check description of setRetries function for meaning of this value
#define R_MSG_RETRIES    (10ul)	      // Check description of setRetries function for meaning of this value

RF24 radio(R_PIN_CE, R_PIN_CSN);

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
typedef enum {
    /** (0) System reset */
    RBT_SYSTEM = 0,
    /** (1) External reset (reset pin) */
    RBT_EXTERNAL_RESET,
    /** (2) BOD 3.3V */
    RBT_BOD3V3,
    /** (3) BOD 1.2V */
    RBT_BOD1V2,
    /** (4) Normaln power on reset - standard */
    RBT_STARTUP,
    /** (6) WDT */
    RBT_WDT,
    RBT_UNKNOWN = 99
} startup_code_ref;

//
// debugging 
template <typename... T>
void debug_me(const char *str, T... args) {
#ifdef DEBUG_ENABLED
	if ( is_debug_initialized == 1 ) {
		int len = snprintf(NULL, 0, str, args...);
		if (len) {
			char buff[len] = {0};
			sprintf(buff, str, args...);
			Serial.print(buff);
		};
		Serial.println();
	};
#endif
}

void enable_debug_mode() {
#ifdef DEBUG_ENABLED
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;	
    Serial.begin(SERIAL_BAUDS);
	debug_me("----------------------------");
	debug_me("Debug enabled");
	is_debug_initialized = 1;
#endif
}

void disable_debug_mode() {
#ifdef DEBUG_ENABLED
	Serial.end();
	is_debug_initialized = 0;
#endif
}
// END debugging

startup_code_ref get_last_reset_cause() {
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
// basic function to shorthand (wrap) enabling and disabling devices
void enable_equipment(const uint8_t pinNumber, const PinStatus state = HIGH, const PinMode mode = OUTPUT) {
	debug_me("Enabling eq pin:%i state:%i mode:%i", pinNumber, state, mode);
	pinMode(pinNumber, mode);
	digitalWrite(pinNumber, state);
}

void disable_equipment(const uint8_t pinNumber, const PinStatus state = LOW, const PinMode mode = INPUT_PULLUP) {
	debug_me("Disabling eq pin:%i state:%i mode:%i", pinNumber, state, mode);
	digitalWrite(pinNumber, state);
	pinMode(pinNumber, mode);
}

// making LED blinking a bit easier
void say_hello_world(const int cycles_count = 2, const int delay_ms = 300, const int led_port = LED_BUILTIN) {
	for (int i = 0; i < cycles_count; i++)
	{
		enable_equipment(led_port, LOW);
		delay(delay_ms);
		disable_equipment(led_port, HIGH);
		if (i + 1 < cycles_count)
		{
			delay(delay_ms);
		};
	};
}


// configure radio
void init_radio() {
	enable_equipment(PIN_RADIO_EN, RADIO_EN_PIN_STATE, OUTPUT);
	delay(100);
	bool radio_being_status = radio.begin();
	if (!radio_being_status)
	{ // Radio failed to init. Give some time and perform reboot
		debug_me("Radio problem");
		say_hello_world(100, 200, LED_BUILTIN_1);
		NVIC_SystemReset();
	};

	radio.enableDynamicPayloads();

	radio.setAutoAck(true);
	radio.setChannel(R_CHANNEL);
	radio.setAddressWidth(R_ADDR_WIDTH);
	radio.setPALevel(R_POWER_LVL);
	radio.setDataRate(R_DATA_RATE);
	radio.setRetries(R_MSG_DELAY, R_MSG_RETRIES);

	radio.openReadingPipe(1, R_ADDR_RX);

#ifdef DEBUG_ENABLED
	radio.printDetails();
	radio.printPrettyDetails();
#endif

	radio.startListening();
}
// radio functions


void loop() {
	debug_me("Start listening for new messages");
	radio.startListening();
	char msg[33] = {'\0'}; // max possible message size is 32bytes for nrf24l01

	while (!radio.available())
	{
		// waiting for new message
		// TODO: in next version we should use interrupt from NRF to handle newly arrived messages
	};

	// process new message
	if (radio.available())
	{
		uint8_t msg_len = radio.getDynamicPayloadSize();
		radio.read(&msg, msg_len);
		Serial1.println(msg);
#ifdef DEBUG_ENABLED
		debug_me(msg);
		say_hello_world(1, 10, LED_BULITIN_2);
#endif
	};
}

void setup(void) {

	enable_debug_mode();
	debug_me("System start");
	say_hello_world(3, 200);

	startup_code_ref startup_code = get_last_reset_cause();

	Serial1.begin(RPI_SERIAL_BAUDS);
	// say hi to Raspberry Pi
	Serial1.println("X: Welcome");
	Serial1.print("X: Last reboot code: ");
	Serial1.println(startup_code);

	init_radio();
}