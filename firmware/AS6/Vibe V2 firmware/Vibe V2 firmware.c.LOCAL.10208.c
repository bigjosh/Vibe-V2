/*
 * Vibe_V2_firmware.c
 *
 * Created: 11/30/2014 12:25:41 PM
 *  Author: josh.com
 */ 


/*

 Note to gentle reader: 
 Before you judge me, consider that this repetitive, verbose, and long form style is thoughtful and intentional.
 The target product is encased in a silicone overmold, so there is no practical way to do a "battery pull" reset 
 on the circuit in case of a latch-up. There was also no good way to add a hardware reset button to the product.
 Therefore, this is a defensive style designed to keep running and self correct even in the face of randomly flipped
 bits that may occur in the variables, the registers, or even the program counter. Each time though the main event loops,
 every important register is re-initialized to a known good value just in case. This makes for ugly code, but it is not that wasteful
 of code space since you'd need to initialize these locations anyway - this code just moves those initializations into the loop 
 rather than running them a single time before entering it. This does waste cycles, but we have plenty of extra cycles in this simple
 application.
 
 -josh
 
*/
 
  
// CPU speed in Hz. Needed for timing functions.
// This is the default fuse setting and works fine for PWM frequencies up to about 10Khz (1% lowest duty), or 100KHz (10% lowest duty). 
// This suits the current speed settings, but a high clock might be needed for higher PWM frequencies

#define F_CPU 1000000						// Name used by delay.h 

#define CYCLES_PER_S F_CPU					// Better name

#define CYCLES_PER_MS (F_CPU/1000UL)		// More convenient unit


// Outputs

#define WHITE_LED_PORT PORTB
#define WHITE_LED_DDR DDRB
#define WHITE_LED_BIT 2

#define RED_LED_PORT PORTA
#define RED_LED_DDR DDRA
#define RED_LED_BIT 7


// Inputs


#define BATTERY_SENSE_PORT PORTA
#define BATTERY_SENSE_DDR DDRA
#define BATTERY_SENSE_BIT 3
#define BATTERY_INT 


#define BUTTON_PORT PORTB
#define BUTTON_PIN	PINB
#define BUTTON_DDR  DDRB
#define BUTTON_BIT	0
#define BUTTON_INT	PCINT8

// Is button currently pressed? Pin has a pullup connected to ground though button, so a down reads a 0 on the pin. 
// (Compiles down to a single SBIC instruction)
#define BUTTON_STATE_DOWN()	((BUTTON_PIN & _BV(BUTTON_BIT))==0)

// EOC is the end-of-charge (battery full) signal. It s Active LOW.
// It is connected to the STAT2 line from the battery controller
// Note we must be pull up this line
#define EOC_PORT PORTA
#define EOC_DDR	 DDRA
#define EOC_PIN PINA
#define EOC_BIT 0
#define EOC_INT PCINT0

// EOC pin is pulled LOW by battery charger to indicate End of Charge
// (Compiles down to a single SBIC instruction)

#define EOC_STATE_ACTIVE()		((EOC_PIN & _BV(EOC_BIT))==0)


// The JACK IO pin is connected to the hot pin of the
// charging jack though a 1K resistor. It could be used
// to detect a charger connection or for a half duplex 
// serial connection

#define JACK_PORT	PORTB
#define JACK_BIT	1
#define JACK_DDR	DDRB
#define JACK_PIN	PINB
#define JACK_INT	PCINT9

// Jack is normally pulled-up by internal resistor, but a slave device can momentarily 
// pull it low to request a connection or send a 0 bit

#define JACK_STATE_LOW() (!(JACK_PIN & _BV(JACK_BIT))) 

// CIP is the charge-in-progress (battery full) signal. It s Active LOW.
// It is connected to the STAT1 line from the battery controller
// Note we must be pull up this line
#define CIP_PORT PORTA
#define CIP_PIN  PINA
#define CIP_DDR	 DDRA
#define CIP_BIT 1
#define CIP_INT PCINT1

// CIP is pulled LOW by battery charger to indicate Charge In Progress
// (Compiles down to a single SBIC instruction)
#define CIP_STATE_ACTIVE()		((CIP_PIN & _BV(CIP_BIT))==0)

#define DEBOUNCE_TIME_MS 25					// How long to wait for a button press debounce

#define BUTTON_LONG_PRESS_MS 250			// How long to hold down button to be considered a long press rather than a push
											// Note that a long press is always preceded by a push

#define BUTTON_STUCK_TIMEOUT_MS	5000		// How long the button is held down for before we assume it is stuck and turn off to save battery

//#define DEBOUNCE_CYCLES (DEBOUNCE_TIME_MS *  CYCLES_PER_MS)

#define LOW_BATTERY_VOLTSx10	(32-03)		// Low battery cutoff, 3.2 volts for battery less the 0.3V diode drop

#define LOW_BATTERY_LED_ONTIME_MS (500)		// Show low battery by flashing red LED for 1/2 second


#define MS_PER_LOOP 2						// Number of milliseconds it takes to get around the main event loop. Alomost all this time is spent in readVcc()

#define LOOPS_PER_MS(x)	(x/MS_PER_LOOP)		// Compute the number of loops for the specified number of milliseconds


#define MAX_MOTOR_INCREASE_PER_MS   (655)		// Maximum ratio of full range that the motor can speed up in a an millisecond (0-65535, 655~=1%)
#define MAX_MOTOR_INCREASE_PER_LOOP (LOOPS_PER_MS(MAX_MOTOR_INCREASE_PER_MS))

#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

// Struct for holding speed steps

// TODO: Add pre-scaller for more dynamic range

typedef struct {
		uint16_t normailzedDuty;			// Duty cycle normalized to 4.2 volts Vcc. 0=off, 0xffff=full on at 4.2 volts power
		uint16_t top;						// Top value, which determines the PWM frequency where 	f = F_CPU/top	
} speedStepStruct;


// TODO: Move speed settings to PROGMEM	or EEprom
	
#define SPEED_STEP_COUNT 4
	
const speedStepStruct speedSteps[SPEED_STEP_COUNT] = {
	
	{                    0,    0 },			// step 0 = off
	{   0b0000110001000000, 8191 },
	{   0b0010110000000000, 8191 },
	{	0b0110000001000000, 8191 },
		
};

// MOTOR FUNCTIONS
// ===============
// Note that register values are hard coded rather than #defined because they 
// can not just be moved around.


uint16_t currentMotorSpeed = 0;		// Keep track of current motor speed so we can do slow start (0=off, 65535=full speed)
									// We only care about speed because we are dealing with physical inertia. This is the duty cycle and basically match/top
									
// Turn the motor completely off- disconnects from PWM generator
// You should call this immediately on reset to turn off the motor MOSFET in case R5 is missing or fails


void motorOff(void) {

	// First get the pin low so the MOSFET doesn't turn on.
	// Try to do this very early in the startup.
	
	PORTA &= ~_BV(5);	// Set pin output to low 	
	
	DDRA |= _BV(5);		// Set pin to output mode


	TCCR1A &= ~( _BV(COM1B1) | _BV(COM1B0) );			// Disconnect Timer1A outputs from pins. "Normal port operation, OC1B disconnected"
	
	currentMotorSpeed = 0;
	
}


// SetVFD motor PWM on pin 8/PA5/OC1B
// Note: also uses OCR1A for TOP function.
// Note: resets all used registers each time from scratch for safety from glitches

// match sets the duty cycle and should be between 0 and top. 0=completely off, top=full on. 
// top sets the frequency where PWM frequency = F_CPU/top. The minimum resolution allowed is 2-bit (top set to 0x0003).

// We do a slow start on the motor to avoid glitching the power line and lower acceleration forces and wear and tear on the physical motor. 


void setMotorPWM( uint16_t match , uint16_t top ) {
	
	uint16_t slowStartupAdjustedMatch;
		
	if (match==0) {			// Special case this because the PWM generator still generates a pulse at 0 duty cycle
							// "If the OCR1x is set equal to BOTTOM (0x0000) the output will be a narrow spike for each TOP+1 timer clock cycle."
		
		motorOff();
		
	} else {
		
		uint16_t requestedNewMotorSpeed = (match*65535UL)/top;		// (0=off, 65535=full on)
		
		if (requestedNewMotorSpeed > currentMotorSpeed )	{		// Are we speeding up?  (Break out steps here to avoid overlow and negative numbers)
							
			if (requestedNewMotorSpeed - currentMotorSpeed > MAX_MOTOR_INCREASE_PER_LOOP )	{ // Too much acceleration?
				
				unsigned long maxNewMotorSpeed = currentMotorSpeed + MAX_MOTOR_INCREASE_PER_LOOP;
												
				slowStartupAdjustedMatch = (maxNewMotorSpeed* top)/65535;			// Compute the match that corresponds to this motorspeed at the current top
				
			} else {
				
				slowStartupAdjustedMatch = match;			// jump straight to slower speed
								
			}
						
		} else {			// Same speed or slowing down

			slowStartupAdjustedMatch = match;			// jump straight to slower speed
			
		}
		
		
		// Set OC1B on Compare Match
		// Clear OC1B at BOTTOM (inverting mode)
	
		// Fast PWM, TOP= OCR1A, Update OCR1x at top
	
		// Clock select clk	I/O/1 (No prescaling)
		
		
		// Assign TOP first to make sure we don't miss the match
		
		OCR1A = top;							// Set TOP. Freq should be IOclk/OCR1A = 16Khz		
		OCR1B = slowStartupAdjustedMatch;		// Set match which sets duty cycle
		
		
		//			0bxx100000	COM1B		PWM Fast mode, Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
		//			0bxxxxxx11	WGM[11:10]	Fast PWM, TOP=OCR1A, Update at OCR TOP
	
		TCCR1A =	0b00100011;
	
		//			0b00011000	WGM[13:12]	Fast PWM TOP=OCR1A UPDATE=TOP, Compare output on pin
		//			0b00000001	CS			clk	I/O/1 (No prescaling)
	
		TCCR1B =	0b00011001;
	
//		TCNT1  = 0x00;		// Start counting at zero
		
				
		// "The actual OC1x value will only be visible on the port pin if the data direction for the port pin is set as output (DDR_OC1x)."
							
		DDRA |= _BV(5);		// Set pin to output mode
		
		currentMotorSpeed = ( (slowStartupAdjustedMatch * 65535UL) / top);
				
	}
	
}


// Returns the current Vcc voltage as a fixed point number with 1 implied decimal places, i.e.
// 50 = 5 volts, 25 = 2.5 volts,  19 = 1.9 volts
//
// On each reading we: enable the ADC, take the measurement, and then disable the ADC for power savings.
// This takes >1ms becuase the internal reference voltage must stabilize each time the ADC is enabled.
// For faster readings, you could initialize once, and then take multiple fast readings, just make sure to
// disable the ADC before going to sleep so you don't waste power. 

uint8_t readVccVoltage(void) {
	
	// Select ADC inputs
	// bit    76543210 
	// REFS = 00       = Vcc used as Vref
	// MUX  =   100001 = Single ended, 1.1V (Internal Ref) as Vin
	
	ADMUX = 0b00100001;
	
	/*
	By default, the successive approximation circuitry requires an input clock frequency between 50
	kHz and 200 kHz to get maximum resolution.
	*/	
				
	// Enable ADC, set pre-scaller to /8 which will give a ADC clock of 8mHz/64 = 125kHz
	
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);
	
	/*
		After switching to internal voltage reference the ADC requires a settling time of 1ms before
		measurements are stable. Conversions starting before this may not be reliable. The ADC must
		be enabled during the settling time.
	*/
		
	_delay_ms(1);
				
	/*
		The first conversion after switching voltage source may be inaccurate, and the user is advised to discard this result.
	*/
	
		
	ADCSRA |= _BV(ADSC);				// Start a conversion


	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
						
		
	/*
		After the conversion is complete (ADIF is high), the conversion result can be found in the ADC
		Result Registers (ADCL, ADCH).		
		
		When an ADC conversion is complete, the result is found in these two registers.
		When ADCL is read, the ADC Data Register is not updated until ADCH is read.		
	*/
	
	// Note we could have used ADLAR left adjust mode and then only needed to read a single byte here
		
	uint8_t low  = ADCL;
	uint8_t high = ADCH;

	uint16_t adc = (high << 8) | low;		// 0<= result <=1023
			
	// Compute a fixed point with 1 decimal place (i.e. 5v= 50)
	//
	// Vcc   =  (1.1v * 1024) / ADC
	// Vcc10 = ((1.1v * 1024) / ADC ) * 10			->convert to 1 decimal fixed point
	// Vcc10 = ((11   * 1024) / ADC )				->simplify to all 16-bit integer math
				
	uint8_t vccx10 = (uint8_t) ( (11 * 1024) / adc); 
	
	/*	
		Note that the ADC will not automatically be turned off when entering other sleep modes than Idle
		mode and ADC Noise Reduction mode. The user is advised to write zero to ADEN before entering such
		sleep modes to avoid excessive power consumption.
	*/
	
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	
	return( vccx10 );
	
}

// Set the motor to run at the specified duty cycle and frequency
// The duty cycle is specified at 4.2 volts as a value 0-65535. It is adjusted to scale to the actual voltage. 
// Of course if you specify 100% at 4.2v and only 3.8v is available, then it will just give 100% at the current voltage

void updateMotor( uint16_t top, uint16_t normalizedDuty, uint8_t vccx10 ) {
			
	unsigned long voltageAdjustedDuty = (((normalizedDuty * 42UL ) / vccx10) );		// All dutys are normalized to 4.2 volts, so adjust to the current volatge level. Note that is could overflow an uint16 if the voltage is lower than the normal value. 
	
	unsigned long voltageAdjusedMatch = (voltageAdjustedDuty  * top ) / 65535UL;	// Covert the duty that is scaled 0-65535 to a match that is scaled 0-top.
																					// Match = (duty/65535) * top, but we need to stay integer so switch the order
																					// Keep as a long because it could be bigger than an int due to scaling because of a low voltage
		
	uint16_t match;
	
	if (voltageAdjusedMatch > top ) {		// Battery to low for requested duty, so give it all we've got
		
		match = top; 
		
	} else {
		
		match = (uint16_t) voltageAdjusedMatch;		// We know that adjusted duty will fit into uint_16 here because it is less than top which is a uint16
		
	}
			
	setMotorPWM( match , top  );

}


// We use Timer0 for timing functions and also PWMing the LEDs

#define TIMER0PRESCALER	8

#define TIMER0_STEPS_PER_S	(CYCLES_PER_S/TIMER0PRESCALER)

#define TIMER_0_STEPS_PER_CYCLE 256		// 8-bit timer overflow

#define TIMER0_CYCLES_PER_S (TIMER_0_STEPS_PER_CYCLE/TIMER0_STEPS_PER_S)

// With a 1Mhz clock, the cycle rate comes out to 488.3 hertz, which is more than fast enough for no flicker on the LEDs

// Note that this just turns on the timer. For the LEDs to come on, we need to set the control bits to let the compare bits show up on the pins
// Also note that we are running in inverted mode, which means there will be a tiny glitch each cycle at full power (I should have put the LEDs in backwards!)

void enableTimer0() {
		
	TCNT0 = 0;		// Start timer counter at 0;
	
	TCCR0A = _BV( WGM01) | _BV( WGM00 ) ;	// Mode 3 Fast PWM TOP=0xff, update OCRx at BOTTOM
		
		//   0bxxxx0xxx	-~WGM02				Mode 3 Fast PWM TOP=0xff, update OCRx at BOTTOM
		//	 0bxxxxx010 CS01				clk/8 (From prescaler). 		
		//   ===========
	TCCR0B = 0b00000010;	
	
	OCR0A = 0;		// Start with LEDs off
	OCR0B = 0;	
	
//	TIMSK0 = _BV(TOIE0);		// Enable interrupt on overflow
		
}


// TODO: user timer0 overflow interrupt for timing 
//EMPTY_INTERRUPT( TIM0_OVF_vect );


void disableTimer0() {

	TCCR0B = 0;			// No clock, timer stopped. 
	TIMSK0 = 0;			// No interrupts from anywhere!
	
}



/*
// Called when the button is pressed

void buttonShortPress() {
	
	
}

// Called when the button is held down longer than BUTTON_LONG_PRESS_MS
// Note that a long press will always be preceded by a short press

void buttonLongPress() {
	
	
}

*/


// Set brightness of LEDs. 0=off, 255=full on

void setWhiteLED( uint8_t b ) {
	
	WHITE_LED_DDR  |= _BV(WHITE_LED_BIT);		// Pin to output
	
	WHITE_LED_PORT &= ~_BV(WHITE_LED_BIT);		// Output to low if we end up being off
	
	OCR0A = ~b;									// Set the compare register (even though it won't matter if set to zero)
												// not-ed becuase we are generating an inverted waveform				
	if (b==0)	{	// Off
		
		TCCR0A &= ~ ( _BV( COM0A1  ) | _BV( COM0A0 ) );		// Normal port operation, OC0A disconnected (happens to hold true for all modes)
	
	} else {
		
		TCCR0A |= ( _BV( COM0A1  ) | _BV( COM0A0 ) );		// Set OC0A on Compare Match, Clear OC0A at BOTTOM (inverting mode)
						
	}		
		
}

void setRedLED( uint8_t b ) {
	
	RED_LED_DDR  |= _BV(RED_LED_BIT);
	
	RED_LED_PORT &= ~_BV(RED_LED_BIT);
			
	OCR0B = ~b;									// Set the compare register (even though it won't matter if set to zero)
												// not-ed becuase we are generating an inverted waveform
	
	if (b==0)	{	// Off
		
		TCCR0A &= ~ ( _BV( COM0B1  ) | _BV( COM0B0 ) );		// Normal port operation, OC0B disconnected (happens to hold true for all modes)
		
	} else {
		
		TCCR0A |= ( _BV( COM0B1  ) | _BV( COM0B0 ) );		// Set OC0B on Compare Match, Clear OC0B at BOTTOM (inverting mode)
		
	}
	
}


#define REBOOT() 	{wdt_enable( WDTO_250MS); while(1);}		// Timeout is long enough to see the LEDs flash, then wait for the inevitable


// Dummy ISRs for the pin change interrupts.
// These will catch and wake on..
// *Change in battery charger status lines
// *Incoming bit on the power port

EMPTY_INTERRUPT( PCINT0_vect );

	// This is a dummy routine. This is here just so the processor has something to do when it wakes up.
	// This will just return back to the main program. 
	// TODO: Figure out how to just put an IRET in the vector table to save time and code space.


EMPTY_INTERRUPT( PCINT1_vect );

	// This is a dummy routine. This is here just so the processor has something to do when it wakes up.
	// This will just return back to the main program.
	// TODO: Figure out how to just put an IRET in the vector table to save time and code space.

/*

// This ISR will catch a button state change
// It can only set the volatile button down state flag. It is reset by the foreground code
// This is probably overkill in this application, but keeps everything very well defined

// Note that there is STILL a tiny race condition if the button state only changes for an amount of time shorter than it takes for the interrupt
// to get called and then check the state of the button bit. As far as I can tell from the data sheet, there is no way to see *which* pin triggered a 
// pin change interrupt, so I do not think it is possible to get rid of this race- although it is extremely unlikely (button push for less than 10us?)

volatile uint8_t button_down_triggered =0;
volatile uint8_t button_up_triggered =0;

uint8_t previousButtonDownState =0;

ISR( PCINT1_vect ) {
	
	uint8_t newButtonDownState = BUTTON_STATE_DOWN();
	
	if (previousButtonDownState != newButtonDownState )		{	// Button state changed?( Interrupt could have been from another pin change)
		
		if (newButtonDownState) {
			
			button_down_triggered = 1;
						
		} else {
			
			button_up_triggered = 1;
					
		}
		
	}
			
}

*/


uint16_t jack_data =0;

// Red data from the power jack
// Called if jack is low

void readJack() {
	
									
	uint8_t		bits = 10;
	uint16_t	bytes =  0;
				
	while (bits>0) {
		

		unsigned int bitTimeout = 3 * CYCLES_PER_MS;			// Wait at most 10 ms for next bit

		while (!JACK_STATE_LOW()) {				// Wait for low at start of bit (will already be low when entering 1st time)
			
			bitTimeout--;
			
			if (bitTimeout==0) return;
			
		}

								
		while (JACK_STATE_LOW()) {				// Wait for rising pulse
			
			bitTimeout--;
			
			if (bitTimeout==0) return;
			
		}

		_delay_ms(1);			// Wait for 1 ms before sampling data bit	
		
		bytes <<=1;
				
		if ( !JACK_STATE_LOW() ) {
						
			bytes |= 0x01;				// Put this bit at the bottom of the data byte we are building
			
		}
		
		bits--;
				
	}				
	
	
	jack_data = bytes;
	
	
	/*
	
	PORTA &= ~_BV(4);
	_delay_us(100);
	PORTA |= _BV(4);
	_delay_us(100);			
	PORTA &= ~_BV(4);
	_delay_us(100);
	
						
	
	uint8_t mask = 0b10000000;
	
	for(int p=0; p<8;p++ ) {


		if (jack_data & mask) {
			PORTA |= _BV(4);
		} else {			
			PORTA &= ~_BV(4);			
		}
		
		_delay_us(200);
			
		
	}
	
	PORTA &= ~_BV(4);
	
	*/

		
	if (jack_data==0) setRedLED(128); else setRedLED(0);
	
}

// EEPROM layout

uint8_t EEMEM eeprom_start_cookie	='J';			// Real data?
uint8_t EEMEM eeprom_ver			=1;				// Block Version 
uint8_t EEMEM eeprom_badisr_flag	=0;				// Ever seen a bad ISR?
uint8_t EEMEM eeprom_badisrs_flag	=0;				// Seen more than one bad ISR?
uint8_t EEMEM eeprom_end_cookie		='L';			// Real data?

// Just in case we ever get a bad interrupt, best thing to do is flash LEDs a bit so it is not totally silent
// and then RESET. Record the event in the EEPROM black box.

// Naked is fine because the only place to go from here is RESET

ISR( BADISR_vect , ISR_NAKED ) {
	
	RED_LED_DDR |= _BV(RED_LED_BIT);
	RED_LED_PORT|= _BV(RED_LED_BIT);
	
	WHITE_LED_DDR |= _BV(WHITE_LED_BIT);
	WHITE_LED_PORT|= _BV(WHITE_LED_BIT);

	// Interrupts are automatically disabled when we get here
	
	if ( !eeprom_read_byte( &eeprom_badisr_flag ) ) {			// Never seen this before?
		
		eeprom_write_byte( &eeprom_badisr_flag , 0x01 );
		
	} else if ( !eeprom_read_byte( &eeprom_badisrs_flag ) ) {	// Only seen once before?
		
		eeprom_write_byte( &eeprom_badisrs_flag , 0x01 );
	}
	
	REBOOT();	
	
}

// Note that the architecture here is a little unconventional. We are constantly resetting all the registers which might seem
// wasteful, but this protects us from many glitches. We have plenty of CPU cycles to spare, and in this application
// the board is sealed up so there is no way to do a battery pull in case of a glitch, so it is worth the extra work. 

int main(void)
{
	MCUSR &= ~ _BV( WDRF );		// Just in case we are coming out of a Watchdog reset
								// "In safety level 1, WDE is overridden by WDRF in MCUSR...."
								// "This means that WDE is always set when WDRF is set."
	
	wdt_disable();		// Just in case the WDT somehow got enabled (or we committed WDT suicide to RESET), we need to kill it before it kills us.
	
	motorOff();			// Always turn the motor off right away on start up. This makes resistor R5 unnecessary.
	
	BUTTON_DDR &= ~_BV(BUTTON_BIT);		// Make sure pin is input mode
	BUTTON_PORT |= _BV(BUTTON_BIT);		// Enable pull-up for button pin
		
	RED_LED_DDR |= _BV(RED_LED_BIT);
	WHITE_LED_DDR |= _BV(WHITE_LED_BIT);
		
	// Blink alternating LEDs at 10Hz to verify...
	//		(1) verify successful power up
	//		(2) quickly check that both LEDs work
	//		(3) test button can go down (pushing button terminates)
	
		
	for( int i=0; i< 100 && !BUTTON_STATE_DOWN(); i++) {
		
		WHITE_LED_PORT &= ~_BV(WHITE_LED_BIT);
		RED_LED_PORT |= _BV(RED_LED_BIT);
		
		for(uint8_t k=0;k<100 && !BUTTON_STATE_DOWN();k++) {
			_delay_ms(1);
		};
		
		RED_LED_PORT &= ~_BV(RED_LED_BIT);
		WHITE_LED_PORT |= _BV( WHITE_LED_BIT);
		
		for(uint8_t k=0;k<100 && !BUTTON_STATE_DOWN();k++) {
			_delay_ms(1);
		};
						
	}
	
	// Both LEDs pulse on while the button is still depressed
	
	enableTimer0();			// Initialize the timer that also PWMs the LEDs
			
	// (times out after 20 seconds)
	
	// Ramp both LEDs down slowly if button still pressed
		
	for( int i=0; i< 100 && BUTTON_STATE_DOWN(); i++) {
		
			for( uint8_t j=0; j<255 && BUTTON_STATE_DOWN() ;j++ ) {
				setRedLED(~j);
				setWhiteLED(~j);
				_delay_ms(1);
			}
			
	}
	
			
	// TODO: Put more code here for some testing and feedback on initial battery connection at the factory. 
						
	// Ready to begin normal operation!
	
	uint8_t buttonStuckFlag = 0;		// Set if the button is held down for a long time
										// Disables the pull-up on the button to save power. 
							
			
	while (1)	{	// Master loop. We pass though here every time we wake from sleep or power up. 
			
		motorOff();			// Turn the motor off again just in case there was a glitch. 
		
		// TODO: ? disableTimer0();	// Should automatically get turned off durring sleep, but justto be safe
				
		setRedLED(0);		// LEDs off 
		setWhiteLED(0);		
								
		// Now get everything set up to wake us when necessary
		
		
		// Jack data setup
		
		JACK_DDR &= ~_BV(JACK_BIT);			// Jack is input 		
		JACK_PORT &= ~_BV(JACK_BIT);		// Disable pull-up on jack just in case there is a drain (maybe from unplugged power supply?)
						
		// Battery Charger status pin setup
				
		EOC_DDR  &= ~_BV(EOC_BIT);				// Make sure input mode
		EOC_PORT |= _BV(EOC_BIT);				// Activate pull-up
		
		CIP_DDR &= ~_BV(CIP_BIT);				// Make sure input mode
		CIP_PORT |= _BV( CIP_BIT);				// Activate pull-up
				
		// Get ready to sleep
						
		// We must enable the interrupts and then clear the flags *before* testing to avoid a race condition where
		// one of the states changes between when we check it and when we turn on the interrupts.

		// (I know that you want to consolidate these bitfield operations, but it is ok because this code is clear and easily changed, and compiles to efficient SBI CLI opcodes)
									
		PCMSK0 |= _BV(EOC_INT);					// Enable interrupt on change in state-of-charge pin
		PCMSK0 |= _BV(CIP_INT);					// Enable interrupt on change in end-of-charge pin
		

		// Button setup
		
		BUTTON_DDR &= ~_BV(BUTTON_BIT);		// Make sure pin is input mode
		
		// Disable button pullup when button is stuck to keep from killing battery

		if (buttonStuckFlag) {							
				
			BUTTON_PORT &= ~_BV(BUTTON_BIT);		// disable the pullup resistor to keep from running the battery dead
				
				
			PCMSK1 &= ~_BV(BUTTON_INT);		// Disable interrupt on button pin.
			// This will cause the pin to be disconnected when we sleep which
			// will save power incase it starts to float.
			
			// Note that we will only be able to wake on a change in battery charger state, but that is
			// ok since a stuck button is an error condition and worth not killing the battery for
				
			// "the digital input signal can be clamped to ground at the
			// input of the schmitt-trigger. The signal denoted SLEEP in the figure, is set by the MCU Sleep
			// Controller in Power-down and Standby modes to avoid high power consumption if some input
			// signals are left floating, or have an analog signal level close to VCC/2.
			// SLEEP is overridden for port pins enabled as external interrupt pins. If the external interrupt
			// request is not enabled, SLEEP is active also for these pins."
			
		} else {	// Button is up like it should be (not stuck), so let a button press wake us!
				

			BUTTON_PORT |= _BV(BUTTON_BIT);		// Enable pullup
			PCMSK1 |= _BV(BUTTON_INT);				// Enable interrupt on button pin so we wake on a press
						
		}


		// TODO: For now we will not wake on a jack data connect
		//		PCMSK1 |= _BV(JACK_INT);				// Enable interrupt on jack input so we wake on a slave connection request
		
		GIFR = _BV(PCIF1) | _BV(PCIF0);			
		
		// Clear pending interrupt flags. This way we will only get an interrupt if something changes
		// after we read it. There is a race condition where something could change between the flag clear and the
		// reads below, so code should be able to deal with possible redundant interrupt and worst case
		// is that we get woken up an extra time.										
		
		// Any reason to stay awake? If any of these are set now, then skip going to sleep since the interrupt will only happen on changes
		
		if ( !CIP_STATE_ACTIVE() && !EOC_STATE_ACTIVE() && (buttonStuckFlag || !BUTTON_STATE_DOWN()) /* && !JACK_STATE_LOW() */ ) {
						
			// Ok, it is bedtime!			
		
			GIMSK |= _BV(PCIE1) | _BV(PCIE0);		// Enable both pin change interrupt vectors (each individual pin was also be enabled above)
				
			set_sleep_mode( SLEEP_MODE_PWR_DOWN );  // Go into deep sleep where only a pin change can wake us.. uses only ~0.1uA!
		
			// GOOD NIGHT!
		
			sleep_enable();							// "To enter any of the three sleep modes, the SE bit in MCUCR must be written to logic one and a SLEEP instruction must be executed."
			sei();                                  // Enable global interrupts. "When using the SEI instruction to enable interrupts, the instruction following SEI will be executed before any pending interrupts." 
			sleep_cpu();							// This must come right after the sei() to avoid race condition
				
			// GOOD MORNING!
			// If we get here, then a button push or change in charger status woke s up....
				
			sleep_disable();						// "To avoid the MCU entering the sleep mode unless it is the programmer�s purpose, it is recommended to write the Sleep Enable (SE) bit to one just before the execution of the SLEEP instruction and to clear it immediately after waking up."
			cli();									// We are awake now, and do don't care about interrupts anymore (out interrupt routines don't do anything anyway)						
						
		}
		
		if (buttonStuckFlag) {
		
			buttonStuckFlag=0;		// Optimistically assume it was fixed.  It will timeout again if still stuck.
			
			BUTTON_PORT |= _BV(BUTTON_BIT);		// re-Enable pullup					
		}
		
		// Ok, we are now turned on and ready for whatever action comes (came?) our way!
						
		// TODO: Turn on watchdog here? If added, remember to turn it off before going to sleep.
		
		// Button Debounce Strategy:
		// For fast response, we want react to a button down instantly without a debounce delay. We debounce by only accepting a button
		// down trigger if the button has already been in a steady up state for at least DEBOUNCE_TIME_MS.
				
			
		uint16_t buttonUpCountdown=0;				// for debouncing. when it gets to zero then we can detect another button push.
		uint16_t buttonDownCountup=0;				// How long has the button been held down for?
													// Note that we do not goto sleep until all bouncing is over so we know there is no
													// bounce when we wake up 
				
		uint16_t redLedCountdown=0;					// If non-zero, then the LED is on and will stay on for this many ms
		
		typedef enum { WHITELED_OFF, WHITELED_BLINK, WHITELED_BREATH, WHITELED_ON } whiteLEDStates ;
		whiteLEDStates whiteLEDState=WHITELED_OFF;			// White LED used for charger status
		
				
		// Enable pull up on jack. This will provide (a tiny amount of) power											
		// and let us detect a slave trying to communicate
				
		JACK_PORT |= _BV(JACK_BIT);			// Enable pull up on jack. This will provide (a tiny amount of) power											// To the connected slave until it can wake up.
		
		// Motor speed
		uint8_t currentSpeedStep = 0;				// What motor speed setting are we currently on?
		
													
		uint8_t ticks=0;							// monotonically increments on each pass tough main even loop from 0 to 255 and then resets. 
				
		enableTimer0();				// Initialize the 488Hz timer that also PWMs the LEDs
		
		
		DDRA |= _BV(4);				// Diagnostics on SCK 
									// TODO: Get rid of this.
		
						
		do {						// Everything in here is our normal ON operation loop
									// Note that we don't even bother to goto sleep while we are on because the power
									// usage of the processor is so tiny compared to the motor and LEDs
									// The loop takes about 1ms for each pass, due manly to the delay inside readVccVoltage()
					
									// A consequence of this strategy is that a button press must be at least 1ms long to be reliably detected									
									// Typical buttons bounce for at least 10ms, so this should not be a problem here.
																																				
			uint8_t buttonStateDown		= BUTTON_STATE_DOWN();			// Read actual button position		
			uint8_t EOCState			= EOC_STATE_ACTIVE();		// Read actual end-of-charge state
			uint8_t CIPState			= CIP_STATE_ACTIVE();		// Read actual charge-in-progress state
			
			
			// First process any button changes, step the motor speed if new button press
			
			if ( buttonStateDown )	{	// Button currently pressed?
								
				if (buttonUpCountdown==0) {			// Is this a new press event?
					
					// Button just pushed
					
					currentSpeedStep++;			// Update to next speed setting
															
				} else {
					
					// Button held down
					
					buttonDownCountup++;
					
					if (buttonDownCountup>=LOOPS_PER_MS(BUTTON_LONG_PRESS_MS)) {
						
						// Long press triggered
						
						currentSpeedStep = 0;		// Turn off motor, which will also put us to sleep when the button is debounced
												
						if (buttonDownCountup >= LOOPS_PER_MS( BUTTON_STUCK_TIMEOUT_MS )) {
							
							buttonStuckFlag = 1;
							
						}
					}
					
				}
				
				
				buttonUpCountdown = LOOPS_PER_MS(DEBOUNCE_TIME_MS);		// Start countdown for debounce

				
			} else {		// Button currently up
				
				buttonDownCountup=0;		// not being held down any more, so reset timer
				
				if (buttonUpCountdown) buttonUpCountdown--;
				
			}
			


			if (currentSpeedStep>=SPEED_STEP_COUNT) {		// Fail safe overflow compare
				
				// We cycled though all motor steps (or currentSpeedStep glitched), so time to turn off
				
				currentSpeedStep=0;
				
			}			
			
			
			whiteLEDState = WHITELED_OFF;		 // Start with white off...	
			
									
			// Charger is attached?

			if (CIPState || EOCState) {		// Is the charger connected?
				
				currentSpeedStep=0;					// Always turn motor off when charger connected (1) for safety, (2) so motor current doesn't interfere with charger sensing the end of charge on the battery

				if (CIPState && EOCState)	{	// Both charging and end of charge?	This is an error state that according to the datatasheet indicates "System Test Mode". We should really never see this.
					
					whiteLEDState=WHITELED_BLINK;
					
					} else if (CIPState) {			// Charge currently in progress
					
					whiteLEDState=WHITELED_BREATH;
					
					} else {							// End of charge
					
					whiteLEDState=WHITELED_ON;
				}								   
							
			} 				
				
			
				
			uint8_t vccx10 = readVccVoltage();				// Capture the current power supply voltage. This takes ~1ms and will be needed multiple times below
			
			
			if (vccx10<=LOW_BATTERY_VOLTSx10 && currentSpeedStep) {				// Motor on and low battery? (Also triggers if the user presses the button to turn on the motor, but cathces before the motor comes on)
				
				currentSpeedStep=0;												// Turn off motor
				redLedCountdown = LOOPS_PER_MS(LOW_BATTERY_LED_ONTIME_MS);		// Blink red LED
				
			}
																																		
				
			// Ok, set outputs	(motor and LEDs)
					
										
			if (currentSpeedStep==0) {		// Special case this out for aesthetics even though updateMotor would work at zero
							
				motorOff();					// Immediately turn off motor so we do not need to wait for a denounce for it to actually go off
				
			} else {
								
				updateMotor( speedSteps[currentSpeedStep].top , speedSteps[currentSpeedStep].normailzedDuty, vccx10);		// Set new motor speed			
			}
			
			// Next check for a request on the data jack....


			PINA |= _BV(4);
			
			if (JACK_STATE_LOW()) {
				
				readJack();

			}
			
			//PORTA &= ~_BV(4);
			
			
			
			switch (whiteLEDState) {
				
				case WHITELED_OFF:		
				
							// If we are not using the WHITE led for charge indication, then
							// Check if there was a recent button press so we can give use feedback
							
							if (buttonUpCountdown)	{		// There was a recent button push
								
								// Flash the white LED on button press and then fade out on release
								
								if (buttonUpCountdown>=64) {		// Max brightness 64 to keep it classy
									
									setWhiteLED(64);
									
								} else {
								
									setWhiteLED(buttonUpCountdown);
									
								} 
							} else {
												
								setWhiteLED(0);
								
							}
							break;
							
				
				case WHITELED_ON:
							setWhiteLED(255);
							break;
				
				case WHITELED_BREATH:					// A slow, even breathing
				
							if (ticks < 128)	{		// Breath up
								
								setWhiteLED(ticks);
								
							} else {					// Breath down
								
								setWhiteLED( ~ticks );
								
							}
				
							break;
							
				case WHITELED_BLINK:					// A quick blink
				
							if (ticks & 0b00010000) {								
								setWhiteLED(255);
							} else {
								setWhiteLED(0);
							}
							break;
							
				default:							// Should never get here, signal with a very fast blink							
				
							if (ticks & 0b00001000) {	// On 1/2 the time with a period of ticks/32
								setWhiteLED(1);
								} else {
								setWhiteLED(0);
							}
							break;
				
			}
			
					
			/*
			if (redLedCountdown>1) {
				
				setRedLED(255);				
				redLedCountdown--;
				
			} else {
								
				setRedLED(0);
				redLedCountdown=0;
			}
			*/
			ticks++;				// uint8 so will wrap at 0xff back to 0
								
						
		} while ( (currentSpeedStep || buttonUpCountdown>0 || redLedCountdown || (whiteLEDState!=WHITELED_OFF) ) && (!buttonStuckFlag) );
		
																// Stay on if...
		
																// Motor is running, or 
																
																// we are in a debounce debounce wait (if we sleep while debouncing, then we will wake up and see the bounce as a new press)
																// ...but the button is not stuck (ok to sleep on stuck button)
																
																// The white LED is in use (indicating some charging status)
																
																// the red LEDs is on (like showing a Recent low battery shutoff)
																
																// and we did not timeout with a stuck down button
																															
																// note that we want to keep looping when motor is on so we can continuously adjust the duty cycle to changing Vcc voltage.  
    }
}