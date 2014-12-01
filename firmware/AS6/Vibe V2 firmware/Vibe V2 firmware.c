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

#define F_CPU 1000000						// Name used by delay.h 

#define CYCLES_PER_S F_CPU					// Better name

#define CYCLES_PER_MS (F_CPU/1000UL)		// More convenient unit

// Inputs

#define BUTTON_PORT PORTB
#define BUTTON_PIN	PINB
#define BUTTON_DDR  DDRB
#define BUTTON_BIT	0
#define BUTTON_INT	PCINT8

// These just make it easier to compare/store button states 
#define BUTTON_STATE_UP		(_BV(BUTTON_BIT))
#define BUTTON_STATE_DOWN	(0)
#define BUTTON_STATE_READ() (BUTTON_PIN & _BV(BUTTON_BIT))


// Outputs

#define WHITE_LED_PORT PORTB
#define WHITE_LED_DDR DDRB
#define WHITE_LED_BIT 2

#define RED_LED_PORT PORTA
#define RED_LED_DDR DDRA
#define RED_LED_BIT 7

// EOC is the end-of-charge (battery full) signal. It s Active LOW.
// It is connected to the STAT2 line from the battery controller
// Note we must be pull up this line
#define EOC_PORT PORTA
#define EOC_DDR	 DDRA
#define EOC_PIN PINA
#define EOC_BIT 0
#define EOC_INT PCINT0

// CIP is the charge-in-progress (battery full) signal. It s Active LOW.
// It is connected to the STAT1 line from the battery controller
//
// Note we must be pull up this line
#define CIP_PORT PORTA
#define CIP_PIN  PINA
#define CIP_DDR	 DDRA
#define CIP_BIT 1
#define CIP_INT PCINT1


#define OCR1_PWM_TOP 1500U		// Set the motor PWM TOP which defines frequency

#define DEBOUNCE_TIME_MS 100		// How long to wait for a button press debounce

#define DEBOUNCE_CYCLES (DEBOUNCE_TIME_MS *  CYCLES_PER_MS)

#define LOW_BATTERY_VOLTSx10	(38-03)		// Low battery cutoff, 3.8 volts for battery less the 0.3V diode drop


#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


// Struct for holding speed steps

typedef struct {
		uint16_t normailzedDuty;			// Duty cycle normalized to 4.2 volts Vcc
		uint16_t top;						// Top value, which determines the PWM frequency where 	f = F_CPU/top	
} speedStepStruct;
	
	
#define SPEED_STEP_COUNT 3
	
const speedStepStruct speedSteps[SPEED_STEP_COUNT] = {
	
	{ 10, 200 },
	{ 250, 500 },
	{ 700, 1000 },
		
};



// MOTOR FUNCTIONS
// ===============
// Note that register values are hard coded rather than #defined because they 
// can not just be moved around.

// Turn the motor completely off- disconnects from PWM generator
// You should call this immedeately on reset to turn off the motor MOSFET 

void motorOff(void) {

	// First get the pin low so the MOSFET doesn't turn on.
	// Try to do this very early in the startup.
	
	PORTA &= ~_BV(5);	// Set pin output to low (until now, the pin was default input and the MOSFET was held low by R5
	
	
	DDRA |= _BV(5);		// Set pin to output mode


	TCCR1A &= ~( _BV(COM1B1) | _BV(COM1B0) );			// Disconnect Timer1A outputs from pins. "Normal port operation, OC1B disconnected"
	
}


// SetVFD motor PWM on pin 8/PA5/OC1B
// Note: also uses OCR1A for TOP function.
// Note: resets all used registers each time from scratch for safety from glitches

// match sets the duty cycle and should be between 0 and top. 0=off, top=on. 
// top sets the frequency where PWM frequency = F_CPU/top

void setMotorPWM( uint16_t match , uint16_t top ) {
	
	if (match==0) {			// Special case this because the PWM generator still generates a pulse at 0 duty cycle
		
		motorOff();
		
	} else {
			
		
		// Set OC1B on Compare Match
		// Clear OC1B at BOTTOM (inverting mode)
	
		// Fast PWM, TOP= OCR1A, Update OCR1x at top
	
		// Clock select clk	I/O/1 (No prescaling)
	
	
		//			0bxx100000	COM1B		PWM Fast mode, Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
		//			0bxxxxxx11	WGM[11:10]	Fast PWM, TOP=OCR1A, Update at OCR TOP
	
		TCCR1A =	0b00100011;
	
		//			0b00011000	WGM[13:12]	Fast PWM TOP=OCR1A UPDATE=TOP
		//			0b00000001	CS			clk	I/O/1 (No prescaling)
	
		TCCR1B =	0b00011001;
	
		TCNT1  = 0x00;		// Start counting at zero
	
		//Below should be handled in motorOn
	
		OCR1A = top;		// Set TOP. Freq should be IOclk/OCR1A = 16Khz
	
		OCR1B = match;		// Set match which sets duty cycle
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
				
	// Enable ADC, set prescaller to /8 which will give a ADC clock of 8mHz/64 = 125kHz
	
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


volatile uint8_t currentSpeedStep = 0;


// Set the motor to run at the current speed step

void updateMotor(void) {
	
	uint16_t top = speedSteps[currentSpeedStep].top;
	
	uint16_t normalizedDuty = speedSteps[currentSpeedStep].normailzedDuty;		// Get the normalized duty setting which is at 4.2 volts
		
	unsigned long adjusedDuty = (normalizedDuty *42UL) / readVccVoltage();	// Adjust duty to current voltage level, use long int to save precicion
	
	uint16_t duty;
	
	if (adjusedDuty > top ) {		// Battery to low for reqested duty, so give it all we've got
		
		duty = top; 
		
	} else {
		
		duty = (uint16_t) adjusedDuty;		// We know that adjusted duty will fit into uint_16 here becuase it is less than top which is a uint16
		
	}
			
	setMotorPWM( duty , speedSteps[currentSpeedStep].top  );

}

// Called when the button is pressed

void buttonShortPress() {
	
	
}

// Called when the button is held down longer than BUTTON_LONG_PRESS_MS
// Note that a long press will always be preceded by a short press

void buttonLongPress() {
	
	
}


void setWhiteLED( uint8_t b ) {
	
	if (b==0) {
		
		WHITE_LED_PORT &= ~_BV(WHITE_LED_BIT);
		
		
	} else {
		
			WHITE_LED_PORT |= _BV(WHITE_LED_BIT);
			WHITE_LED_DDR  |= _BV(WHITE_LED_BIT);
		
	}
		
}

void setRedLED( uint8_t b ) {
	
	if (b==0) {
		
		RED_LED_PORT &= ~_BV(RED_LED_BIT);
		
		
		} else {
		
			RED_LED_PORT |= _BV(RED_LED_BIT);
			RED_LED_DDR  |= _BV(RED_LED_BIT);
		
	}
	
}


// Dummy ISRs for the pin change interrupts.
// These will catch and wake on..
// *Button press
// *Change in battery charger status lines
// *Incoming bit on the power port

ISR( PCINT0_vect ) {
	// This is a dummy routine. This is here just so the processor has something to do when it wakes up.
	// This will just return back to the main program. 
	// TODO: Figure out how to just put an IRET in the vector table to save time and code space.
}

ISR( PCINT1_vect ) {
	// This is a dummy routine. This is here just so the processor has something to do when it wakes up.
	// This will just return back to the main program.
	// TODO: Figure out how to just put an IRET in the vector table to save time and code space.
}



// Note that the architecture here is a little unconventional. We are constantly resetting all the registers which might seem
// wasteful, but this protects us from many glitches. We have plenty of CPU cycles to spare, and in this application
// the board is sealed up so there is no way to do a battery pull in case of a glitch, so it is worth the extra work. 

int main(void)
{

	// Put code here for some testing and feedback on initial battery connection at the factory. 
		
	while (1)	{	// Master loop. We pass though here every time we wake from sleep or power up. 
		
		motorOff();			// Always turn the motor off right away on start up. This makes resistor R5 unnecessary.
		
		setRedLED(0);		// LEDs off and in output state
		setWhiteLED(0);
		
		// Button setup
		BUTTON_DDR &= ~_BV(BUTTON_BIT);		// Make sure pin is input mode
		BUTTON_PORT |= _BV(BUTTON_BIT);		// Enable pull-up for button pin
		
		PCMSK1 |= _BV(BUTTON_INT);		// Enable interrupt on button pin
				
		// Battery Charger status pin setup
		
		EOC_DDR  &= ~_BV(EOC_BIT);				// Make sure input mode
		EOC_PORT |= _BV(EOC_BIT);				// Activate pull-up 
		
		CIP_DDR &= ~_BV(CIP_BIT);				// Make sure input mode
		CIP_PORT |= _BV( CIP_BIT);				// Activate pull-up 
	
		
		PCMSK0 |= _BV(EOC_INT);					// Enable interrupt on change in state-of-charge pin
		PCMSK0 |= _BV(CIP_INT);					// Enable interrupt on change in end-of-charge pin
		
		GIMSK |= _BV(PCIE1) | _BV(PCIE0);		// Enable both pin change interrupt vectors (each individual pin must also be enabled)
		
		
		set_sleep_mode( SLEEP_MODE_PWR_DOWN );    
		
		// GOOD NIGHT!
		// We sit here when not in use to keep from draining the battery.
		
		sleep_enable();
		sei();                                  // Enable global interrupts    		
		sleep_cpu();							// This must come right after the sei() to avoid race condition
		
		
		// GOOD MORNING!
		// If we get here, then a button push or change in charger status woke us up....
				
		sleep_disable();		
		cli();									// We are awake now, and do don't care about interrupts anymore
						
		// TODO: Turn on watchdog here?
				
		uint8_t sleepFlag=0;						// A flag to tell us when to go back to sleep
		
		uint8_t buttonState = BUTTON_STATE_UP;		// Assume button up coming out of sleep
		uint8_t debounceCountDown=0;				// When is it safe to register next button state change?
		
		uint8_t ledState = 0;		// 0= Red, 1=white
		
		uint8_t off_count = 0;
		
		while (!sleepFlag)	{		// Everything here is our normal ON operation loop
									// Note that I don't even bother to goto sleep while we are on because the power
									// usage of the processor is so tiny compared to the motor and LEDs
					
			GIFR = _BV(PCIF1) | _BV(PCIF0);					// Clear pending interrupt flags. This way we will only get an interrupt if something changes
															// after we read it. There is a race condition where something could change between the flag clear and the
															// reads below, so code should be able to deal with possible reduncant interrrupt and worst case
															// is that we get woken up a n extra time. 
												
			uint8_t newButtonState = BUTTON_STATE_READ();	// Read actual button position		
			uint8_t newEOCState = (EOC_PIN & _BV(EOC_BIT));	// Read actual end-of-charge state
			uint8_t newCIPState = (CIP_PIN & _BV(CIP_BIT));	// Read actual charge-in-progress state
						
			if ( newButtonState != buttonState )	{	// Has the button state changed?
				
				
				if (debounceCountDown==0)	{					// This is a real, new change and not just a bounce
					
					buttonState = newButtonState;
					
					if (buttonState == BUTTON_STATE_DOWN) {	// Ok, we got a new button down event...
						
						ledState = !ledState;		// Swap LEDs 
						
						currentSpeedStep++;			// Update to next speed setting
											
					} else {	// BUTTON_STATE_UP
												
						off_count++;
												
					}
				}
					
				debounceCountDown = DEBOUNCE_TIME_MS;			// Start counting down to debounce
					
			} else {											// Has not changed since last pass
				
				if (debounceCountDown) debounceCountDown--;		// ...so count down debounce timer if it is currently counting
				
			}
			
		
					
			if (currentSpeedStep>=SPEED_STEP_COUNT) {		// Fail safe overflow compare
			
				motorOff();
				currentSpeedStep=0;
				
				
			
			}
		
		for(int i=0;i<currentSpeedStep+1;i++) {
		
			setWhiteLED(1);
			_delay_ms(100);
			setWhiteLED(0);
			_delay_ms(100);
			
		}
		
		setRedLED(1);
		
		updateMotor();
			
		_delay_ms(1000);
				
		motorOff();
		
		setRedLED(0);
*/		
		
			if (ledState) {
			
				setWhiteLED(1);
				setRedLED(0);
			
			} else {
			
				setWhiteLED(0);
				setRedLED(1);
			
			}
			
			
			if (off_count==3 && !debounceCountDown) {
				sleepFlag=1;
				_delay_ms(300);
			}
			
		}
    }
}