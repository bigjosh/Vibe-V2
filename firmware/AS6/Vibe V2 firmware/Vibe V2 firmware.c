/*
 * Vibe_V2_firmware.c
 *
 * Created: 11/30/2014 12:25:41 PM
 *  Author: josh.com
 */ 


/*

 Note to gentle reader: 
 The target product is encased in a silicone overmold, so there is no practical way to do a "battery pull" reset 
 on the circuit in case of a latch-up. There was also no good way to add a hardware reset button to the product.
 
 Therefore, this is a defensive style designed to keep running and self correct.
 The program is designed to completely reset the MPU every time the motor turns off or the power charger
 is unplugged or the button is held down for too long.
 
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
// pull it low to to send a bits

#define JACK_STATE_LOW() (!(JACK_PIN & _BV(JACK_BIT))) 

#define JACK_ON()  (JACK_PORT |= _BV(JACK_BIT))			// Enable pull up on power jack. A connected controller will pull down to signal to communicate. 
#define JACK_OFF() (JACK_PORT &= ~_BV(JACK_BIT))		// Disable pull up on power jack. A connected controller will sense this and disconnect. 

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

#define BUTTON_DEBOUNCE_TIME_MS 25			// How long to wait for a button press debounce

#define JACK_DEBOUNCE_TIME_MS 100			// How long to wait for a battery charger state change debounce

#define BUTTON_LONG_PRESS_MS 500			// How long to hold down button to be considered a long press rather than a push
											// Long press immediately turns off motor without needing to cycle though remaining speeds


#define BUTTON_STUCK_TIMEOUT_S	(60*30)		// How long does the button need to be held down for to enter button lockout mode? 

// Different cutoffs because the drain of the motor on the battery lowers the voltage, which will recover when the 
// motor is turned off

#define LOW_BATTERY_VOLTS_WARMx10	(31-03)		// Low battery cutoff while running, 3.0 volts for battery less the 0.3V diode drop
#define LOW_BATTERY_VOLTS_COLDx10	(33-03)		// Low battery cutoff to turn on   , 3.3 volts for battery less the 0.3V diode drop

#define LOW_BATTERY_LED_ONTIME_MS (1000)	// Show low battery by flashing red LED for 1 second

#define BUTTON_FEEDBACK_BRIGHTNESS 100		// Blink the LEDs when the button is pressed at this brightness

#define REBOOT() 	{wdt_enable( WDTO_30MS);while(1);}		// Use watchdog to reset MPU. Note that this is sometimes used to also debounce button so probably not quicker than 32ms

#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

// Struct for holding speed steps

// TODO: Add pre-scaller for more dynamic range

typedef struct {
		uint16_t normailzedDuty;			// Duty cycle normalized to 4.2 volts Vcc. 0=off, 0xffff=full on at 4.2 volts power
		uint16_t top;						// Top value, which determines the PWM frequency where 	f = F_CPU/top	
} speedStepStruct;


	
#define SPEED_STEP_COUNT 4
	
const speedStepStruct speedSteps[SPEED_STEP_COUNT] PROGMEM = {
	
	{                    0,    0 },			// step 0 = off
	{		3136,  7510 },
	{	   12520, 10600 },
	{	   25400, 15660 },
		
};

// MOTOR FUNCTIONS
// ===============
// Note that register values are hard coded rather than #defined because they 
// can not just be moved around.


// Turn the motor completely off- disconnects from PWM generator

void motorOff(void) {

	
	TCCR1A = 0;		// Disconnect Timer1A outputs from pins. "Normal port operation, OC1B disconnected"	
					// Will revert back to PORT value, which is always zero
	
}

// Initialize the motor pin. Sets to output mode, which will drive is LOW 
// Call this as soon as possible after reset to keep the mosfet from floating and turning on the motor

void motorInit() {
	
	DDRA |= _BV(5);		// Set pin to output mode. It will already be low because ports default to 0 on reset
	
}

// SetVFD motor PWM on pin 8/PA5/OC1B
// Note: also uses OCR1A for TOP function.
// Note: resets all used registers each time from scratch for safety from glitches

// match sets the duty cycle and should be between 0 and top. 0=completely off, top=full on. 
// top sets the frequency where PWM frequency = F_CPU/top. The minimum resolution allowed is 2-bit (top set to 0x0003).


void setMotorPWM( uint16_t match , uint16_t top ) {
	
	if (match==0) {			// Special case this because the PWM generator still generates a pulse at 0 duty cycle
							// "If the OCR1x is set equal to BOTTOM (0x0000) the output will be a narrow spike for each TOP+1 timer clock cycle."
		
		motorOff();
		
	} else {
		
		
		// Set OC1B on Compare Match
		// Clear OC1B at BOTTOM (inverting mode)
	
		// Fast PWM, TOP= OCR1A, Update OCR1x at top
	
		// Clock select clk	I/O/1 (No prescaling)
		
		
		// Assign TOP first to make sure we don't miss the match
		
		OCR1A = top;							// Set TOP. Freq should be IOclk/OCR1A = 16Khz		
		OCR1B = match;		// Set match which sets duty cycle
		
		
		//			0bxx100000	COM1B		PWM Fast mode, Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
		//			0bxxxxxx11	WGM[11:10]	Fast PWM, TOP=OCR1A, Update at OCR TOP
	
		TCCR1A =	0b00100011;
	
		//			0b00011000	WGM[13:12]	Fast PWM TOP=OCR1A UPDATE=TOP, Compare output on pin
		//			0b00000001	CS			clk	I/O/1 (No prescaling)
	
		TCCR1B =	0b00011001;
	
		// "The actual OC1x value will only be visible on the port pin if the data direction for the port pin is set as output (DDR_OC1x)."
		// We set to output mode on startup
							
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

#define TIMER0_PRESCALER	1

#define TIMER0_STEPS_PER_S	(CYCLES_PER_S/TIMER0PRESCALER)

#define TIMER0_STEPS_PER_CYCLE 256		// 8-bit timer overflow

#define TIMER0_CYCLES_PER_S (TIMER_0_STEPS_PER_CYCLE/TIMER0_STEPS_PER_S)

// With a 1Mhz clock, the cycle rate comes out to 488.3 hertz, which is more than fast enough for no flicker on the LEDs

// Note that this just turns on the timer. For the LEDs to come on, we need to set the control bits to let the compare bits show up on the pins
// Also note that we are running in inverted mode, which means there will be a tiny glitch each cycle at full power (I should have put the LEDs in backwards!)

void enableTimer0() {
		
	TCNT0 = 0;		// Start timer counter at 0;
	
	TCCR0A = _BV( WGM01) | _BV( WGM00 ) ;	// Mode 3 Fast PWM TOP=0xff, update OCRx at BOTTOM
		
		//   0bxxxx0xxx	-~WGM02				Mode 3 Fast PWM TOP=0xff, update OCRx at BOTTOM
		//	 0bxxxxx001 CS01				clk/1 prescaler
		//   ===========
	TCCR0B = 0b00000001;	
	
	OCR0A = 0;		// Start with LEDs off
	OCR0B = 0;	
	
		
}


void disableTimer0() {

	TCCR0B = 0;			// No clock, timer stopped. 
	
}


// Set brightness of LEDs. 0=off, 255=full on

void setWhiteLED( uint8_t b ) {
	
	if (b==0)	{	// Off
		
		WHITE_LED_PORT &= ~_BV(WHITE_LED_BIT);				// Normal port Output to low 			
		TCCR0A &= ~ ( _BV( COM0A1  ) | _BV( COM0A0 ) );		// Normal port operation, OC0A disconnected (happens to hold true for all modes)
	
	} else {
		
		b/=16;												// Account for missing current limiting resistor - empirically found
		
		OCR0A = ~b;											// Set the compare register	- double buffered so will update at next top	
		TCCR0A |= ( _BV( COM0A1  ) | _BV( COM0A0 ) );		// Set OC0A on Compare Match, Clear OC0A at BOTTOM (inverting mode)
						
	}		
		
}

void setRedLED( uint8_t b ) {
	
	if (b==0)	{	// Off
		
		RED_LED_PORT &= ~_BV(RED_LED_BIT);					// Normal port output to low	
		TCCR0A &= ~ ( _BV( COM0B1  ) | _BV( COM0B0 ) );		// Normal port operation, OC0B disconnected (happens to hold true for all modes)
		
	} else {
		
		b/=16;												// Account for missing current limiting resistor - empirically found
		
		OCR0B = ~b;											// Set the compare register	- double buffered so will update at next top	
		TCCR0A |= ( _BV( COM0B1  ) | _BV( COM0B0 ) );		// Set OC0B on Compare Match, Clear OC0B at BOTTOM (inverting mode)
		
	}
	
}

void setLEDsOff() {
	setRedLED(0);
	setWhiteLED(0);
}

// Show signal sampling points on testpoint MOSI

//#define DEBUG_JACK 1

// Read data from the power jack
// Called if jack is low during the main event loop

// Returns 1 if speed was changed

uint8_t readJack( speedStepStruct *speedStepPtr ) {
	
	uint8_t		bits = 16;
	uint16_t	data =  0;
    
    static speedStepStruct recievedSpeedStep;
				
	while (bits>0) {
		
		unsigned int bitTimeout = 15 * CYCLES_PER_MS;			
              
		while (!JACK_STATE_LOW()) {				// Wait for low at start of bit (will already be low when entering 1st time)
			
			bitTimeout--;
			
			if (bitTimeout==0) {
                setWhiteLED(40);
                return(0);
            }                
			
		}

								
		while (JACK_STATE_LOW()) {				// Wait for rising pulse
			
			bitTimeout--;
			
			if (bitTimeout==0) {
    			setWhiteLED(255);
    			return(0);
			}
			
		}
        
        #ifdef DEBUG_JACK
            PORTA |= _BV(6);
        #endif
        
		_delay_ms(1);			// Wait for 1 ms so we are sampling level at the exact center of the bit	
		
		data <<=1;
				
		if ( !JACK_STATE_LOW() ) {
						
			data |= 0x01;				// Put this bit at the bottom of the data byte we are building
			
		}

        #ifdef DEBUG_JACK
            PORTA &= ~_BV(6);
        #endif
        
		
		bits--;
				
	}				
    
    uint8_t jack_command = data >> 12 ;
    uint8_t jack_param   = (data >> 4 ) & 0xff;
        
    uint8_t checksum = ( (jack_command) ^ ( jack_param ) ^ ( jack_param >> 4 ) ^ 0x0f ) & 0x0f;
    
    if ( checksum != (data & 0x0f) ) {          // Bad checksum        


        #ifdef DEBUG_JACK
        PORTA |= _BV(6);
        #endif
        
        #ifdef DEBUG_JACK
        PORTA &= ~_BV(6);
        #endif
        
        
        setRedLED(16);
        return(0);        
    }        
        
        
    
    switch (jack_command) {
        
        case 0x00:      // first byte of top
            recievedSpeedStep.top = jack_param << 8;
            break;
        
        case 0x01:      // second byte of top     
            recievedSpeedStep.normailzedDuty|= jack_param;
            break;
            
        case 0x02:      // first byte of duty
            recievedSpeedStep.normailzedDuty = jack_param << 8;
            break;
        
        case 0x03:      // second byte of duty
            recievedSpeedStep.normailzedDuty|= jack_param;
            break;
            
        case 0x04:      // Activate change atomically so we do not get an intermediate value
            speedStepPtr->top = recievedSpeedStep.top;
            speedStepPtr->normailzedDuty = recievedSpeedStep.normailzedDuty;
            return(1);
            break;
        
    }
    
    return(0);
    
}



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


int main(void)
{

    #ifdef DEBUG_JACK
    DDRA |= _BV(6);
    #endif
    
	motorInit();				// Initialize the motor port to drive the MOSFET low

	uint8_t watchDogResetFlag = MCUSR & _BV(WDRF);		/// Save the watchdog flag

	MCUSR &= ~ _BV( WDRF );		// Clear the watchdog flag
								// "In safety level 1, WDE is overridden by WDRF in MCUSR...."
								// "This means that WDE is always set when WDRF is set."
								// IF we left this Set, then we could get watchdogged while sleeping

	wdt_enable( WDTO_8S );		// Give ourselves 8 seconds before forced reboot
	
	enableTimer0();				// Initialize the timer that also PWMs the LEDs
	
	WHITE_LED_DDR	|= _BV(WHITE_LED_BIT);		// Pin to output
	RED_LED_DDR |= _BV(RED_LED_BIT);
	
	// Button sense pin setup	

	BUTTON_PORT |= _BV(BUTTON_BIT);		// Enable pull-up for button pin
	
	// Battery Charger status pin setup
		
	EOC_PORT |= _BV(EOC_BIT);				// Activate pull-up
		
	CIP_PORT |= _BV( CIP_BIT);				// Activate pull-up
		
	_delay_us(1);							// Give the pull-ups a second to work	
	
	if ( !watchDogResetFlag )		{		// Are we coming out of anything except for a WatchDog reset?
	
		// Cold boot, run test mode

		// Blink back and forth to show LEDs work and solicit a button press	
		

		for(uint8_t i=0;i<100 && !BUTTON_STATE_DOWN(); i++ ) {
	
			setRedLED(255);
	
			for(uint8_t j=0; j<100 && !BUTTON_STATE_DOWN();j++ ) { 
			_delay_ms(1);
			}
		
			setRedLED(0);
			setWhiteLED(255);
		
			for(uint8_t j=0; j<100 && !BUTTON_STATE_DOWN();j++ ) {
			_delay_ms(1);
	}
	
			setWhiteLED(0);
	
			wdt_reset();
			
		}
	
		_delay_ms(BUTTON_DEBOUNCE_TIME_MS);
		
		// TODO: Put more code here for some testing and feedback on initial battery connection at the factory.
			
	}
	
	// Ready to begin normal operation!
	
							
	if (BUTTON_STATE_DOWN())	{		// Possible stuck button?
			
		// If we get here, either we just finished test mode and the button is Still down, in which case
		// we are testing to make sure it goes back up 
			
		// Otherwise we just reset and the button was down when we woke, so likely it is stuck down 
		// and that is what caused the reset. In this case, show the user and then eventually disable the button.
		
		// Each pass of this loop takes ~1 sec.
				
		for( uint16_t t=0; (t <= BUTTON_STUCK_TIMEOUT_S) && BUTTON_STATE_DOWN(); t++ ) {
								
		
			// To indicate that we are in a stuck-button sequence, we will blink the white LED
			// at 50% brightness, 1Hz, 10% duty cycle. This looks different than other states 
			// and also minimizes battery usage (the LED pulls 10+mA) since we might be doing this
			// for many minutes
		
			// Start with LED off because it looks better when we are coming in from a watchdog
			// reset because the button was held down for more than 8 secs. Otherwise user
			// sees an odd blink pattern. 
		
			setWhiteLED(0);
						
			// Leave the white LED off for 900 ms or until the button goes up
				
			for( uint8_t l=0; l<90 && BUTTON_STATE_DOWN() ; l++) {
				_delay_ms(10);
			}
		
				
			setWhiteLED(BUTTON_FEEDBACK_BRIGHTNESS);
						
			// Leave the white LED on for 100 ms or until the button goes up
			// Could do this as a single _delay_ms(100) but that might feel un-responsive

									
			for( uint8_t l=0; l<10 && BUTTON_STATE_DOWN() ; l++) {
				_delay_ms(10);
			}
		
			wdt_reset();		

		}
		
		
		// Turn off LED before continuing
		setWhiteLED(0);

		// Debounce the possible button up transition 				
				
		_delay_ms(BUTTON_DEBOUNCE_TIME_MS);		
				
	}
				
	if (BUTTON_STATE_DOWN())	{			// Do we still have a stuck button?
			
		BUTTON_PORT &= ~_BV(BUTTON_BIT);	// Disable pull up to avoid running the battery down
				
		// Do not enable interrupt on button pin change - we will require a charger state change to wake up
		// Since the interrupt is not enabled, the pin will be disconnected during sleep so any floating
		// on it will not waste power.
			
	} else {
				
		// Leave pull-up enabled

		PCMSK1 = _BV(BUTTON_INT);				// Enable interrupt on button pin so we wake on a press
						
		}

	PCMSK0 = _BV(EOC_INT) | _BV(CIP_INT);	// Enable interrupt on change in state-of-charge pin or end-of-charge pin no matter what

	GIMSK |= _BV(PCIE1) | _BV(PCIE0);		// Enable both pin change interrupt vectors (each individual pin was also be enabled above)
		
		// Clear pending interrupt flags. This way we will only get an interrupt if something changes
		// after we read it. There is a race condition where something could change between the flag clear and the
		// reads below, so code should be able to deal with possible redundant interrupt and worst case
	// is that we get woken up an extra time and go back to sleep.	

	GIFR = _BV(PCIF1) | _BV(PCIF0);			// Clear interrupt flags so we will interrupt on any change after now...
																		
	if ( !CIP_STATE_ACTIVE() && !EOC_STATE_ACTIVE()  ) {			// Check if conditions are ALREADY true since we only wake on change....
										
		// Ok, it is bedtime!
												
		set_sleep_mode( SLEEP_MODE_PWR_DOWN );  // Go into deep sleep where only a pin change can wake us.. uses only ~0.1uA!
		
		// GOOD NIGHT!		
						
		// This code disables the Watchdog. Note that we can not use the library wdt_disable() becuase it has a bug
		// that causes intermittent unwanted resets.
		
		// Note interrupts are already clear when we get here, otherwise we would need to worry about getting interrupted between the two following lines
				
		WDTCSR |= _BV(WDCE) | _BV(WDE);		// In the same operation, write a logic one to WDCE and WDE.
											// Note we use OR to preserve the prescaler
		
		WDTCSR = 0;							//	Within the next four clock cycles, in the same operation, write the WDE and WDP bits
											// as desired, but with the WDCE bit cleared.
		
			sleep_enable();							// "To enter any of the three sleep modes, the SE bit in MCUCR must be written to logic one and a SLEEP instruction must be executed."
			sei();                                  // Enable global interrupts. "When using the SEI instruction to enable interrupts, the instruction following SEI will be executed before any pending interrupts." 
			sleep_cpu();							// This must come right after the sei() to avoid race condition
				
			// GOOD MORNING!
			// If we get here, then a button push or change in charger status woke s up....
				
			sleep_disable();						// "To avoid the MCU entering the sleep mode unless it is the programmer?s purpose, it is recommended to write the Sleep Enable (SE) bit to one just before the execution of the SLEEP instruction and to clear it immediately after waking up."
		
			cli();									// We are awake now, and do don't care about interrupts anymore (out interrupt routines don't do anything anyway)						
						
		wdt_enable( WDTO_8S );			// Re-enable watchdog on wake Give ourselves 8 seconds before reboot
	}
		
	// Ok, now we are running!!!
				
	// Motor speed
	uint8_t currentSpeedStep = 0;		// What motor speed setting are we currently on?
    speedStepStruct speedStep;
    
    JACK_ON();			// Enable pull up on power jack. A connected controller will pull down to signal it wants to communicate
		
	while (1)	{		
        
													
		// This main loop runs for as long as the motor is on. 
		// It can be terminated by battery charger change of state, low battery detection, button press back to 0 speed, or long button press
		// All these changes terminate the loop in a reboot. 
				
						
		if (EOC_STATE_ACTIVE())		{		// End of charge?
					
			motorOff();						//Turn motor off in case were running before plug went in
																																				
			setWhiteLED(255);				// White LED full on
			
			
			_delay_ms( JACK_DEBOUNCE_TIME_MS );
			
			while (EOC_STATE_ACTIVE()); 	// White LED on for as long as we are charging....
			// Note that this will watchdog timeout after 8 seconds and reboot us,
			// After which we will immediately fall right back to here and continue to show the white LED
								
			setWhiteLED(0);					// Turn it off now, for instant feedback if unplugged (otherwise it will be on for extra 250ms waiting for watchdog reset)
					
			// Charger unplugged, reboot for goo measure
					
			REBOOT();
															
		}
					
					
		if (CIP_STATE_ACTIVE())		{		// Charging?
					
			motorOff();						//Turn motor off in case were running before plug went in
						
			uint8_t brightness=0;
			int8_t direction=1;
						
			_delay_ms( JACK_DEBOUNCE_TIME_MS );
												
			while (CIP_STATE_ACTIVE())	{	// White LED pulse for as long as we are charging....
							
				setWhiteLED(brightness);
							
				if (brightness==255) {
					
					direction=-1;
				
				} else if (brightness==0) {
				
					direction=1;

				}
				
				brightness+=direction;
				
				_delay_ms(1);		// Slows the speed of the rampping LED
				
				wdt_reset();
				
			}
			
			setWhiteLED(0);					// Turn it off now, for instant feedback if unplugged (otherwise it will be on for extra 250ms waiting for watchdog reset)

			// All done charing, reboot for good measure

			REBOOT();
				
			}			
            
			
			
		uint8_t vccx10 = readVccVoltage();				// Capture the current power supply voltage. This takes ~1ms and will be needed multiple times below
			
		if ( vccx10 <= LOW_BATTERY_VOLTS_COLDx10) {
									
			if ( (currentSpeedStep==0) || ( vccx10 <= LOW_BATTERY_VOLTS_WARMx10) ) {	// Motor off, or running and really low?

				motorOff();
				
				setWhiteLED(0);									// Needed bacuse both LEDs might be on if we are in the middle of a button press

				setRedLED(255);
					
				_delay_ms(LOW_BATTERY_LED_ONTIME_MS);			// Show red LED to user to show low battery
					
				while (BUTTON_STATE_DOWN());					// Wait for button to be released if pressed
																// Will watchdog timeout in 8 seconds if stuff
				setRedLED(0);
					
				REBOOT();
					
				}								
			} 				
				                
			
		uint8_t buttonPressedFlag=0;
			
		if (BUTTON_STATE_DOWN())	{		// Button pushed?
                        				
			setWhiteLED(BUTTON_FEEDBACK_BRIGHTNESS);
				
			_delay_ms(BUTTON_DEBOUNCE_TIME_MS);			// debounce going down...
																															
			if ( currentSpeedStep ==0 ) {				// Special case first press turning on instantly
				
				updateMotor( pgm_read_word(&speedSteps[1].top) , pgm_read_word(&speedSteps[1].normailzedDuty), vccx10);		// Set new motor speed
					
			}
			
			uint16_t buttonDownCount=0;
			
			while (BUTTON_STATE_DOWN()) {			// Wait for button to go back up or longpress timeout
				
				if (buttonDownCount++ >= BUTTON_LONG_PRESS_MS ) {		// Long press? Shut motor off
					
					// The reboot would do both of these anyway, but we do them redundantly here so UI feels responsive-
					// The full reboot cycle takes 100+ ms.
					
					motorOff();
					
					while(BUTTON_STATE_DOWN());		// Wait for the button to be release or watchdog to timeout after 8 secs and reboot us
					
					setWhiteLED(0);
					
					REBOOT();
					
					// If the button is still down once we reboot, we will land in the stuck button detection sequence
					// which will blink the LED for 1/10th second every second until either the button goes up
					// or the Stuck button timeout expires
					
					// Note that REBOOT takes at least 60ms so will effectively debounce the button up event
					
				}
				
				_delay_ms(1);		// One loop=~1ms
								
			}
							
			// Pressed less than a long press
				
			buttonPressedFlag=1;		// Debounce after setting new motor speed so UI feels resposive
				
			currentSpeedStep++;
				
			if (currentSpeedStep >= SPEED_STEP_COUNT )	{ // Cycled all the Way around?
								
				currentSpeedStep=0;
								
			}
            
            speedStep.top               = pgm_read_word(&speedSteps[currentSpeedStep].top);
            speedStep.normailzedDuty    = pgm_read_word(&speedSteps[currentSpeedStep].normailzedDuty);
            
								
		}
								
		updateMotor( speedStep.top , speedStep.normailzedDuty , vccx10);		// Set new motor speed
				
		if (buttonPressedFlag) {
							
			// Button released, white LED off again
				
			setWhiteLED(0);
						
			_delay_ms(BUTTON_DEBOUNCE_TIME_MS);		// debounce the button returning back up
							
				
		}
				
		if (currentSpeedStep==0) {		// Either we stepped though the settings back to off, or we got a spuriuous wake up
			REBOOT();	
			}
		

		// Next check for a request on the data jack....			
		if (JACK_STATE_LOW()) {
            
                  			
           	if (readJack( &speedStep )) {
                   
                   currentSpeedStep = 3;        // Skip to the final speed step once a controller is connected
                                                // so we turn off in next button press
               }
                      
           
		}
        
		// If we get to here, then we check for a low battery and had the chance to reboot if we found one,
		// so ok to postpone reset...		
		
		wdt_reset();
								
	}
			
}