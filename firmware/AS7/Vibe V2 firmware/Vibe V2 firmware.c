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

/*

3/8/7 RC  V3.0 	Change the christmas lights to flash 4 times instead of 100 times.  This is so that the production people can measure the idle current quickly,

3/13/2018

E3_1	Works now with OC1A, PA6-pin7


4/18/2018

E3_2  Works with PCBWAY pcb 1, following works:

BUT1, get button feedback blink.
BUT2,3 get button feedback blink too short.
BUT2,3 can change peak_strength value.  press short works, press long also works.

Motor works only to turn on to the old setting one - update motor routine needs work.

E3_3  Speed settings work.  But TOP value set to hard value 15000.
but1 and 2 works to go through all the strength presets and variable settings, but need refinement of key press feel.
Need to add sleep mode and fine tune on led stuff.
Need to try on new motor.

E3_4 

1.  Charging flashing light works.
2.  EOC solid light works.
3.  Plug in charging motor stops.  Remove charger, unit can be operated.
4.  Need to put in erratic.
5.  Need refinement of key press feel.
Need to add sleep mode and fine tune on led stuff.
Need to try on new motor.

E3_5
1.  Erratic looks like works, but need more refinement.

E3_6 

1.  Erratic looks better, rand applies to strength and sampling period.

4.  Need to check code make sure to have limit checks on everything.
5.  
Need to try on new motor.

E3_7

1.  Charging does not wake up when in sleep mode.
2.  When wake up in sleep mode, it skips to next pattern.

E3_8

1.  In sleep mode, charging light works.



E3_9

Need adjustment of 
Need refinement of key press feel.
Need to add sleep mode and fine tune on led stuff.
2.  When wake up in sleep mode, it skips to next pattern.  bug



E3_10

Janet and Alex eval.  some comments/fixes needed

Pulse 2x faster  make even faster  try 1.5x
Erratic 1.5x slower - chked
Hold down to turn off changed from 2.5 to 1secs then to .75 secs.  -chked
Ramp funny stuff at the bottom not there anymore   chked Alex


E3_11

Ramp add .25 secs hold for top and bottom.  Janet, Alex and Amanda all tested.  -chked

Make hold down strength increase/decrease 2x fastr for steady, per Janet


E3_12

1.  Change set_motor  change back to EVA code for set_motor.  Glitches are fixed when continously changing strength values.

E3_13

1.  Voltage detect stuff
2.  Click to advance waveform:  How to pass the strength setting ?  Ans:  Should pass the %dynamicRange

E3-14

1.  Fixed click to advance waveform.
2.  Fixed erratic so that it the lowest value is not zero, the lowest value is OFFSET_WAVEFORM.    Janet - pls check.


E3_15

1.  Travel mode works according to requirements as discussed with Janet
2.  Turn on and off works according to requirements per Janet.  (Timing diagram)
3.  When turn on return to previous wave and strength.

E3_16

Changed some variables to const int or const uint_8


E3_17

led low battery COLD 3.47, WARM 3.16  (conditions, Eva battery

E3_18

Review with Janet
Wave increase speed a little.
Erratic slower.
Pulse faster

FIXed ERRATIC STRENGTH NOT INCREASING WHILE HOLDING DOWN

Fixed the extra blink for transit lockout.

E3_19 BETA2
Put in a check against spurious interrupts cause be reverse polarity insertion of charging cable.
Symptom when insert in reverse quickly and in succession, spurious interrupts can cause unit to turn on.  This happens frequently and reproducibly.

E3_20
Guard against any kind of spurious wakeups.


E3_21
Pulse Only changing while on

E3_22
Put some markers everywhere. Add setDebugA and setDebugB

E3_23
Led intensity while in continuous mode  Goes from charging intensity to button press intensity

E3_24
ButtonFeedbackBrightness be proportional to the current vibration intensity when in discrete increase/decrease mode and also in 
In addition increased the max brightness to 40% duty cycle
Increase charge connector debounce time from 100 to 300 ms.  Changed logic of charge detection to deal with rapid connect disconnect.


E3_25
Problem with charge light not stopping at EOC.  Fix bug of Debug Pin DDR setting.
Vbat Vs PWM Vs Temp Test.
Charging Test

E3_26 

1.  Comment out the low voltage red light code.
2.  Add proportional control pwm vs vbat


E3_27  BETA4.HEX

1.  Put back low voltage detect

E3_28  BETA5.HEX

1.  Add watchdog timeout
2.  Change order of waveforms to STEADY WAVE ERR RAMP PULSE

E3_29

Some possible fixes for esd event.
EMPTY_INTERRUPT( EXT_INT0_vect );
Get rid of DebugA and DebugB
Add ResetPin pullup
REBOOT after christmas lights


E_3_30  BETA7.HEX

Introduce BUTTON1_ONLY() -  if button1 is pressed together with other buttons.  Button1 will not activate any actions.
Remove commented out obsolete code from end of file
Remove various commented out code.

E3_31  TempTest
Adjust Steady top intensity to lower value to reduce temperature

E3_32  Beta8.hex

Change the ChargeLight algorithm so that it is more like EVA.   During auto-recharge the led flashes. 

E3_33  77.hex

77%  786

E3_34s

Steady Max Restore to 1000.
add min, max variables.
Change the ChargeLight algorithm so that it is more like FIN. During auto-recharge the led does not flash.

E3_35

Change steady max percent_g to 924 (87%)

E3_36

Change back steady to 1000value










Loop Times
Pulse 0.98secs




Below TODO list.

Led Intensity Changes   DONE
Activate motor control peak percent proportional to Vcc.
Order of waveforms: Steady, Wave, Err, Ramp, Pulse




Look into power loss/ christmas lights occasional problem.

Ug:  when turn on, accidentally ump to next wave or next next wave  fixed


check for accidental turn on by touch - glitch .....

Need to check code make sure to have limit checks on everything.
Need refinement of key press feel.
Need to add sleep mode and fine tune on led stuff.
Need to try on new motor.

Review all lines with ***RC

*/
     
// CPU speed in Hz. Needed for timing functions.
// This is the default fuse setting and works fine for PWM frequencies up to about 10Khz (1% lowest duty), or 100KHz (10% lowest duty). 
// This suits the current speed settings, but a high clock might be needed for higher PWM frequencies

#define F_CPU 1000000						// Name used by delay.h 

#include <stdlib.h>
#include <stdio.h>

#include <avr/io.h>


#include <avr/interrupt.h>
#include <util/delay.h>

//#include "defines.h"
//#include "uart.h"


#define CYCLES_PER_S F_CPU					// Better name

#define CYCLES_PER_MS (F_CPU/1000UL)		// More convenient unit


// Outputs

#define WHITE_LED_PORT PORTB		// PB2  OC0A
#define WHITE_LED_DDR DDRB
#define WHITE_LED_BIT 2

#define RED_LED_PORT PORTA			// PA7 OC0B
#define RED_LED_DDR DDRA
#define RED_LED_BIT 7

#define DEBUG_PINA_PORT PORTA			// PA3 Use as debugging output
#define DEBUG_PINA_DDR DDRA
#define DEBUG_PINA_BIT 3

#define DEBUG_PINB_PORT PORTA			// PA3 Use as debugging output
#define DEBUG_PINB_DDR DDRA
#define DEBUG_PINB_BIT 4


// Inputs

#define BUTTON1_PORT PORTB	// PB0, Pin2, PCINT8
#define BUTTON1_PIN	PINB
//#define BUTTON1_DDR  DDRB
#define BUTTON1_BIT	0
#define BUTTON1_INT	PCINT8

#define RESET_PORT PORTB	
#define RESET_PIN PINB
#define RESET_BIT 3

#define BUTTON2_PORT PORTB	// PB1, Pin3, PCINT9
#define BUTTON2_PIN	PINB
//#define BUTTON2_DDR  DDRB
#define BUTTON2_BIT	1
//#define BUTTON2_INT	PCINT9

#define BUTTON3_PORT PORTA		//PA2, Pin 11, PCINT2
#define BUTTON3_PIN	PINA
//#define BUTTON3_DDR  DDRA
#define BUTTON3_BIT	2
//#define BUTTON3_INT	PCINT2



// Is button currently pressed? Pin has a pullup connected to ground though button, so a down reads a 0 on the pin. 
// (Compiles down to a single SBIC instruction)
#define BUTTON3_STATE_DOWN()	((BUTTON3_PIN & _BV(BUTTON3_BIT))==0)
#define BUTTON2_STATE_DOWN()	((BUTTON2_PIN & _BV(BUTTON2_BIT))==0)
#define BUTTON1_STATE_DOWN()	((BUTTON1_PIN & _BV(BUTTON1_BIT))==0)

#define BUTTON1_ONLY()	(BUTTON1_STATE_DOWN() && !BUTTON2_STATE_DOWN() && !BUTTON3_STATE_DOWN())




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

#define V_SENSE_ACTIVE()		((EOC_PIN & _BV(EOC_BIT))==0)

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

#define JACK_DEBOUNCE_TIME_MS 300			// How long to wait for a battery charger state change debounce   RC change from 100 to 300 for magnetic cable

//#define BUTTON_LONG_PRESS_MS 500			// How long to hold down button to be considered a long press rather than a push
											// Long press immediately turns off motor without needing to cycle though remaining speeds
											
#define BUTTON_LONG_PRESS_MS 2000			// How long to hold down button to be considered a long press rather than a push
#define LONG_PRESS_TURNON (750-30)	// 30 is the overhead
#define LONG_PRESS_1SEC (1000-30)	// 30 is the overhead

											
#define BUTTON_TRANSIT_TIMEOUT_S	(10)	// How long does the button need to be held down for to enter transit lockout mode?		
											// The first 8 seconds happens 

// Different cutoffs because the drain of the motor on the battery lowers the voltage, which will recover when the 
// motor is turned off
// RC Comment  Vcc vs Adc count    inverse relationship
//Vcc	Adc count
//2.95  382
//3.00	375
//3.05	369
//3.10	363  **************
//3.15	358
//3.20	352
//3.25	347
//3.30	341  **************
//3.35  336
#define LOT_DEVIATION	8		// Beta2 lot  Fin Attiny25 has a deviation of 8counts from production lot.
#define LOW_BATTERY_ADC_WARM	(363-LOT_DEVIATION)		// RC ADD using adc counts can give better resolution
#define LOW_BATTERY_ADC_COLD	(341-LOT_DEVIATION)

//#define LOW_BATTERY_VOLTS_WARMx10	(31-03)		// Low battery cutoff while running, 3.0 volts for battery less the 0.3V diode drop
//#define LOW_BATTERY_VOLTS_COLDx10	(33-03)		// Low battery cutoff to turn on   , 3.3 volts for battery less the 0.3V diode drop

#define LOW_BATTERY_LED_ONTIME_MS (1000)	// Show low battery by flashing red LED for 1 second

//#define BUTTON_FEEDBACK_BRIGHTNESS 155		// Blink the LEDs when the button is pressed at this brightness
#define BUTTON_FEEDBACK_BRIGHTNESS 75			// Change 227 to 113 to 75

#define MAX_BRIGHTNESS	227						// Janet decided that we can go up to 234 (45%) without getting too yellow.  5/22/08
#define MAX_CHRG_BRIGHTNESS	46					// 137 to 69 to 46
#define BRIGHTNESS_MIN	MAX_CHRG_BRIGHTNESS

#define RANGE_BRIGHTNESS	(MAX_BRIGHTNESS-BRIGHTNESS_MIN)	// When at motor min intensity, led is at MAX_CHRG_BRIGHTNESS.  when at motor max intensity, led is at MAX_BRIGHTNESS
#define CHARGING_RAMP_DELAY 6500		// Slows the speed of the ramping LED 2700		RC  E3_23  2700/137 is roughly the same flashing speed/brightness of the EV2.	


#define REBOOT() 	{wdt_enable( WDTO_30MS);while(1);}		// Use watchdog to reset MPU. Note that this is sometimes used to also debounce button so probably not quicker than 32ms

#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#define PLUS_MINUS_BUTTONS_DOWN()	(BUTTON2_STATE_DOWN() || BUTTON3_STATE_DOWN())

#define SPEED_STEP_COUNT 5

//#define MIN_STRENGTH 70

enum {
	VIBE_OFF = 0,
	MEDIUM,
	WAVE,
	ERRATIC,
	RAMP,
	PULSE,	
NUM_WAVEFORMS};

const char* getModeName[] = {
	"VIBE_OFF",
	"MEDIUM",
	"WAVE",
	"ERRATIC",
	"RAMP",
	"PULSE",
	"NUM_WAVEFORMS"
};

#define TOP_INIT	15

#define MIN_STRENGTH_WAVEFORM 170	// This is the peak to peak strength
#define MAX_STRENGTH_WAVEFORM 850	// Peak to peak of biggest wave
#define MIN_STRENGTH_STEADY 200
#define MAX_STRENGTH_STEADY 925

#define RANGE_STEADY	(MAX_STRENGTH_STEADY - MIN_STRENGTH_STEADY)
#define RANGE_WAVEFORM	(MAX_STRENGTH_WAVEFORM - MIN_STRENGTH_WAVEFORM)

#define OFFSET_STEADY 0
#define OFFSET_WAVEFORM 150  // that is 150
/*
int strength_setting[2][SPEED_STEP_COUNT] = 	{		// this is duty cycle   e.g. 260 = 26%
	{200, 320, 490, 690, 925}, // Janet approved 4/25/2018
	{150+170 , 150+310, 150+450,150+620, 1000} // Last one is MIN_STRENGTH +850   // Janet approved 4/25/2018
//{320 , 460, 600,770, 1000}
};
*/

// 6/6/2018  Temp Test Steady:  Reduce max % to 87 percent.  Change 1000 (92.5%)-> 924 (87%)-> 855 (82%) - 786 (77%) -> 717 (72%)
// Below I changed the representation from above to a percentage of range representation
const int strength_setting[2][SPEED_STEP_COUNT] = 	{		// this is % of range ,166 = 16.6%
	{0, 166, 400, 676, 1000},  // Actual Strength =  %s x RANGE + OFFSET  (but offset for steady and pulse is Zero)
	{0, 206 , 412, 662, 1000}	// Actual Strength = %s x RANGE + OFFSET
};
const int delta_g[NUM_WAVEFORMS][SPEED_STEP_COUNT-1] = 	{		// 5 different wave types, 4 different deltas
	{4, 4, 5, 7},  // Steady
//	{3, 3, 3, 4},	// Wave
//		{2, 2, 2, 3},	// Wave
			{2, 2, 2, 2},	// Wave
	{1, 2, 2, 2},	// Ramp
//	{2, 2, 2, 3},	// Pulse
		{2, 2, 3, 4},	// Pulse
	{13, 12, 15, 19} // Erratic
};


const int range[2] = 	{RANGE_STEADY, RANGE_WAVEFORM} ; // Actual Strength =  %s x RANGE + OFFSET  (but offset for steady and pulse is Zero)
const int offset[2] = 	{OFFSET_STEADY, OFFSET_WAVEFORM} ; // Actual Strength =  %s x RANGE + OFFSET  (but offset for steady and pulse is Zero
const int min_strength[2] = {MIN_STRENGTH_STEADY, MIN_STRENGTH_WAVEFORM};
	


/*
uint16_t strength_setting[2][SPEED_STEP_COUNT] = 	{		INIT_SAMPLEPERIOD_ERRATIC
	{70, 115, 170, 270, 470}, // per Janet use these for straight and pulse  1_13C { 1x, 1.6, 2.4, 3.9, 6.7x}

	{170, 242, 336,457, 615}// per Janet nums for Wave/Ramp	1_13C //{ 1x, 1.5, 2, 2.7, 3.6x }
};

*/

#define INIT_STRENGTH_IDX	2		// set it to the middle strength2x
#define INIT_STRENGTH 400			// put the strength in the middle just to get a buzz

#define INIT_SAMPLEPERIOD 60			// Attiny  75->50 5-07-2018

//#define INIT_SAMPLEPERIOD_ERRATIC	((INIT_SAMPLEPERIOD*3)/2)	// Janet/alex eval   Erratic should be 1.5x slower  E3_10
#define INIT_SAMPLEPERIOD_ERRATIC	200
//#define INIT_SAMPLEPERIOD 100  // 1_13A  Changed from 100 to 200 per Janet



#define LOOP_LENGTH	100

const uint8_t wave[LOOP_LENGTH] = {
	0,0,1,2,4,6,
	9,12,16,20,24,29,34,40,46,52,
59,65,72,80,87,95,102,110,118,126,134,142,150,157,165,172,180,187,194,200,
206,212,218,223,228,232,236,240,243,246,
248,250,251,252,252,252,251,250,248,246,
243,240,236,232,228,223,218,212,206,200,
193,187,180,172,165,157,150,142,134,126,
118,110,102,95,87,80,72,65,58,52,
46,40,34,29,24,20,16,12,9,6,
4,2,1,0,};








void Delay( int16_t period){
	
	for( int i=0; i < period; i++){
		_delay_us(250);		
	}
}

// Associate physical digital i/o with a variable.  Variable
// process_input not used in 1_14
// variable loops to min->max->min
void process_input( uint8_t switchTF, int16_t delta, int* variable, int var_max){
	
	if( switchTF){
		*variable = (*variable+delta);
		if( *variable > var_max ) *variable = 1;
		if( *variable <= 0 ) *variable = var_max;
//		printf("\n\rMODE: %s\n\r", getModeName[*variable]);
	}
}


void process_input1( uint8_t switchTF, int16_t delta, int* variable, int var_max, int var_min){
	
	if( switchTF){
		*variable = (*variable+delta);
		
		if( *variable < var_min ) *variable = var_min;
		if( *variable > var_max ) *variable = var_max;
//		printf("\n\r%d\n\r", *variable);
	}
}
int get_next_higher( int strength, uint8_t waveType ){// returns the new_strength that is higher than current strength
	int i;
	uint8_t idxWaveType = 0;
	int new_strength = 0;

	switch( waveType ){

		case MEDIUM:
		idxWaveType = 0;
		break;
		
		case RAMP:
		idxWaveType = 1;
		break;
		
		case PULSE:
		idxWaveType = 0;
		break;
		
		case WAVE:
		case ERRATIC:
		idxWaveType = 1;
		break;
		default:
		break;
	}
	new_strength = strength_setting[idxWaveType][4];
	
	
	for( i = 1; i<=4; i++ ){
		if( strength < strength_setting[idxWaveType][i]  && strength >= strength_setting[idxWaveType][i-1] ) return strength_setting[idxWaveType][i];
	}
	return new_strength;
	
}

int get_next_lower( int strength, uint8_t waveType ){// returns the new_strength that is higher than current strength

	int i;
	int new_strength = 0;
	uint8_t idxWaveType = 0;


	switch( waveType ){

		case MEDIUM:
		idxWaveType = 0;
		break;
		
		case RAMP:
		idxWaveType = 1;
		break;
		
		case PULSE:
		idxWaveType = 0;
		break;
		
		case ERRATIC:
		case WAVE:
		idxWaveType = 1;
		break;
		default:
		break;
	}
	
	new_strength = strength_setting[idxWaveType][0];

	for( i = 0 ; i<4; i++ ){
		if( strength <= strength_setting[idxWaveType][i+1]  && strength >strength_setting[idxWaveType][i] ) return strength_setting[idxWaveType][i];
	}
	return new_strength;
}
int8_t get_next_lower_idx( int strength, uint8_t waveType ){// returns the new_strength that is higher than current strength

	int8_t i=0;	
	int8_t idxWaveType = 0;

	switch( waveType ){

		case MEDIUM:
		case PULSE:
			idxWaveType = 0;
		break;
		
		case RAMP:
		case ERRATIC:
		case WAVE:
			idxWaveType = 1;
		break;
	
		default:
		break;
	}

	for( i = 0 ; i<4; i++ ){
		if( strength < strength_setting[idxWaveType][i+1]  && strength >=strength_setting[idxWaveType][i] ) return i;
	}	
	
	return 0;
}

#define DIF(x)	(strength - strength_setting[idxWaveType][x])
int get_closest( int strength, uint8_t waveType ){// returns the new_strength that is higher than current strength

	
	
	uint8_t idxWaveType = 0, i=0, closest_i=0;


	switch( waveType ){

		case MEDIUM:
		idxWaveType = 0;
		break;
		
		case RAMP:
		idxWaveType = 1;
		break;
		
		case PULSE:
		idxWaveType = 0;
		break;
		
		case ERRATIC:
		case WAVE:
		idxWaveType = 1;
		break;
		
		default:
		break;
	}
	
	for( i = 0, closest_i = 0; i< 4; i++ ){
		if(  abs(DIF(i+1)) < abs(DIF(closest_i)) ) closest_i = i+1;	
	}	
	
	return strength_setting[idxWaveType][closest_i];	// seed with lowest strength
}

int8_t get_delta(  uint8_t waveType, int strength, uint16_t iLoop ){	// returns the new_strength index that is lower or equal than current strength

	int8_t delta=0, i;
	if( waveType > 0){
		i = get_next_lower_idx( strength, waveType );
		delta = delta_g[waveType-1][i];
	
	//if( (iLoop %2) == 0){
	
		switch( waveType ){

			case MEDIUM:
			//delta = delta_g[waveType-1][i];
			//delta = 5;
			break;
			
			case WAVE:
			//delta = 3;
			break;
		
			case RAMP:
			//delta = 3;
			break;
		
			case PULSE:
				if (iLoop >= LOOP_LENGTH/2) delta = 0;
			 
			break;
		
			
		
			case ERRATIC:
			//delta = 15;
			break;
		
			default:
			break;
		}
	//}
	
	}
	return delta;	// seed with lowest strength
}







//1_15B
//  get the strength from the 2d strength_setting array

int get_strength_limits( int current_speedstep, int minmax){
	
	uint8_t idxWaveType = 0;
	
	switch( current_speedstep ){

		case MEDIUM:
		case PULSE:
			idxWaveType = 0;
			break;
		
		case RAMP:
		case WAVE:
		case ERRATIC:
			idxWaveType = 1;
			break;
		
		default:
			break;
	}
	if( minmax == 0 )	return strength_setting[idxWaveType][0];
	else 							return strength_setting[idxWaveType][4];

}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Update variables strength, waveType based on Control Method
// 

// peak_strength 0-1000, x loop indx
#define GET_PEAK_STRENGTH()	(((long)peak_percent*range[idxWaveType])/1000 + min_strength[idxWaveType] + offset[idxWaveType])  // define this to save typing
#define RAMP_OVERHANG	24
#define MAX_DUTY		1000
uint16_t compute_motor1(uint16_t idx, int peak_percent, uint8_t waveType, uint16_t* samplePeriod, uint16_t vccx100 ){
	
	uint16_t peak_strength = 0;
	int adjusted_peak = 0;
	long y=0, z=0;		// z is an intermediate variable
	uint8_t idxWaveType = 0;
	
	*samplePeriod = INIT_SAMPLEPERIOD;
	//vccx100 = 380;
	
	switch( waveType ){

		case MEDIUM:
		case PULSE:
			idxWaveType = 0;
			break;
		
		case RAMP:
		case WAVE:
		case ERRATIC:
			idxWaveType = 1;
			break;
		
		default:
			break;
	}
	adjusted_peak = peak_percent * 380UL / vccx100;
	switch( waveType ){

		case MEDIUM:// put cap on y LATER
		case PULSE: // put cap on y
		case WAVE:	// put cap on y	
		case ERRATIC:
			peak_strength = ((long)adjusted_peak*range[idxWaveType])/MAX_DUTY + min_strength[idxWaveType] + offset[idxWaveType];
			break;		
		
		case RAMP:
			if(adjusted_peak > MAX_DUTY ) adjusted_peak = MAX_DUTY;
			peak_strength = ((long)adjusted_peak*range[idxWaveType])/MAX_DUTY + min_strength[idxWaveType] + offset[idxWaveType];
			break;
		
		default:
		break;
	}			
	
	switch( waveType ){
		
		case VIBE_OFF:
			y = 0;
			break;
			
		case MEDIUM:
			y = (uint16_t) peak_strength;		
			if( y >  MAX_DUTY ) y = MAX_DUTY;				// E3_26 Clipping
			break;
		
		case RAMP:  // 1_13C	// Big changes for ramp - Add Overhang  E3_11
		//			z = (long) peak_strength*idx ;
//		if(idx == 0 ) setDebugPinA(1);
//		if(idx == 2 ) setDebugPinA(0);		
			
			y = OFFSET_WAVEFORM;	// ( idx   LOOP_LENGTH+12 To LOOP_LENGTH+24
			if(idx >= 0 && idx < LOOP_LENGTH){								// for Ramp, idx can go from 0-LOOP_LENGTH +24
				z =  (((long)peak_strength-OFFSET_WAVEFORM)*idx);
				y =  (z/(LOOP_LENGTH)  )+OFFSET_WAVEFORM; // y = mx
			} else{	
				if( idx < LOOP_LENGTH+(RAMP_OVERHANG/2) ){	
					y =  peak_strength;
//					setDebugPinA(1);
				}
//				else setDebugPinA(0);
			}
			break;
		
		case PULSE:
			z = (LOOP_LENGTH )/2; //    _/^^^\_/^^^\_  ratio low:high is 2:1
			if(idx > z){
				y = 0;
			}
			else{
				
				y =  peak_strength;
			}
			*samplePeriod = INIT_SAMPLEPERIOD/3;	// Alex eval.  make pulse 2x faster  E3_10
			if( y >  MAX_DUTY ) y = MAX_DUTY;				// E3_26 Clipping
		break;
		
		case WAVE:	// 1_13C
			
			y = map(wave[idx], 0, 255, OFFSET_WAVEFORM, peak_strength); // 13C
			if( y >  MAX_DUTY ) y = MAX_DUTY;				// E3_26  Clipping
			break;
		
		case ERRATIC:
			
			y =  (rand() % (peak_strength - OFFSET_WAVEFORM))+ OFFSET_WAVEFORM ;
			*samplePeriod = (uint16_t) (rand() % (INIT_SAMPLEPERIOD_ERRATIC)) + INIT_SAMPLEPERIOD_ERRATIC;  // E3_10 eval
			
			if( y >  MAX_DUTY ) y = MAX_DUTY;				// E3_26 Clipping
			break;
		
		default:
			break;
		
	}
	// Debug output to serialPlot program
	//printf("%d\n\r", ( uint16_t) y);
	return ( uint16_t) y;
	
}



	

 
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

// RC  4-29-2018  If use mode 14, will have glitches, missing pulses when use continuous strength decrease.  Must use mode 15.
// therefor cannnot use ICR1 for TOP 
//Using the ICR1 Register for defining TOP works well when using fixed TOP values. By using
//ICR1, the OCR1A Register is free to be used for generating a PWM output on OC1A. However,
//if the base PWM frequency is actively changed (by changing the TOP value), using the OCR1A
//as TOP is clearly a better choice due to its double buffer feature.
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
uint16_t readVccVoltage(void) {

	ADMUX = 0b00100001;
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);
		
	_delay_ms(1);

	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
	uint8_t low, high;
	uint16_t adc;
	
	
		ADCSRA |= _BV(ADSC);				// Start a conversion
		while( ADCSRA & _BV( ADSC) ) ;		// Wait for 2nd conversion to be ready...	
		low  = ADCL;
		high = ADCH;

		adc = (high << 8) | low;		// 0<= result <=1023
	
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	
	return( adc );	
}

// Set the motor to run at the specified duty cycle and frequency
// The duty cycle is specified at 4.2 volts as a value 0-65535. It is adjusted to scale to the actual voltage. 
// Of course if you specify 100% at 4.2v and only 3.8v is available, then it will just give 100% at the current voltage

void updateMotor( uint16_t top, uint16_t duty, uint8_t vccx10 ) {
			
	unsigned long voltageAdjustedDuty = (((duty * 42UL ) / vccx10) );		// All dutys are normalized to 4.2 volts, so adjust to the current volatge level. Note that is could overflow an uint16 if the voltage is lower than the normal value. 
	
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

			
void updateMotorNew( uint16_t top, uint16_t duty ) {	// duty is 0-1000,  However, if duty > 1000, it will be capped at maximum
	
	uint16_t match;	
	
	if( top <= 65 && duty <= 1500){		
	
		match =  (duty * top);   // duty is 0-1000  1000 is 100%, max top is 65530, 

		top = top* 1000;		// 
		if (match > top ) {		// Battery to low for requested duty, so give it all we've got	
			match = top;	
		} 	
		setMotorPWM( match , top );	
	}
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
//#define BUTTON_FEEDBACK_BRIGHTNESS 100	H:22us L:230us  ie 8.7%,  125-H26us L:226	135- H:34us			period 252  @Vbat 3.97V  duration 100-200ms
// 200 is 44usec   17.5%
// 227 is 50usec   20%
// 137 is 30usec   12%		// 59%/5 = 12%, therefore 137 is the number for MAX_CHRG_BRIGHTNESS
 

// For reference,  EV2 led is 100duty cycle for duration of 125ms -162ms for Button Feedback  100ohm resistor  
// EV2 1.0V- 1.7V across resistor  ~12mA
// EV2 Vbat3.25  Vr =  1V	~10mA
// EV2 Vbat4.08  Vr = 1.7V  ~17mA
// EV2 Notes:  RC V5-0 LED Brightness change 150-150-3  150--------1.2ms/2.03msec       59% verified
// EV2 MAX_CHRG_BRIGHTNESS  150

// EXT 100ms -200ms  @Vbat = 3.94V   Vr0.82V @10ohms  ~82mA .. so roughly 5x EV2 current, so duty should be 1/5x or 20%duty
// If value of 200 =>44usec, then 227=>50usec, i.e. 20%duty cycle.



void setWhiteLED( uint8_t b ) {
	
	if (b==0)	{	// Off
		
		WHITE_LED_PORT &= ~_BV(WHITE_LED_BIT);				// Normal port Output to low 			
		TCCR0A &= ~ ( _BV( COM0A1  ) | _BV( COM0A0 ) );		// Normal port operation, OC0A disconnected (happens to hold true for all modes)
	
	} else {
		
		int16_t c = (int16_t)b*6/9;							// 4/9
		b = (uint8_t) c;
//		b/=5;												// Account for missing current limiting resistor - empirically found
		
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
void setDebugPinA( uint8_t b ) {
/*	
	if (b==0)	{	// Off		
		DEBUG_PINA_PORT &= ~_BV(DEBUG_PINA_BIT);					// Normal port output to low		
	} else {															// Account for missing current limiting resistor - empirically found
		DEBUG_PINA_PORT |= _BV(DEBUG_PINA_BIT);
	}
*/	
}
void setDebugPinB( uint8_t b ) {
/*	
	if (b==0)	{	// Off
		DEBUG_PINB_PORT &= ~_BV(DEBUG_PINB_BIT);					// Normal port output to low
	} else {															// Account for missing current limiting resistor - empirically found
		DEBUG_PINB_PORT |= _BV(DEBUG_PINB_BIT);
	}
*/	
}

void toggleDebugPinB( void ){
	/*
	DEBUG_PINB_PORT ^= _BV(DEBUG_PINB_BIT);
	*/
}
void toggleDebugPinA( void ){
	/*
	DEBUG_PINA_PORT ^= _BV(DEBUG_PINA_PORT);
	*/
}
	



void setLEDsOff() {
	setRedLED(0);
	setWhiteLED(0);
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
	
EMPTY_INTERRUPT( EXT_INT0_vect );



void pulseRedLed( int delay ){
	
	setRedLED(255);
	Delay(delay);
	setRedLED(0);
	Delay(delay);
}
void pulseWhiteLed( int delay ){
	
	setWhiteLED(255);
	Delay(delay);
	setWhiteLED(0);
	Delay(delay);
}


//1_15B
// Check if button is held down for 2secs, if yes reset, if not then return-noaction

#define KEY_REPEAT_DELAY	250  // change from 1200 to 500
#define KEY_REPEAT_RATE		0x17

static int peak_percent_g __attribute__ ((section (".noinit")));
static int waveType_g __attribute__ ((section (".noinit")));

#define MTR_STATUS_OFF()	(TCCR1A == 0)
	
int main(void)
{
	motorInit();				// Initialize the motor port to drive the MOSFET low
	
	uint8_t watchDogResetFlag = MCUSR & _BV(WDRF);		/// Save the watchdog flag
	MCUSR &= ~ _BV( WDRF );		// Clear the watchdog flag
	
	wdt_enable( WDTO_8S );		// Give ourselves 8 seconds before forced reboot
	
	enableTimer0();				// Initialize the timer that also PWMs the LEDs
	
	
	int buttonPushed1 = 0;
	int buttonPushed2 = 0;
	int buttonPushed3 = 0;
	
	
	uint16_t vccx100 = 420;	// init to 4.2 Volts
	uint16_t pulseWidth = 0;	
	uint16_t buttonDownCount=0; 
	uint8_t brightness=0;
	
	int8_t flg_system_running = 0;
	int8_t flg_key_repeat = 0;	
	 	
	uint16_t samplePeriod = 0;
	int max =0, min=0;
	
	uint16_t iLoop;
	int i = 0;
	
	 
	int period = 999;
	 
//	int t= 1, f=0, s= SET, r = RESET;
	
	// Initializations
	
	iLoop = 1;

	samplePeriod = INIT_SAMPLEPERIOD;
	
	
//		usiuartx_init();
//		sei();	// Enable global interrupts

	
	
	WHITE_LED_DDR	|= _BV(WHITE_LED_BIT);		// Pin to output
	RED_LED_DDR		|= _BV(RED_LED_BIT);
//	DEBUG_PINA_DDR	|= _BV(DEBUG_PINA_BIT);		// Fix bug of missing 60mA when charging  E3_25
//	DEBUG_PINB_DDR	|= _BV(DEBUG_PINB_BIT);

	// Button sense pin setup	
	
	BUTTON1_PORT |= _BV(BUTTON1_BIT);		// Enable pull-up for button pin
	BUTTON2_PORT |= _BV(BUTTON2_BIT);		// Enable pull-up for button pin			//RC ADD 2 AND 3
	BUTTON3_PORT |= _BV(BUTTON3_BIT);		// Enable pull-up for button pin
	RESET_PORT |= _BV(RESET_BIT);			// Enable pull-up for button pin
	
	
	// Battery Charger status pin setup
	
	EOC_PORT |= _BV(EOC_BIT);				// Activate pull-up	
	CIP_PORT |= _BV( CIP_BIT);				// Activate pull-up
	
	_delay_us(1);							// Give the pull-ups a second to work	

	
	
	//pulseRedLed(100);  25ms on 25ms off
	
	// CHRISTMAS LIGHTS
	if ( !watchDogResetFlag )		{		// Are we coming out of anything except for a WatchDog reset?
		waveType_g= 1;
		peak_percent_g = strength_setting[0][2];	// 0 for Medium and Pulse, 2 put it right in the center
		
		
		// Cold boot, run test mode
		
		// Blink back and forth to show LEDs work and solicit a button press
		for(uint8_t i=0;i<4 && !BUTTON1_STATE_DOWN(); i++ ) {					// RC Change from i<100 to i<4,  3/8/7			
			setRedLED(MAX_BRIGHTNESS);			
			for(uint8_t j=0; j<100 && !BUTTON1_STATE_DOWN();j++ ) { 
				_delay_ms(1);				
			}			
			setRedLED(0);
			setWhiteLED(255);			
			for(uint8_t j=0; j<100 && !BUTTON1_STATE_DOWN();j++ ) {
				_delay_ms(1);
			}			
			setWhiteLED(0);	
			wdt_reset();				
		}	
		REBOOT();	// E3_29
		_delay_ms(BUTTON_DEBOUNCE_TIME_MS);
							
	}
		
								
	// Ready to begin normal operation!	
	
	
	if (BUTTON1_ONLY())	{		// Possible stuck button?  Pre-TLO flash sequence
		
		for( uint16_t t=0; (t < BUTTON_TRANSIT_TIMEOUT_S) && BUTTON1_ONLY(); t++ ) {		// Pre TLO Flash 10 seconds if button held down
			setWhiteLED(0);			
			for( uint8_t l=0; l<90 && BUTTON1_ONLY() ; l++) {
				_delay_ms(10);
			}
			setWhiteLED(BUTTON_FEEDBACK_BRIGHTNESS);			
			for( uint8_t l=0; l<10 && BUTTON1_ONLY() ; l++) {
				_delay_ms(10);
			}	
			wdt_reset();				
		}
		setWhiteLED(0);		
		_delay_ms(BUTTON_DEBOUNCE_TIME_MS);				
	}
	// TRANSIT LOCKOUT MODE START  ************************************
	// RC Comment Out:  Detect Stuck button, flash tlo leds, prepare to go into tlo code.
	
	if (BUTTON1_ONLY())	{			// Do we still have a stuck button?

		// Indicate we are entering transit mode with a quick double flash of both LEDs		
				
		_delay_ms(900);			// Don't break the nice visual pattern established durring the lockout sequence
		
		
		brightness=255;
		
		while(brightness--) {
			setWhiteLED(brightness);
			_delay_ms( 1000/255);			// Whole sequence will take about 1 sec
		}
		
	
		BUTTON1_PORT &= ~_BV(BUTTON1_BIT);	// Disable pull up to avoid running the battery down. 
		BUTTON2_PORT &= ~_BV(BUTTON2_BIT);	// Disable pull up to avoid running the battery down. 
		BUTTON3_PORT &= ~_BV(BUTTON3_BIT);	// Disable pull up to avoid running the battery down. 
	
		// Do not enable interrupt on button pin change - we will require a charger state change to wake up
		// Since the interrupt is not enabled, the pin will be disconnected during sleep so any floating
		// on it will not waste power.
	
	} else {
	
		// Leave pull-up enabled
	
		PCMSK1 = _BV(BUTTON1_INT);				// Enable interrupt on button pin so we wake on a press
	
	}
	// TRANSIT LOCKOUT MODE END ********************************
	
	

// RC SLEEP CODE 
 
	
	PCMSK0 = _BV(EOC_INT) | _BV(CIP_INT);	// Enable interrupt on change in state-of-charge pin or end-of-charge pin no matter what
		
	GIMSK |= _BV(PCIE1) | _BV(PCIE0);		// Enable both pin change interrupt vectors (each individual pin was also be enabled above)
			
	// Clear pending interrupt flags. This way we will only get an interrupt if something changes
	// after we read it. There is a race condition where something could change between the flag clear and the
	// reads below, so code should be able to deal with possible redundant interrupt and worst case
	// is that we get woken up an extra time and go back to sleep.	
	
	GIFR = _BV(PCIF1) | _BV(PCIF0);			// Clear interrupt flags so we will interrupt on any change after now...
																		
	if ( !CIP_STATE_ACTIVE() && !V_SENSE_ACTIVE()  ) {			// Check if conditions are ALREADY true since we only wake on change....
			
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
			
		sleep_disable();						// "To avoid the MCU entering the sleep mode unless it is the programmer’s purpose, it is recommended to write the Sleep Enable (SE) bit to one just before the execution of the SLEEP instruction and to clear it immediately after waking up."
		
		cli();									// We are awake now, and do don't care about interrupts anymore (out interrupt routines don't do anything anyway)
		
		wdt_enable( WDTO_8S );					// Re-enable watchdog on wake Give ourselves 8 seconds before reboot

		
		// re-enable inputs, just in case.
	
	BUTTON1_PORT |= _BV(BUTTON1_BIT);		// Enable pull-up for button pin
	BUTTON2_PORT |= _BV(BUTTON2_BIT);		// Enable pull-up for button pin			//RC ADD 2 AND 3
	BUTTON3_PORT |= _BV(BUTTON3_BIT);		// Enable pull-up for button pin
	RESET_PORT |= _BV(RESET_BIT);
	
	_delay_us(1); // give time for pull ups to take effect.
	}
	
	// 2 possibilities for waking up:  1.  Button press, 2.  charging.  3 spurious interrupts.
	// CHECK FOR TURN-ON SEQUENCE
		_delay_ms(BUTTON_DEBOUNCE_TIME_MS);		// potential cure for the problem.
		buttonDownCount = 0;
		if( BUTTON1_ONLY() ){
			while ( BUTTON1_ONLY() ) {		// If button down, then wait for button to go back up or longpress timeout
				if (buttonDownCount++ == LONG_PRESS_TURNON ) {// Have to get a Long press to wake.
					setWhiteLED(BUTTON_FEEDBACK_BRIGHTNESS);
					updateMotorNew( 25 , 200);		// a low motor speed
					flg_system_running = 1;			// set this flag to guard against spurious turn-ons.  E3_20

				}
				if (buttonDownCount >= LONG_PRESS_TURNON+LONG_PRESS_1SEC ) {// If button kept being held down, then have to reboot and go to tlo
					motorOff();
					setLEDsOff();
					REBOOT();
				}		
				Delay(4);		// One loop=~1ms				
			}
			if (buttonDownCount < LONG_PRESS_TURNON) REBOOT();	// Finger lifted and less than the TURNON HOLD TIME
		}		
		setWhiteLED(0);	
			
	// Ok, now we are running!!!
	
	// Motor speed
					// What motor speed setting are we currently on?
// This main loop runs for as long as the motor is on.
// It can be terminated by battery charger change of state, low battery detection, button press back to 0 speed, or long button press
// All these changes terminate the loop in a reboot
	while (1)	{		
		

// CHARGE CODE Here.  Same as FIN
		if ( V_SENSE_ACTIVE() ) {			//   charger is connected
			
			motorOff();								// Always turn off motor when charger connected

			_delay_ms( JACK_DEBOUNCE_TIME_MS );		// We might see bouncing as an energized plug is seated in the jack.
			// This just prevents unnecessary blinking of the LED from us rebooting because we thought charging is over when it is Really just a bounce.
			// Probably not need because the filed readCommand would have taken long enough...
			
			
			brightness=0;
			int8_t direction=1;
			
			
			while (  CIP_STATE_ACTIVE() || V_SENSE_ACTIVE() )	{	// Stay here as long as the plug is in. 
				
				// This is slightly complex because we can transition back and forth from CIP to EOC asynchronously, when say either the battery becomes full
				// or when the charger has been sitting connected for long enough that the battery self depletes low enough to trigger a top-off (unlikely)
				
				// We could just let this fall though after each of those transitions, but the tie it takes to reboot would make the LED blink a little and thats ugly.
				
				
				// The effect of the slightly convoluted code below is is make the transitions between CIP and EOC smooth. The little things count - even if no one notices!
				// When CIP is active, the LED will smoothly ramp up and down and up and down.
				// When CIP is not active, the current ramp will continue in the current direction, but once it rises to max value it will stay there as long as !CIP
				
				setWhiteLED(brightness);
				
				if (brightness==0) {
					
					direction=1;
					_delay_ms(100);			// Pause for a second at off because it looks nice and gives the charger IC a moment to see the current drain without any LED load.
					
				} else if (brightness>=MAX_CHRG_BRIGHTNESS) {
					
					if (!CIP_STATE_ACTIVE()) {		// If !CIP, then we are at EOC so smoothly get to full on and then stay there
						direction=0;
						} else {
						direction=-1;
					}
					
				}
				
				brightness += direction;
				
				_delay_us(CHARGING_RAMP_DELAY);		// Slows the speed of the ramping LED		RC  E3_23  This is roughly the same flashing speed of the EV2.
				
				wdt_reset();
				
			}
			
			setWhiteLED(0);					// Turn it off now, for instant feedback if unplugged

			REBOOT();						// Reboot for good measure. Note that this is the ONLY way out once we have detected CIP or high voltage (without valid data),
			
		}			
	// End of CHARGE CODE here.	

		if( flg_system_running != 1)	REBOOT();			// Put this here to guard against spurious turn-ons.  REBOOT clears the flag. E3_20
				
		uint16_t adc_16t = readVccVoltage();				// Capture the current power supply voltage. This takes ~1ms and will be needed multiple times below
		vccx100 =   (110 * 1024UL) / adc_16t;
		// LOW VOLTAGE DETECT ****
		if ( (  adc_16t > LOW_BATTERY_ADC_COLD && MTR_STATUS_OFF() ) ||  ( adc_16t > LOW_BATTERY_ADC_WARM)  ) { 	// adc count and voltage inverse relationship
			
			motorOff();
			
			setWhiteLED(0);									// Needed becuase both LEDs might be on if we are in the middle of a button press
			
			setRedLED(MAX_BRIGHTNESS);
			
			_delay_ms(LOW_BATTERY_LED_ONTIME_MS);			// Show red LED to user to show low battery
				
			while (BUTTON1_STATE_DOWN());					// Wait for button to be released if pressed
																// Will watchdog timeout in 8 seconds if stuff
			setRedLED(0);
						
			REBOOT();				
			
		}
		// LOW VOLTAGE DETECT **** END
		
			
		buttonDownCount=0;
		
		buttonPushed1 = BUTTON1_STATE_DOWN();
		buttonPushed2 = BUTTON2_STATE_DOWN();
		buttonPushed3 = BUTTON3_STATE_DOWN();
							
		uint8_t buttonPressedFlag=0;
		
		if ( buttonPushed1 ||buttonPushed2 || buttonPushed3)	{		//  Any Button pushed?
			
			
			if( buttonPushed1 )	brightness = BUTTON_FEEDBACK_BRIGHTNESS;
			else				brightness = (uint8_t)((peak_percent_g*(unsigned long)RANGE_BRIGHTNESS/1000)+BRIGHTNESS_MIN);
			
			setWhiteLED(brightness);			
			_delay_ms(BUTTON_DEBOUNCE_TIME_MS);			// debounce going down...
			
			//iLoop = 1;  // intialize loop counter each time speed change  RC REVISIT
			//  HOLD DOWN CENTER BUTTON SHUTOFF SEQUENCE
			buttonDownCount=0;	
			
			while (BUTTON1_ONLY()) {			// Wait for button to go back up or longpress timeout
				
				if (buttonDownCount++ >= LONG_PRESS_TURNON  ) {		// Long press? Shut motor off	shut down sequence
					motorOff();
					setLEDsOff();											
					REBOOT();													
				}								
				_delay_ms(1);		// One loop=~1ms			
				
			}				
			//  HOLD DOWN CENTER BUTTON SHUTOFF SEQUENCE  - END
					
			// Pressed less than a long press			
			buttonPressedFlag=1;		// Debounce after setting new motor speed so UI feels responsive	
			
			// NEW CODE
			buttonDownCount = 0;
			flg_key_repeat = 0;
			while ( PLUS_MINUS_BUTTONS_DOWN() && flg_key_repeat == 0 ) {			// Stay in the loop for the KEY DELAY PERIOD
				if (buttonDownCount++ >= KEY_REPEAT_DELAY ) flg_key_repeat = 1;
				_delay_ms(1);
			}
			int8_t delta = 0;
			if ( PLUS_MINUS_BUTTONS_DOWN() && flg_key_repeat == 1 ){
				 iLoop = (iLoop+25) % LOOP_LENGTH;// 25x10msec is about the interval for KEY_REPEAT_DELAY
				 i=0;
			}
			while ( PLUS_MINUS_BUTTONS_DOWN() && flg_key_repeat == 1 ) {			// Process input repeatedly		
			//				if( peak_percent_g == max) setDebugPinB(1);
				 		
				min = get_strength_limits( waveType_g, 0 );
				max = get_strength_limits( waveType_g, 1 );
				
				if( peak_percent_g == 0){ setDebugPinB(0); i=1; }
				if( peak_percent_g > strength_setting[1][i] ){
					toggleDebugPinB();
					i++;
				}
				if(peak_percent_g == strength_setting[1][4]) setDebugPinB(0);

				
				delta = get_delta( waveType_g, peak_percent_g, iLoop);
				process_input1( buttonPushed2, -delta, &peak_percent_g, max, min );
				process_input1( buttonPushed3,  delta, &peak_percent_g, max, min );
				setWhiteLED( (uint8_t)((peak_percent_g*(unsigned long)RANGE_BRIGHTNESS/1000)+BRIGHTNESS_MIN) );  // E3_24

				
				if( waveType_g == RAMP)	iLoop = (iLoop+1) % (LOOP_LENGTH+RAMP_OVERHANG);	// E3_11 Ramp holds at peak for .25 sec and drops to MIN for .25sec, that extra .5 secs is the Overhang
				else					iLoop = (iLoop+1) % LOOP_LENGTH;	
				
				pulseWidth = compute_motor1(iLoop, peak_percent_g, waveType_g, &samplePeriod, vccx100);			
				period = ( (pulseWidth/28) + 14 );
				if( waveType_g != MEDIUM  && waveType_g != PULSE ) {
					period = 25;
				}
				updateMotorNew( period , pulseWidth);		// Set new motor speed							
				//Delay( samplePeriod+10);
				Delay( samplePeriod+6);	// change from +10 to +6 to compensate for E3_24 setWhiteLED delay
				wdt_reset();		// Add this to EXC code, not in orig Vibe code
				
				if( iLoop >=2 && iLoop < 5 ) setDebugPinA(1);
				if( iLoop >= 5 ) setDebugPinA(0);		
			}
			setWhiteLED(0);			
			setDebugPinB(1); 	

			// SHORT PRESS  ********************************************
			if (buttonDownCount < KEY_REPEAT_DELAY ){ 		// SHORT PRESS  ********************************************
				if( buttonPushed1 ){
					process_input( buttonPushed1, 1, &waveType_g, NUM_WAVEFORMS-1 );
					peak_percent_g = get_closest( peak_percent_g, waveType_g );
					iLoop = 1;  // intialize loop counter each time WAVE change  RC REVISIT
				}
				
				if( buttonPushed2 ){
					peak_percent_g = get_next_lower( peak_percent_g, waveType_g );
				}
				if( buttonPushed3 ){
					peak_percent_g = get_next_higher( peak_percent_g, waveType_g );
				}
			}
			// SHORT PRESS  end   ******************************************				
		}	//if ( buttonPushed1 ||buttonPushed2 || buttonPushed3)	{		//  Any Button pushed?
		// Below lines execute each time regardless of whether of not there is button press.
		if (buttonPressedFlag) {  // if ( buttonPushed2 )
			// Button released, white LED off again
			
			setWhiteLED(0);
			
			_delay_ms(BUTTON_DEBOUNCE_TIME_MS);		// debounce the button returning back up
			
			
		}		
			
			if( waveType_g == RAMP)	iLoop = (iLoop+1) % (LOOP_LENGTH+24);	// E3_11 Ramp holds at peak for .25 sec and drops to MIN for .25sec
			else					iLoop = (iLoop+1) % LOOP_LENGTH;	
								
			pulseWidth = compute_motor1(iLoop, peak_percent_g, waveType_g, &samplePeriod, vccx100);
			period = ( (pulseWidth/28) + 14 );
			
			if( waveType_g != MEDIUM  && waveType_g != RAMP ) period = 25;
				
			updateMotorNew( period , pulseWidth);		// Set new motor speed	
			Delay( samplePeriod );	
			if( iLoop >=2 && iLoop < 5 ) setDebugPinA(1);
			if( iLoop >= 5 ) setDebugPinA(0);
		
		// End:  This get executed each time regardless of whether of not there is button press.
		wdt_reset();	// Last line of the while(1) loop


	}
}

/*
		if (V_SENSE_ACTIVE() && !CIP_STATE_ACTIVE())		{		// End of charge?
			
			motorOff();						//Turn motor off in case were running before plug went in			
			setWhiteLED(MAX_CHRG_BRIGHTNESS);				// White LED full on	
			_delay_ms( JACK_DEBOUNCE_TIME_MS );			
			while (V_SENSE_ACTIVE() && !CIP_STATE_ACTIVE()); 	// White LED on for as long as we are charging....			
			setWhiteLED(0);					// Turn it off now, for instant feedback if unplugged (otherwise it will be on for extra 250ms waiting for watchdog reset)			
			// Charger unplugged, reboot for goo measure
									
			REBOOT();	
		}	
	*/	

		// CHARGE CODE Here	  Same as EVA
		
/*		
		if (V_SENSE_ACTIVE() && !CIP_STATE_ACTIVE())		{		// End of charge?
			
			motorOff();						//Turn motor off in case were running before plug went in		
			setWhiteLED(MAX_CHRG_BRIGHTNESS);				// White LED full on		
			_delay_ms( JACK_DEBOUNCE_TIME_MS );
			
			while (V_SENSE_ACTIVE() && !CIP_STATE_ACTIVE()); 	// White LED on for as long as we are charging....
			// Note that this will watchdog timeout after 8 seconds and reboot us,
			// After which we will immediately fall right back to here and continue to show the white LED
			
			setWhiteLED(0);					// Turn it off now, for instant feedback if unplugged (otherwise it will be on for extra 250ms waiting for watchdog reset)			
			// Charger unplugged, reboot for goo measure		
			REBOOT();		
		}			
		if ( CIP_STATE_ACTIVE() )		{		// Charging?			
			motorOff();						//Turn motor off in case were running before plug went in						
			brightness=0;
			int8_t direction=1;
			
			_delay_ms( JACK_DEBOUNCE_TIME_MS );				// debounce the JACK			
			
			while (CIP_STATE_ACTIVE())	{	// White LED pulse for as long as we are charging....				
				setWhiteLED(brightness);				
				if (brightness>=MAX_CHRG_BRIGHTNESS) {					
					direction=-1;					
				} else if (brightness==0) {				
					direction=1;	
					_delay_ms(100);
									
				}				
				brightness+=direction;				
				_delay_us(CHARGING_RAMP_DELAY);		// Slows the speed of the ramping LED		RC  E3_23  This is roughly the same flashing speed of the EV2.
				wdt_reset();		
			}
			
			setWhiteLED(0);		// Turn it off now, for instant feedback if unplugged (otherwise it will be on for extra 250ms waiting for watchdog reset)				
			REBOOT();			// All done charing, reboot for good measure
		}
		
*/		


