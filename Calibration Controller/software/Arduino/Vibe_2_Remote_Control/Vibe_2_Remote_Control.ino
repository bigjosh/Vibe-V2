//Sample using LiquidCrystal library
#include <LiquidCrystal.h>
#include <util\delay.h>
/*******************************************************

This program will test the LCD panel and the buttons
Mark Bramwell, July 2010

********************************************************/

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// read the buttons
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 // For V1.1 us this threshold
 
 /*
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 250)  return btnUP; 
 if (adc_key_in < 450)  return btnDOWN; 
 if (adc_key_in < 650)  return btnLEFT; 
 if (adc_key_in < 850)  return btnSELECT;  
*/
 // For V1.0 comment the other threshold and use the one below:

 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 195)  return btnUP; 
 if (adc_key_in < 380)  return btnDOWN; 
 if (adc_key_in < 450)  return btnLEFT; 
 if (adc_key_in < 790)  return btnSELECT;   



 return btnNONE;  // when all others fail, return this...
}


 

// Jack is pulled high by the vibe, so we signal by pulling low

// Each byte starts with a 5ms LOW preamble to allow sync
// Then each bit has a 0.5ms high to signal start of bit
// The then value of the bit should be sampled 1ms later. 
// There is a 0.5ms low guard at the end of each bit

#define JACK_HIGH     (DDRC &= ~_BV(5))    
#define JACK_LOW      (DDRC |= _BV(5) )
 
void send(uint16_t x) {
  

   JACK_LOW;    // Go low
   
   _delay_ms(10);        // Give time for Reciever to sync (2ms to make sure it is waiting)
   
   uint16_t bitmask = (1 << 11);

   cli();              // Dont inetrrupt this 'cuase timing in Critical
   
   while (bitmask) {
     
     bitmask >>= 1;
       
     JACK_HIGH;    // Go high to trigger a bit
     
      _delay_us(500);
          
     if (!(x & bitmask)) {
       
        JACK_LOW;  // Go low to send the 0 bit       
        
     }
   
      _delay_ms(1);      // keep bit for at least 2 us so reciever sees it. 
      
     
      JACK_LOW;  // Go low to to resync (note that we might already be low if last bit was 0

      _delay_us(500);     
  
   }
     
   sei();
 
   JACK_HIGH;   // Go high  
   
   _delay_ms(20);    // Plenty of space for resync
       
}

void sendCommand( uint8_t command, uint8_t data ) {
  
  send( (command << 8 ) | data);
  
}

char buff[10];

uint8_t mode=0;    // 0=duty, 1=top

uint16_t duty = 0;

uint16_t top  = 8191;


// Refresh the Screen

void refreshLabels() {
  
 lcd.setCursor(0,0);
 
 if (mode==0) {
   lcd.setCursor(0,0);
   lcd.print("  =DTY=   TOP   "); // print a simple message
   
 } else {

   lcd.setCursor(0,0);
   lcd.print("   DTY   =TOP=  "); // print a simple message     
 }
 
}


// left padded, 5 digit string

void digitstring( uint16_t x ) {
  
  uint8_t place = 5;
   
  while (place--) {
    
    uint16_t next = x/10;
    
    buff[place] = (x-(next*10)) + '0';
    
    if (!next) {
      
      while (place--) {
        
        buff[place] = ' ';
        
      }
      
      return;
      
    }
    
    x=next;
    
  }
    
  
}

void refreshDuty() {
 lcd.setCursor(2,1); 
  
 digitstring(duty); 
 lcd.print(buff); 
  
}

void refreshTop() {
 
 lcd.setCursor(9,1); 
 
 digitstring(top);
 lcd.print(buff); 
  
}

void refreshValues() {
  refreshDuty();
  refreshTop();
 
}

void setup()
{
  lcd.begin(16, 2);              // start the library
  refreshLabels();
  refreshValues();
  
}

uint16_t keyDelay=1000;      // start with 1 second delay btween ticks

unsigned long nextKeyTime=0;        // Next time key will repeat

uint8_t lastkey=0;
 
uint16_t velocity = 1;     
 
void loop()
{
        
 lcd_key = read_LCD_buttons();  // read the buttons
 
 if (lcd_key == lastkey) {     // Key held down
 
   if (nextKeyTime > millis() ) {      // Still waiting
   
       return;
       
   }
   
   if (keyDelay) {
   
     keyDelay /=2;
     
     
   } else {
     
     if (velocity < 1000) {
       
     
       velocity +=10;
       
     }
     
   }
   
   
 } else {     // Different key
 
   keyDelay = 500;    // Reset delay
   velocity = 1;
   
 }
 

 uint16_t  x;
 
 if (mode) {
   x=top; 
 } else {
   x=duty;
 } 
 
 uint8_t old_mode = mode;
  
 uint16_t old_x = x;
 
 switch (lcd_key)               // depending on which button was pushed, we perform an action
 {
   case btnRIGHT:
     {
     mode = 1;
     break;
     }
     
   case btnLEFT:
     {
     mode = 0;       
     break;
     }
     
   case btnUP:
     {        
     x+=velocity;
     break;
     }
     
   case btnDOWN:
     {
     x-=velocity;
     break;
     }
     
   case btnSELECT:
     {
     break;
     }
 }
 
 
 if ( mode != old_mode ) {
   
   refreshLabels();
   
 } else {      // If mode changed, then change in x irrelevant
      
   if ( x != old_x ) {
     
       if (mode) {
         top=x; 
         refreshTop();
         
         sendCommand( 0x00 , top >> 8 );
         sendCommand( 0x01 , top & 0xff);         
         
       } else {
         duty=x;
         refreshDuty();
         
         sendCommand( 0x02 , duty >> 8 );
         sendCommand( 0x03 , duty & 0xff);         
         
       } 
     
     
   }
   
 }
 
 
 delay(10);    // Debounce
 

 nextKeyTime = millis() + keyDelay;    
 
 lastkey = lcd_key;
 
}
