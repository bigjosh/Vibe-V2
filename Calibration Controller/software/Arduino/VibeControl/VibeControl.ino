//Sample using LiquidCrystal library
#include <LiquidCrystal.h>
#include <util\delay.h>

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
    if (adc_key_in < 520)  return btnLEFT;
    if (adc_key_in < 790)  return btnSELECT;



    return btnNONE;  // when all others fail, return this...
}


// Jack is pulled high by the vibe, so we signal by pulling low

// Each byte starts with a 5ms LOW preamble to allow sync
// Then each bit has a 0.5ms high to signal start of bit
// The then value of the bit should be sampled 1ms later.
// There is a 0.5ms low guard at the end of each bit

#define JACK_PIN (A5)

void setupJack(){
    pinMode( JACK_PIN , INPUT );
    digitalWrite( JACK_PIN , LOW);
}

#define JACK_HIGH     (pinMode( JACK_PIN  , INPUT ))
#define JACK_LOW      (pinMode( JACK_PIN  , OUTPUT ))

#define JACK_READ() (digitalRead( JACK_PIN ))


// Send 16 bits, MSB first
// Return 0=success, 1=VIBE not connected or listening

uint8_t send(uint16_t x) {
    
    JACK_LOW;    // Go low
    
    _delay_ms(10);        // Give plenty of time for receiver to sync
    
    cli();              // Dont interrupt this 'cause timing is critical
    
    for( uint8_t b=0; b<16; b++ ) {
        
        JACK_HIGH;    // Go high to trigger a bit
        
        _delay_us(500);
        
      //  if ( JACK_READ() == 0 ) return(1);    // VIBE is not listening
        
        if ( (x & 0x8000) == 0  ) {
            
            JACK_LOW;  // Go low to send the 0 bit
            
        }
        
        _delay_ms(1);      // keep bit for at least 2 us so receiver sees it.
        
        JACK_LOW;  // Go low to to resync (note that we might already be low if last bit was 0

        _delay_us(500);
        
        x *= 2;
    }
    
    sei();
    
    JACK_HIGH;   // Go high
    
    //_delay_ms(20);    // Plenty of space for resync
    
    return(0);
    
}

// Command 0-15, data full byte

// A packet is a 4 bit command, 8 bit data, 4 bit checksum (checksum is count of 1 bits in command and data)

// Return 0=success, 1=VIBE not connected or listening


uint8_t sendCommand( uint8_t command, uint8_t data ) {
    
    uint8_t checksum=0;
    
    uint16_t payload = (command << 8 ) | data;
    
    // Checksum is the nibbles of the command and data XORed together and then inverted.
    
    checksum =  ( (command) ^ (data) ^ (data>>4) ^ 0x0f ) & 0x0f;
    
    uint16_t packet = (command << 8+4) | (payload << 4 ) | checksum;
    
    return (send( packet ));
    
}

char buff[10];

uint8_t mode=0;    // 0=duty, 1=top

uint16_t duty = 0;

uint16_t top  = 8191;


// Refresh the Screen

void refreshLabels() {
    
    lcd.setCursor(0,0);
    
    if (mode==0) {
        
            lcd.print(" >Duty<   Top   "); // print a simple message
        
        } else {
            
            lcd.print("  Duty   >Top<  "); // print a simple message
    }
    
}

void notConnectedScreen() {
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("VIBE must be on");  // print a simple message
    lcd.setCursor(0,1);
    lcd.print(" and connected "); // print a simple message
    
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
    
    setupJack();
    lcd.begin(16, 2);              // start the library
    
    lcd.clear();    // Clear the "not connected" message
    
    refreshLabels();
    refreshValues();
    
}


void showKeyPress() {
    
    uint16_t a = analogRead(0);
    
    lcd.setCursor(2,1);
    
    digitstring(a);
    lcd.print(buff);
    
    _delay_ms(1000);
    
    return;
}

uint16_t keyDelay=1000;      // start with 1 second delay between ticks

unsigned long nextKeyTime=0;        // Next time key will repeat

uint8_t lastkey=0;

uint16_t velocity = 1;

// Return 0=success, 1=VIBE not connected or listening

uint8_t sendValues() {
    
    if (sendCommand( 0x00 , top >> 8 )) return(1);
    if (sendCommand( 0x01 , top & 0xff)) return(1);
    if (sendCommand( 0x02 , duty >> 8 )) return(1);
    if (sendCommand( 0x03 , duty & 0xff)) return(1);
    if (sendCommand( 0x04 , 0 )) return(1);            // Activate
    return(0);
}

void loop()
{
    /*
    
    if ( JACK_READ() == 0 ) {    // We should see a pull-up from the VIBE if connected
        
        notConnectedScreen();
        
        while ( JACK_READ() == 0 );  // Wait for the connection

        lcd.clear();    // Clear the "not connected" message
        
        refreshLabels();
        refreshValues();
                       
    }
    
    */
    
    
    lcd_key = read_LCD_buttons();  // read the buttons
    
    if (lcd_key == lastkey) {     // Key held down
        
        if (nextKeyTime > millis() ) {      // Still waiting
            
            return;
            
        }
        
        if (keyDelay) {
            
            keyDelay /=2;
            
            
            } else {
            
            if (velocity < 3000) {
                
                
                velocity +=20;
                
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
                sendValues();
                
                //         sendCommand( 0x00 , top >> 8 );
                //         sendCommand( 0x01 , top & 0xff);
                
             } else {
                
                duty=x;
                refreshDuty();            
                sendValues();
                
                //         sendCommand( 0x02 , duty >> 8 );
                //         sendCommand( 0x03 , duty & 0xff);
                
            }
            
        }
        
    }
    
    nextKeyTime = millis() + keyDelay;
    
    lastkey = lcd_key;
    
    delay(50); // debounce button
}