Vibe V2
=======

Firmware and board files for Dame Products EVA Vibe V2. More product info here...

http://www.dameproducts.com/

Note that this V2 of the board. The older V1 board files are here...

https://github.com/bigjosh/VibeSystem

User Interface
--------------

###Transit Lockout Mode
Device ships in Transit Lockout Mode to prevent inadvertently turning on while in transit. In this mode, the button is completely disconnected and non-functional. To escape from Transit Lockout Mode into normal operating mode, you must connect the unit to the charger.

You can put the device back into Transit Lockout Mode by holding the button down for more than 10 seconds. Both red and white LEDs will quickly flash twice to indicate that Transit Lockout Mode has been activated.  

Note: this mode also protects the unit from over discharging the battery in case the button gets stuck down because something is continuously pressing on it.  

###Normal Operating Mode
Device is normally in a low-power off state.

A long button press (~0.25 sec) goes straight to off state from any speed. 

A short button press steps to the next speed setting upon button release. There are 3 speed settings (low, medium, high) and a 4th button press returns to off state.

When the battery gets low, the motor turns off and the red LED lights for about 1 second. Subsequent button pushes blink the red LED again to indicate that therer is not enough power to turn on. Note that Lithium Polymer batteries recover some voltage while resting, so it is possible to briefly turn the unit back on even after it has automatically turned off from low battery. Doing this repeatedly will lead to shorter and shorter on times until eventually the battery is too depleted to have any recovery left.

Because the battery has reduced voltage under load, there are different cutoff voltages for initial turn on and continuing operation. The battery must be at least 3.3 volts for the motor to start, but once started it will continue to run until the battery drops to 3.1 volts. 

For user feedback, the white LED is lit for about 250ms when the button is pressed. If the button is held down, it the white LED will flash for 100ms every second until either the button is released or <a href="#transit-lockout-mode">Transit Lockout Mode</a> is activated.  

The white LED also indicates charger status: 

* Off means no charger connected
* On solid, full brightness means charger is connected and battery is fully recharged (LED blinks for 62ms once every 8 seconds)
* Slow pulse (1Hz) means charging


Motor is always off while charger connected. Button state is ignored while the charger is connected.

Test Mode
---------
On initial power-up, the devices enters a test mode...

1. You should see the LEDs alternate blinking red and white at about 10Hz. This lets you visually verify both LEDs are working. 
  1. If you don't see anything on power-up, then either both LEDs are broken or there is some worse problem.
  2. If you see a single LED blinking on and off at 10Hz, then the other LED is bad.
  3. If you see the while LED on at 50%, then you probably have a stuck button. 

2. Press the button. The white LED should light solid at 50% brightness for as long as you hold down the button. The white LED should go out when you release the button.
  1. If the LEDs continue to alternate flash after you push the button, then you probably have a bad button or bad connection from the board to the button. 
  2. If the white LED stays lit at 50% brightness after you release the button, then you probably have a stuck button.

Note that if the button is pressed upon initial power-up, then the normal test mode red/white blinking is skipped (because this is typically terminated by a button press).

Note that both above LED indications time out after about 30 seconds to avoid killing the battery. If the button stays down past the timeout, the the unit will eventually enter <a href="#transit-lockout-mode"Transit Lockout Mode</a>. 

Note that test mode only happens on initial power up. The board is very low power so it can continue to operate off the residual charge in the capacitor for a long time (days-months) after the battery is dinconnected. Because of this, you must follow this procedure to re-enter test mode once the device has been powered up (it is not enough just to remove the battery and replace it)... 

1. disconnect all power (unplug the battery, unplug charger)
2. push and hold the button for a second to exhaust all the residual power from the capacitors
3. reconnect power and test mode should start again
 

Features
--------
* Current draw of <0.1uA when idle, so battery self-drain is likely the limiting factor for maximum off time.
* Zero latency button debounce.
* Motor output power is scaled to battery power so motor speed stays constant though battery discharge (at least until there is not enough voltage left to maintain speed). 
* Code size of about 1.5K easily fits into cheap ATTINY24A parts. 
* A full processor WatchDog reset is executed every time the motor is turned off or the charger is unplugged. This hopefully makes the unit more robust to failures.

Connections
-----------
The specified battery is a 160mAh Polymer Li-ion. Note that the battery polarity is swapped compared  to normal! Follow the markings on the board (positive battery wire towards middle of the board). Verify the polarity on the connector because it is non-standard and connecting a battery with reversed polarity will blow a protection diode. 

The specified motor is a vibration type nominally rated for 130mA at 3V.
