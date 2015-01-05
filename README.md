Vibe V2
=======

Firmware and board files for Dame Products EVA Vibe V2. More product info here...

http://www.dameproducts.com/

Note that this V2 of the board. The older V1 board files are here...

https://github.com/bigjosh/VibeSystem

User Interface
--------------
Device is normally in a low-power off state.

Each press of the button steps to the next speed setting. There are 3 speed settings (low, medium, high) and 4th button press returns to off state.

A long button press (~0.25 sec) goes straight to off state from any speed.

Motor turns off when battery voltage drops below ~3.2 volts. The red LED blinks about 0.5 sec when this happens. Subsequent button pushes blink the red LED again to indicate not enough power to turn on. 

The white LED indicates charger status: 

* Off means no charger connected
* On means charger connected and battery fully recharged
* Slow blink (1Hz) means charging
* Fast blink (0.1s on, 0.9s off) means charger in "test mode", which should never happen.

Motor is always off while charger connected. 

Test Mode
---------
On initial power-up, the devices enters a test mode...

1. You should see the LEDs alternate blinking red and white at about 10Hz. This lets you visually verify both LEDs are working. 
  1. If you don't see anything on power-up, then either both LEDs are broken or there is some worse problem.
  2. If you see a single LED blinking on and off at 10Hz, then the other LED is bad.
  3. If you see both LEDs pulsing simultainiously, then the button is stuck or shortted. 
  4. If you only seeone LED pulsing, then you probably have a stuck button *and* a bad LED. 

2. Press the button. 

3. The red and while LEDs simultainiously pulse at about 10Hz for as long as you hold down the button. The LEDs should go out when you release the button.
  1. If both the LEDs continue to pulse after you release the button (or if you haven't even pushed it yet), then you probably have a stuck button. 
  
  
Note that both above LED indications time out after about 30 seconds to avoid killing the battery. 

Features
--------
* Current draw of <0.1uA when idle, so battery self drain is likely the limiting factor for maximum off time.
* Motor slow-start limits maximum acceleration to %1 of full scale per millisecond. This reduces the power glitch on motor start and lowers physical wear and tear on motor. 
* Zero latency button debounce.
* Motor output power is scaled to battery power so motor speed stays constant though battery discharge (at least until there is not enough voltage left to maintain speed). 

Connections
-----------
The specified battery is a 160mAh Polymer Li-ion. Verify the polarity on the connector because it is non-standard and reversed polarity will blow a protection diode. 

The specified motor is a vibration type nominally rated for 130mA at 3V.

TODO
----
* Would like to add special handling for the case where the button is stuck down- if say it is in a drawer with stuff piled on top. Right now, the CPU will goto sleep, but the pull-up resistor on the button will continue to draw about 1mA continuously until the battery is dead. Better to disconnect the resistor and require a power plug to wake up from that state. 

* Smooth LED dimming using Timer0. Right now LEDs are on or off.

* Power plug communication for setting up speeds and maybe reprogramming firmware.

* Easter eggs!

