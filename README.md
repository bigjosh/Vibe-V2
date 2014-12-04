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
When the device is initially connected to power, it enters a test mode where the LEDs blink back and forth. Test mode is terminated by either a button press or a 10 second timeout, after which the device enters normal off state and waits for a button press. This test mode lets you verify that both LEDs and button are working.


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

