Vibe V2
=======

Firmware and board files for Dame Products EVA Vibe V2. More product info here...

http://www.dameproducts.com/

Note that this V2 of the board. The older V1 board files are here...

https://github.com/bigjosh/VibeSystem

User Interface
--------------
Device is normally in a low-power off state.

Button presses cycle though the 3 speed settings and then back to off.

A long button press (~0.5 sec) goes striaght to off.

Motor turns off when battery voltage drops below 3.8 volts. The red LED blinks about 0.5 sec when this happens. Subsequent button pushes blink the red LED again to indicate not enough power to turn on. 

The white LED indicates charger status: 

*Off means no charger connected
*On means charger connected and battery fully recharged
*Slow blink (1Hz) means charging
*Fast blink (0.1s on, 0.9s off) means charger in "test mode", which should never happen.

Motor is always off while charger connected. 


Connections
-----------
The specified battery is a 160mAh Polymer Li-ion. Verify the polarity on the connector becuase it is non-standard and reversed polarity will blow a protection diode. 

The specified motor is a vibration type nominally rated for 130mA at 3V.

TODO
----
*Power usage is still ~4uA at idle. Not sure why. Hopefully we can get that down to <0.1uA.

*Would like to add special handling for the case where the button is stuck down- if say it is in a drawer with stuff piled on top. Right now, the CPU will goto sleep, but the pull-up resistor on the button will continue to draw about 1mA continuously until the battery is dead. Better to disconnect the resistor and require a power plug to wake up from that state. 

*Smooth LED dimming using Timer0. Right now LEDs are on or off.

*Power plug communication for setting up speeds and maybe reprogramming firmware.

*Easter eggs!




