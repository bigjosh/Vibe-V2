The Calibration Controller helps you find precise values for the VIBE's speed settings. It connects to a specially outfited VIBE device though the power jack so 
settings can be tested interactively under real use conditions. 

<img src="/Calibration Controller/DSC08125.JPG">

#Theory of Operation

Each speed setting on the VIBE has two parameters – the DUTY and the TOP. 

The DUTY controls what percentage of the time the motor is getting power. It ranges from 0 (completely off) to 65535 (always on). So, a value of about 32000 means that the motor will be getting power about ½ the time. The motor is only rated for 3 volts, so best not to run with DUTY set higher than about 40000 for too long or the motor might over heat. NOTE: We could someday add a “turbo” mode that severely over powers the motor for short bursts as long as the busts are short enough that the motor doesn’t get too hot. 

The TOP controls how fast the motor is turned on and off. It ranges from 1 (1 million times per second) to 65535 (about 15 times per second).  As the TOP gets lower, the time between cycles get shorter and so there is less for the power setting. So, for example, with a TOP of 40000 there would probably be thousands of discrete DUTY settings that would be noticeably different whereas with a TOP of 20 there might only be 20 discrete power steps (so at TOP 200, a DUTY of 545 and a DUTY of 722 might actually feel the same).  NOTE: As you scan though the TOP values you will find some that are at frequencies that your ear can actually hear. We can someday use this to make the motor make beeps and sounds. 

#Usage

To use the Remote Control, plug it into the special VIBE though the power port and then turn on the VIBE and select a speed using the button on the unit. 

You can then use the Remote to update the settings for the currently selected speed. The LEFT and RIGHT buttons switch between DUTY and TOP, the UP and DOWN buttons change the value. Holding UP or DOWN will scroll the value with acceleration for big changes. 

For reference, the current values are…

|Speed|DUTY|TOP|
|----|----|----|
|1|3136|8191|
|2|11264|8191|
|3|24640|8191|

Once you find values that you like for a given speed, write down the two numbers since once you switch to another speed or unplug the Remote, there is no easy way to get them back. 

#Construction

The controller is actually just an Arduino Uno with an [LED Keypad Shield](http://www.dfrobot.com/wiki/index.php?title=Arduino_LCD_KeyPad_Shield_%28SKU:_DFR0009%29). The two wires from the VIBE power plug connect...

|Lead| Pin |
|---|---|
|(+) |  A5|
|(-) | GND |

Additionally there is a ~300 ohm pullup resistor from A5 to 5+.

Additionally the trace from the power connector to the battery chanrging chip on the VIBE has to be cut to keep the chip form kicking in and messing up the communications. Hopefully this can be avoided in the future. 


