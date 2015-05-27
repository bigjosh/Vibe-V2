#VIBE V2 Programming Jig Instructions

These jigs let you reprogram the ATTINY84A on the VIBE V2 board. You have full access to flash, EEPROM, and fuses.

There are several jig versions....

- FRONT versions let you program from the front of the board, which is useful for boards that are already installed in shells with battery and motor connectors plugged in.
- BACK versions let you program from the back, which is handy for new boards striaght off the line. 
    
    6 and 10 pin versions differ only in the number of pins on the header. Different AVR programmer models use different numbers of pins.


6 pin front:
![6 pin front](photos/DSC07862.JPG "6 pin front")

10 pin back:
![10 pin back](photos/DSC07900.JPG "10 pin back")
    

There are also different versions for spear tio pins and conical pins becuase the pins have different diameters so need different sized holes. 

These jigs are completely passive and only route the signals from the header to the test points on the board. 

Jigs are assembled with 10-24 screws and nuts. 1 1/4" or 1 1/2" screws seem best. Adjust the nuts so that the pogo pins have to correct force against the bottom of the board.

So far my favorate combination is conical pins with a layer 1 board on the top with plastic ramps and little cut peices of a credit card to act as landing pads for the motor and battery connectors. Here is what it looks like...

![Conical with ramps - unloaded](photos/Conical%20ramp%20unloaded.JPG)
![Conical with ramps - loaded](photos/Conical%20ramp%20loaded.JPG)

Note that people who have need to do way more boards than me (1,000's at a time) prefer no ramps, alignment pins, and spear tip pogos. Note that alignment pins are just spear tips pins installed upside down. 

![Spear with Alignment](photos/Spear%20with%20Alignment.JPG)

Besides the jig, you will also need an AVR programmer capable a serial programing and supplying power to the target. 
Almost any AVR programmer can meet the requirements. 

Below are set and use instructions for either an ATMEL Dragon programmer or a USBTINY programmer. Other programmers will be similar to one of these.

#AVR Dragon Programmer

The following instructions show how to setup and use the programming jig with an [ATMEL Dragon board](http://www.atmel.com/tools/avrdragon.aspx) and a Windows 7/8 machine. 

##Set up
1. Follow these instructions to download and install ATMEL Studio...<br>
http://www.atmel.com/products/microcontrollers/avr/start_now.aspx
2. Install a jumper wire from a VCC pin to pin #4 on the JTAG header on the board like this...
![Supply power to target pin jumper](https://wpdotjoshdotcom.files.wordpress.com/2014/12/wpid-20141212_122136-11.jpg)
This will supply power to the target board while programming. More info [here](http://wp.josh.com/2014/12/12/easy-hack-to-allow-an-avr-dragon-to-power-the-target-device-over-the-isp-cable/). 
3. Connect the the programming jig to the Dragon using either a 6 or 10 pin cable (depending on which jig and cables you have).

4. Download the latest firmware image as a hex file from here...<br>
https://github.com/bigjosh/Vibe-V2/releases

##Use
---
1. Connect a VIBE V2 board to the jig. Video showing the 10-pin backside jig is here...<br>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=tlA72ofPwDw" target="_blank"><img src="http://img.youtube.com/vi/tlA72ofPwDw/0.jpg" 
width="240" height="180" border="10" /></a>
4. Launch an ATMEL Studio Command Window (*a shortcut should have been created durring install) and change into the directory with the firmware hex file.
5. Enter the following command ..<br>
`atprogram -t avrdragon -i isp -d attiny84a -cl 250khz program -c -fl  -f Vibe.V2.firmware.V2-2-1.hex --verify`
5. Wait for the programing to complete and check that the verify succeeded. 
5. Remove the board and repeat. Note that for subsequent boards, you can just press F3 and then enter to repeat the last command.

#USBTINY or USBASP programmer 

The following instructions show how to setup and use the programming jig with a USBTINY (like the  [Sparkfun Pocket Programmer](https://www.sparkfun.com/products/9825)) 
or a USBASP (like the [RioRand USBASP](https://www.amazon.com/RioRand-Downloader-Programmer-ATMega8-Support/dp/B00N2FQ770/ref=as_sl_pc_ss_til?tag=joshcom-20&linkCode=w01&linkId=A2ZBKDCQCTFXGWQM&creativeASIN=B00N2FQ770))
and a Windows 7/8 machine. 


##Set up
------
1. Follow these instructions to download and install drivers for the board...<br>
https://learn.adafruit.com/usbtinyisp/drivers
2. Download and install the WinAVR toolchain from here...<br>
http://sourceforge.net/projects/winavr/
3. Download the latest firmware image as a hex file from here...<br>
https://github.com/bigjosh/Vibe-V2/releases

Use
---
1. The Pocket Programmer seems to be sensitive to static electricity and sometimes looses the USB connection 
to the computer when you instert a new board. If you do loose the connection, you will get an 
`Unable to connect` message. I've found that grounding yourself with a braclet to a real gorund (not the just ground pin on the programmer) while insterting soves this problem. Note that this does not seem to be a problem with other programmers like the ATMEL Dragon. 
6. Connect a VIBE V2 board to the jig. Video showing the 10-pin backside jig is here...<br>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=tlA72ofPwDw" target="_blank"><img src="http://img.youtube.com/vi/tlA72ofPwDw/0.jpg" 
width="240" height="180" border="10" /></a>
4. Launch a Command Window in the directory with the firmware hex file and enter the following command..<br>
For USBTINY:
`AVRDUDE -c usbtiny -pt84 -U flash:w:Vibe.V2.firmware.V2-2-1.hex -B 4`<br>
For USBASP:
`AVRDUDE -c usbasp -pt84 -U flash:w:Vibe.V2.firmware.V2-2-1.hex -B 4`<br>
Note that the `-U` and `-B` are case sensitive! 
5. Wait for the programing to complete and check that the verify succeeded. 
5. Remove the board and repeat. Note that for subsequent boards, you can just press F3 and then enter to repeat the last command.



