VIBE V2 Programming Jig Instructions
=====================================
These jigs let you reprogram the ATTINY84A on the VIBE V2 board. You have full access to flash, EEPROM, and fuses.

There are several jig versions....

- FRONT versions let you program from the front of the board, which is useful for boards that are already installed in shells with battery and motor connectors plugged in.
- BACK versions let you program from the back, which is handy for new boards striaght off the line. 
    
    6 and 10 pin versions differ only in the number of pins on the header. Different AVR programmer models use different numbers of pins.


6 pin front:
![6 pin front](https://raw.githubusercontent.com/bigjosh/Vibe-V2/master/programming%20jigs/photos/DSC07862.JPG "6 pin front")

10 pin back:
![10 pin back](https://raw.githubusercontent.com/bigjosh/Vibe-V2/master/programming%20jigs/photos/DSC07900.JPG "10 pin back")
    
These jigs are completely passive and only route the signals from the header to the test points on the board. 

Set up (Windows with [Sparkfun Pocket Programmer](https://www.sparkfun.com/products/9825))
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
`Unable to connect to USBtiny` message. I've found that grounding yourself with a braclet to a real gorund (not the just ground pin on the programmer) while insterting soves this problem. Note that this does not seem to be a problem with other programmers like the ATMEL Dragon. 
6. Connect a VIBE V2 board to the jig. Video showing the 10-pin backside jig is here...<br>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=tlA72ofPwDw" target="_blank"><img src="http://img.youtube.com/vi/tlA72ofPwDw/0.jpg" 
width="240" height="180" border="10" /></a>
4. Launch a Command Window in the directory with the firmware hex file and enter the following command..<br>
`AVRDUDE -c usbtiny -p84 -U flash:w:vibe10-flash.hex -B 4`<br>
Note that the `-U` and `-B` are case sensitive! 
5. Remove the board and repeat. Note that for subsequent boards, you can just press F3 and then enter to repeat the last command.


