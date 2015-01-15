#VIBE Testing Procedures

##Battery Charger Test

###When: 
This test should be performed on boards after component assembly. 

###Requires:
1. Board
3. A powered charging cable 
4. A testing circuit as pictured below…

<IMG SRC="images/Bat Charger Tester.PNG">

Note that pin #1 on the battery connector is the one closest to the edge of the board. 

Note that this circuit has a constant drain of about 8uA. Disconnect the battery if you plan on storing for longer than a few weeks. 

**IMPORTANT**: The flyback diode test and the battery charger test can be performed on a single physical fixture, 
but the two circuits must have independent power supplies and grounds!

The device should be REJECTED if there is any deviation from the specified response. 

###Steps:
1. Start with an unpowered board (no battery or charger cable connected).
2. Press the button for a second to bleed any residual charge that might be in the capacitors. The red LED may flash for a split second.
3. Connect the testing fixture to the battery connector on the board. The LEDs should begin alternate flashing at about 10Hz when the power is connected. The alternate flashing will timeout in about 20 seconds, so be sure to complete the next step before that.
5. Press the button. The LEDs should stop alternate flashing and the white LED should remain on solid for as long as you hold down the button. The white LED will timeout after about 20 seconds, so be sure to complete the next step before that. 
6. Release the button. The white LED should go out.
7. Connect the charging cable to the jack on the board. The LED should begin to pulse at about 1Hz indicating that it is charging the battery. If the white LED comes on solid, then it is possible the battery is already full, in which case you should try a different (less full) battery. 
8. Watch the battery charge for a few seconds. 
9. Press button S1. The white LED should switch to full brightness on, indicating a full charge. 
10. Release S1, unplug charging cable, remove board from fixture (order unimportant). 

##Flyback Diode Test

###When: 
This test should be performed on boards after component assembly. 

###Requires:
1. Board 
2. A testing circuit as pictured below…

<IMG SRC="images/Flyback Testing Fixture.PNG">

Note that pin #2 on the motor connector is the one closest to the edge of the board. 

**IMPORTANT**: The flyback diode test and the battery charger test can be performed on a single physical fixture, 
but the two circuits must have independent power supplies and grounds!


The device should be REJECTED if there is any deviation from the specified response. 

1. Connect testing fixture to the motor connector on the board. LED1 should NOT be lit. 
3. Press button S1. LED1 should light. 
3. Release S1 and remove board from fixture (order unimportant). 


##Operation and Connections Test 

###When: 
This test should be performed just prior to assembling the board into the plastic housing. 

###Requires:
1. Board
2. Battery
3. Motor

The board should be REJECTED if there is any deviation from the specified response. 

1. Start with an unpowered board (no battery or charger cable connected).
2. Press the button for a second to bleed any residual charge that might be in the capacitors. Either LED may flash for a split second.
3. Plug the battery and motor into the connectors on the board. The motor may start for a split second. The LEDs should begin alternate flashing at about 10Hz. The alternate flashing will timeout in about 20 seconds, so be sure to complete the next step before that.
5. Press the button. The LEDs should stop alternate flashing and the white LED should remain on solid for as long as you hold down the button. The white LED will timeout after about 20 seconds, so be sure to complete the next step before that.
6. Release the button. The white LED should go out.
7. Press and release the button. The white LED should light while the button is depressed and go off when released. The motor should come on at low speed. Note that for each of the following button presses, if you hold the button down for longer than about 0.5 seconds the motor will turn off. In this case, you can start the cycle over again. 
8. Press and release the button. The white LED should come on while the button is down and turn off when released. The motor should switch to medium speed.
9. Press and release the button. The white LED should come on while the button is down and turn off when released. The motor should switch to medium speed.
10. Press and release the button. The white LED should come on while the button is down and turn off when released. The motor should turn off. 

##Final Operation Test 

###When: 
This test should be performed just prior to packaging the assembled device.

###Requires:
1. Assembled Device

The device should be REJECTED if there is any deviation from the specified response. 

1. The LEDs and the motor should be off at the start of the test.
7. Press and release the button. The white LED should light while the button is depressed and go off when released. The motor should come on at low speed. Note that for each of the following button presses, if you hold the button down for longer than about 0.5 seconds the motor will turn off. In this case, you can start the cycle over again. 
8. Press and release the button. The white LED should come on while the button is down and turn off when released. The motor should switch to medium speed.
9. Press and release the button. The white LED should come on while the button is down and turn off when released. The motor should switch to medium speed.
10. Press and release the button. The white LED should come on while the button is down and turn off when released. The motor should turn off. 

