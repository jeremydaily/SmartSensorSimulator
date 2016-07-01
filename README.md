#Setup for Programming on Windows 10
1. Download Arduino 1.6.8 from https://www.arduino.cc/en/Main/OldSoftwareReleases
2. Download the Arduino sketches of interest from the Arduino Sketch Library folder (or you can download the whole repository).
3. Open Arduino, click File -> Preferences (or press Ctrl-comma)
  1. Place the following link into the box labeled Additional Boards Manager URLs:  `https://raw.githubusercontent.com/SynerconTechnologies/SmartSensorSimulator/Arduino-1.6.8-Support/package_synercon_SmartSensorSimulator_index.json`
  2. Press OK.
4. Open the Board Manager by selecting Tools -> Boards -> Board Manager...
5. Scroll to find the entry for Smart Sensor Simulator by Synercon Technologies
6. Click on the entry and press Install. This will install the board definitions and libraries needed to use the Smart Sensor Simulator.


## Programming Notes
The daughter board uses the SSS mega board definition.

## Troubleshooting
1. Be sure 12V power is applied through the Kycon 4-pin connector. The system draws too much current an can affect the USB power source.
2. You can only program the SSS primary (ATmega328) processor using a programmer.
  * The 0.1uF capacitor C5 is missing on the Rev 10 boards so the primary atmega processor does not get reset on the DTR signal. This prevents the SSS from cycling the key switch when the USB cable is plugged in. However, it also prevents programming of the ATmega328 processor by way of the serial. As such, the ISP headers are used for programming the smaller processor. An Arduino as ISP is the preferred method for the latest versions of windows since the AVRISP mkII is no longer sold by ATMEL.   
2. You have to select the right COM port to Upload using USB. Check to see that 2 COM ports are available by looking at the Windows Device Manager. 
  3. The Cypress drivers needed for these connections are Windows certified and should automatically install (it may take a while and multiple things are installed).
  4. LEDs D1, D10, and D5 should be lit on an SSS Rev 10 Board.
4. If a compilation fails, be sure to check that you have the right board selected. For example Serial2.begin(9600) will not work on an ATmega328 board. It will work a on the ATmega2560 processor.


## Source Code
Example source code is in the examples directories for the SSS libraries. These are read-only files and can be modified and saved elsewhere. Commonly the serial number of the SSS will need to match the serial number on the case. 

Commenting the settings for the output with the signal name and wire can be helpful.

##Burning Boot Loaders

Note: this only has to be done once, unless the processors are not taking commands. This is originally done by Synercon before the SSS unit is shipped.

1. Confirm that the SSS Primary shows up under the Arduino IDE Tools -> Boards menu. 
2. Select the SSS Primary board. This loads the fuse bits and boot loader options needed to set up the SSS for programming.
3. Plug in the SSS and Connect a programming pod to the ISCP1 (2x3). 
4. Select Tools -> Burn Bootloader
5. Switch the board definition by selecting the SSS Mega.
6. Move the programming pod to ISCP2 (near the power inlet).
7. Select Tools -> Burn Bootloader
8. If needed, burn the daughterboard boot loader with the SSS mega.


