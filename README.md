Setting up the Synercon SSS toolchain on Window 7-64
1. Download and install Arduino 1.6 IDE
http://arduino.cc/download_handler.php?f=/arduino-1.6.3-windows.exe
A copy is also available in the Arduino Tools directory.
2. Download the latest version of AVRDUDE to enable burning the boot loader with the AVR ISP mkII
http://download.savannah.gnu.org/releases/avrdude/avrdude-6.1-mingw32.zip
2.1. Copy the contents in the AVRDUDE zip file to C:\Users\XXXXXXX\AppData\Roaming\Arduino15\packages\arduino\tools\avrdude\6.0.1-arduino3\bin
3. Set the Arduino preferences to point to your Arduino Library.
3.1 Download the SSS Libraries from https://github.com/SynerconTechnologies/SmartSensorSimulator/Arduino
3.2 Extract the library source to a known location. For example: C:\Users\jeremy-daily.UTULSA\Documents\GitHub\SmartSensorSimulator\Arduino
3.3 Point the Arduino IDE library search path to the location in which you just saved the SSS libaries. Example: C:\Users\jeremy-daily.UTULSA\Documents\GitHub\SmartSensorSimulator\Arduino
4. Run the installer in avrispmkii_libusb-win32_1.2.1.0.zip
http://mightyohm.com/blog/wp-content/uploads/2010/09/avrispmkii_libusb-win32_1.2.1.0.zip
5. Install the cypress USB to UART Drivers. http://www.cypress.com/?rID=63794 
These drivers should install automatically in Windows.

#Programming the SSS

## Setting up the USB ports
1. Plug in the SSS to a Windows computer 
2. The Cypress USB to serial device drivers should automatically install.
  1. LEDs D1, D10, and D5 should be lit on an SSS Rev 10 Board.
3. Download the CyusbUart utility from Cypress.
4. 
## Burning Boot Loaders
(i.e. burning the boot loaders)
Note: this only has to be done once, unless the processors are not taking commands. This is originally done by Synercon before the SSS unit is shipped.
1. Confirm that the SSS Primary shows up under the Arduino IDE Tools -> Boards menu. It should indicate that it is a Synercon AVR Based Board.
2. Select the SSS Primary board. This loads the fuse bits and boot loader options needed to set up the SSS for programming.
3. Plug in the SSS and Connect a programming pod to the ISCP1 (2x3). 
4. Select Tools -> Burn Bootloader
5. Switch the board definition by selecting the SSS Mega.
6. Move the programming pod to ISCP2 (near the power inlet).
7. Select Tools -> Burn Bootloader

