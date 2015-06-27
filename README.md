#Setting up the Synercon SSS toolchain on Windows 7
1. Download and install the Arduino IDE (1.0.6)
http://arduino.cc/download.php?f=/arduino-1.0.6-windows.exe
A copy is also available in the Arduino Tools directory.

2. Clone or download this repository and remember where you save it. Set the Arduino preferences to point to your Arduino Library.

3. Point the Arduino Sketchbook path to the location in which you just saved the SSS libaries. Example: C:\Users\jeremy-daily.UTULSA\Documents\GitHub\SmartSensorSimulator\Arduino Sketch Library

4. Run the installer in avrispmkii_libusb-win32_1.2.1.0.zip to get functionality of an Atmel AVRISP mkII.
http://mightyohm.com/blog/wp-content/uploads/2010/09/avrispmkii_libusb-win32_1.2.1.0.zip

5. Install the cypress USB to UART Drivers. http://www.cypress.com/?rID=63794 
These drivers should install automatically in Windows when an SSS is plugged in.

## Add the Smart Sensor Simulator Board Definitions to Arduino
1. Open the boards.txt file found in C:\Program Files (x86)\Arduino\hardware\arduino\
2. Copy and paste the following code at the bottom of the file and resave:
    ##############################################################

    sssmega.name=SSS Mega2560

    sssmega.upload.protocol=wiring
    sssmega.upload.maximum_size=258048
    sssmega.upload.speed=115200

    sssmega.bootloader.low_fuses=0xBF
    sssmega.bootloader.high_fuses=0xD8
    sssmega.bootloader.extended_fuses=0xFD
    sssmega.bootloader.path=stk500v2
    sssmega.bootloader.file=stk500boot_v2_mega2560.hex
    sssmega.bootloader.unlock_bits=0x3F
    sssmega.bootloader.lock_bits=0x0F

    sssmega.build.mcu=atmega2560
    sssmega.build.f_cpu=16000000L
    sssmega.build.core=arduino
    sssmega.build.variant=sssmega

    ##############################################################

    ssspro.name=SSS Primary

    ssspro.upload.protocol=arduino
    ssspro.upload.maximum_size=30720
    ssspro.upload.speed=57600

    ssspro.bootloader.low_fuses=0xFF
    ssspro.bootloader.high_fuses=0xD8
    ssspro.bootloader.extended_fuses=0x05
    ssspro.bootloader.path=atmega
    ssspro.bootloader.file=ATmegaBOOT_168_atmega328.hex
    ssspro.bootloader.unlock_bits=0x3F
    ssspro.bootloader.lock_bits=0x0F

    ssspro.build.mcu=atmega328p
    ssspro.build.f_cpu=16000000L
    ssspro.build.core=arduino
    ssspro.build.variant=standard

    ##############################################################
    
Adding this section to boards.txt enables a bootloader to be burned with the correct pinouts.

3. Create a new directory called sssmega in the C:\Program Files (x86)\Arduino\hardware\arduino\variants\ directory.
4. Copy the pins_arduino.h file found in the Arduino Tools directory of this repository into the newly created directory. The file C:\Program Files (x86)\Arduino\hardware\arduino\variants\sssmega\pins_arduino.h should now exist. This defines all the pins used by the larger processor on the SSS.


#Programming the SSS

## Setting up the USB ports
1. Plug in the SSS to a Windows computer 
2. The Cypress USB to serial device drivers should automatically install.
  1. LEDs D1, D10, and D5 should be lit on an SSS Rev 10 Board.
3. Download the CyusbUart utility from Cypress.
4. 
## Burning Boot Loaders

Note: this only has to be done once, unless the processors are not taking commands. This is originally done by Synercon before the SSS unit is shipped.
1. Confirm that the SSS Primary shows up under the Arduino IDE Tools -> Boards menu. 
2. Select the SSS Primary board. This loads the fuse bits and boot loader options needed to set up the SSS for programming.
3. Plug in the SSS and Connect a programming pod to the ISCP1 (2x3). 
4. Select Tools -> Burn Bootloader
5. Switch the board definition by selecting the SSS Mega.
6. Move the programming pod to ISCP2 (near the power inlet).
7. Select Tools -> Burn Bootloader
8. If needed, burn the daughterboard boot loader with the SSS mega.

## Compilation
If a compilation fails, be sure to check that you have the right board selected. For example Serial2.begin(9600) will not work on an ATmega328 board. It will work a on the ATmega2560 processor.

##