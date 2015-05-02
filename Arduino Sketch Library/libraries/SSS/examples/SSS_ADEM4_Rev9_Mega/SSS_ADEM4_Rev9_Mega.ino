/*
Copyright 2014 
Synercon Technologies, LLC

Title: SSS Rev 9 Development code for the ATmega2560

There are 78 settings that can be adjusted

Author: Jeremy Daily

Date: 6 Feb 2015

For pin mappings see 
https://github.com/SynerconTechnologies/Arduino/blob/master/SSSmega/PinMappingTable.csv

Sample Commands:

CAN first ID byte

1XXX XXXX = Reset messages 
X1XX XXXX = Sends Command to primary (0x40)
XX1X XXXX = Use second channel
XX0X XXXX = Use first channel (J1939)

so F8 will send an id that starts with x18 to the primary processor after resetting the messages and use the seconf processor.

CAN18FEF1230200aabbccddeeff1122 creates a CAN message with ID x18FEF123 that is broadcast with a period of 200 ms 
  with data  aabbccddeeff1122

If the leading id nibble is a 4, then the CAN message is sent over i2c to the other processor to be transmitted.
eg. CAN413EF1230150AABBCCDDEEFF1122

This command has all zeros for the time period. This is caught and set to a minimum of 10 ms.
CAN413EF1220000AABBCCDDEEFF1122

Type can8 to reset the can messages.

re resets the CAN and wipers to default

setC45 sets U5-3 wiper to 45%

^r Toggles the connection to VCC 

the first nibble of the CAN message tells the SSS where to put the message.
*/

#include <SSS_ADEM4_Rev10_defs.h>
#include <SSS.h>

#include <Wire.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Mcp4261.h>

boolean displayCAN = false;


void setup()
{ 
  
 
   
  Serial.println("Starting Up...");
  Serial.println("Synercon Technologies Smart Sensor Simulator");
  Serial.println("Caterpillar ADEM4");
  Serial.println("Originally written by: Jeremy Daily on 6 Feb 2015");
  Serial.println("Last edited by:_____");
  
 

  }
  
//generate CAN messages:
//Use a leading 5 (= 4 + 1) where 1 is the leading ID bit. Puts this message generation on the other processor 
//By putting the messages on the other processor, it enables the ATmega2560 to run service routines.
//  commandString = "CAN18F0013100990000000000000000"; //Electronic Brake Controller from SA=49; 0100 = 100 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
//
//  commandString = "CAN18F0010B00990000000000000000"; //Electronic Brake Controller from SA=11; 0100 = 100 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
//  
//  commandString = "CAN18FEF11700990000000000000000"; //CCVS from SA=23; 0100 = 100 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
//
//  commandString = "CAN18FEF12100990000000000000000"; //CCVS from SA=33; 0100 = 100 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
//
//  commandString = "CAN18FEF13100990000000000000000"; //CCVS from SA=49; 0100 = 100 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
//
//  commandString = "CAN18E0001900990000000000000000"; //Cab Message 1 CM1  from SA=25; 0100 = 100 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
// 
//  //Use a leading 5 (= 4 + 1) where 1 is the leading ID bit. Puts this message generation on the other processor 
//  commandString = "CAN18E0003100990000000000000000"; //Cab Message 1 CM1  from SA=49; 0100 = 100 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
//
//
//  //Use a leading 4 to put this message generation on the other processor. Zero is the leading ID bit.
//  commandString = "CAN0CF0020300090000000000000000"; // Electronic Transmission Controller 1 (ETC1); 0010 = 10 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
//  
//  //Use a leading 4 to put this message generation on the other processor. Zero is the leading ID bit.
//  commandString = "CAN4CF0050300990000000000000000"; // Electronic Transmission Controller 2 (ETC1); 0099 = 10 ms period
//  commandString.toCharArray(command,32);
//  processCommand(31);
  commandString = "CAN18FEF521100000000000000000000"; // PGN 65269 Ambient Conditions 1000 = 1 second period
  commandString.toCharArray(command,32);
  processCommand(31);

//  delay(50);
  Serial.println("Finished Starting Up... Type a command:");

} //end startup()






void loop(){
  currentMillis = millis();
  ignition = digitalRead(ignitionPin);
  
  //Check for new commands on the serial bus
  if (Serial.available()>0) 
  {
    int nDataBytes = Serial.readBytesUntil(separatorChar,command,99);
    sss.processCommand(nDataBytes);
  }

  
  //Transmit periodic CAN messages that are stored in memory.

 
  if (ignition && numCANmsgs >0 ){
    for (int i = 0; i < numCANmsgs; i++){
      currentMillis = millis();
      if ((currentMillis - previousCANmillis[i]) > CANtxPeriod[i]) { // Do this on time.
        previousCANmillis[i] = currentMillis;
        if (CANchannel[i] == 0) CAN1.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
        else if (CANchannel[i] == 32) CAN3.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
      } //end if
    } // end for
  } // end if
   
  // Set a timestamp when the igition turned on the first time. Once ignition is on, resetting the starTime stops
  if (!ignition) startTime = currentMillis;
   
  if (runOnce && currentMillis-startTime > accelCalibrationDelay) {
    runOnce = !runOnce;
    //AccelCalibtration();
      }
  
  
  if(CAN1Received){
    CAN1Received = false;
    //processCAN1message();
  }
  
  if(CAN3Received){
    CAN3Received = false;
    //processCAN3message();
  }
  
} //end loop()


void processCAN1message(){
  Serial.print("CAN1 RX, ");
  rxId = CAN1.getCanId();                    // Get message ID
  Serial.print("ID: ");
  Serial.print(rxId, HEX);
  CAN1.readMsgBuf(&len, buf);
  Serial.print(", DLC: ");
  Serial.print(len, HEX);
  Serial.print(", CAN message = ");
  for (int i = 0; i < len; i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
void processCAN3message(){
  Serial.print("CAN3 RX, ");
  rxId = CAN1.getCanId();                    // Get message ID
  Serial.print("ID: ");
  Serial.print(rxId, HEX);
  CAN1.readMsgBuf(&len, buf);
  Serial.print(", DLC: ");
  Serial.print(len, HEX);
  Serial.print(", CAN message = ");
  for (int i = 0; i < len; i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}








void MCP2515RX1Int(){
  CAN1Received = true;
}

void MCP2515RX3Int(){
  CAN3Received = true;
}

void printHelp(){
  Serial.println("This is a helper message:");
}
