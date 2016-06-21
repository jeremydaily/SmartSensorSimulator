/* CAN bus watch dog. This program listens to a J1939 network and
 *  transmits messages it is looking for. 
 *  
 *  To add a new message to the list, do the following:
 *  1. Add an elapsedMillis counter with a new variable name
 *  2. Add an else if statement to check for the desired ID and then reset the unique milisecond counter
 *  3. Add and if statement to check to see if the time has elapsed. If it has, then send a message.
 * 
 */

#include <FlexCAN.h>

FlexCAN CANbus(250000);
static CAN_message_t txmsg,rxmsg;

//Set up millisecond timers for CANmessages
elapsedMillis ID18FEF100millis;
elapsedMillis ID18FEF121millis;

uint8_t allFs[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  pinMode(7, OUTPUT);
  digitalWrite(7,LOW);
   
  Serial.begin(115200);
  CANbus.begin();
  txmsg.id = 0;
  txmsg.ext = 1; // use extended IDs
 // txmsg.timeout = 0; // disable waiting
  txmsg.len = 8;
  memcpy(txmsg.buf,allFs,8);
  delay(1500);
  Serial.println("J1939 Watchdog. Below are all the received messages");
}

void loop() {
  // check to see what messages are available already and reset their timer if seen.
  
  digitalWrite(13,CANbus.available()); //Flash the Teensy LED if there is a CAN message.
  
  if (CANbus.available()){
    CANbus.read(rxmsg);

    char idChars[9];
    sprintf(idChars,"%08X",rxmsg.id);
    Serial.print(idChars);
    for (int i = 0;i<rxmsg.len;i++){
      char hexChars[5];
      sprintf(hexChars,", %02X",rxmsg.buf[i]);
      Serial.print(hexChars);
    }
    Serial.println();

         if (rxmsg.id == 0x18FEF100) ID18FEF100millis = 0;
   else if (rxmsg.id == 0x18FEF121) ID18FEF121millis = 0;
  }

  //Check to see if enough time has passed befor sending the message.
  if (ID18FEF100millis > 101) {txmsg.id=0x18FEF100; CANbus.write(txmsg);}
  if (ID18FEF121millis > 101) {txmsg.id=0x18FEF121; CANbus.write(txmsg);}

}
