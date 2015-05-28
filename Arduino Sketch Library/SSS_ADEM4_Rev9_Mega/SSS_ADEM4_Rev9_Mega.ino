/*
Title: SSS Rev 9 for ADEM4 Smart Sensor Simulators
There are 78 settings that can be adjusted

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

Last used on SSS-ADEM4-1R90004
*/


#include <Wire.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Mcp4261.h>

const long accelCalibrationDelay = 30000;

const int numCommands = 79;
const char decrementChar = '_';
const char incrementChar = '^';
const char separatorChar = ',';

const char commandChar = '*';
const String decrementCommands[numCommands] = {"q","w","e","r","t","y","u","i","a","s","d","f","g","h","j","k","z","x","c","v","b","n","m","o","p","l","`","<",".",">","/","?","[","{","]","}","\\","|","^q","^w","^e","^r","^t","^y","^u","^i","^a","^s","^d","^f","^g","^h","^j","^k","^z","^x","^c","^v","_q","_w","_e","_r","_t","_y","_u","_i","_a","_s","_d","_f","_g","_h","_j","_k","_z","_x","_c","_v","="};
const char incrementCommands[numCommands][2] = {'Q','W','E','R','T','Y','U','I','A','S','D','F','G','H','J','K','Z','X','C','V','B','N','M','O','P','L','`','<','.','>','/','?','[','{',']','}','\\','|','^Q','^W','^E','^R','^T','^Y','^U','^I','^A','^S','^D','^F','^G','^H','^J','^K','^Z','^X','^C','^V','_Q','_W','_E','_R','_T','_Y','_U','_I','_A','_S','_D','_F','_G','_H','_J','_K','_Z','_X','_C','_V','='};
const char headings[numCommands][5]={'U1-P0','U1-P1','U1-P2','U1-P3','U2-P0','U2-P1','U2-P2','U2-P3','U3-P0','U3-P1','U3-P2','U3-P3','U4-P0','U4-P1','U4-P2','U4-P3','U5-P0','U5-P1','U5-P2','U5-P3','VoutA','VoutB','VoutC','VoutD','PWM1','PWM2','19391','19392','CAN21','CAN22','CAN31','CAN32','V2WS1','V2VS2','E2WS1','E2WS2','E2WS3','E2WS4','U1-0V','U1-1V','U1-2V','U1-3V','U2-0V','U2-1V','U2-2V','U2-3V','U3-0V','U3-1V','U3-2V','U3-3V','U4-0V','U4-1V','U4-2V','U4-3V','U5-0V','U5-1V','U5-2V','U5-3V','U1-0G','U1-1G','U1-2G','U1-3G','U2-0G','U2-1G','U2-2G','U2-3G','U3-0G','U3-1G','U3-2G','U3-3G','U4-0G','U4-1G','U4-2G','U4-3G','U5-0G','U5-1G','U5-2G','U5-3G','CAN2P'};

//These are the default settings. Change these according to the research for the specific ECM.
unsigned int defaultSettings[numCommands] = {50,50,50,10,50,00,00,00,50,30,55,50,50,50,50,50,50,07,10,65,2500,2000,3000,2000,50,35,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int settings[numCommands];//       {00,01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18,19,0020,0021,0022,0023,24,25,6,7,8,9,3,1,2,3,4,5,6,7,8,9,4,1,2,3,4,5,6,7,8,9,5,1,2,3,4,5,6,7,8,9,6,1,2,3,4,5,6,7,8,9,7,1,2,3,4,5,6,7}

//set Up PWM 
int PWMPin1 = 8; //PH5
int PWMPin2 = 9; //PH6
int PWMPin4 = 4; //PG5 in Rev 9b
int pwm1 = 0;
int pwm2 = 0;

//Set up DAC values for the MCP4728
int LDACPin = 41;
int DACAddress = 0x61;
int daughterDACAddress = 0x60;

MCP_CAN CAN1(6); // Set CS to PH3
MCP_CAN CAN3(7); // Set CS to PH4
int CAN1Int = 0; // Interrupt 0 is on pin PE4,INT4,Pin D2. See http){//arduino.cc/en/Reference/attachInterrupt
int CAN3Int = 1; // Interrupt 1 is on pin PE5,INT5,Pin D3. See http){//arduino.cc/en/Reference/attachInterrupt
boolean CAN1Received = false;
boolean CAN3Received = false;

//initialize storage for 50 CAN messages to be transmitted.
int numCANmsgs = 0;
byte CANchannel[50];
unsigned long CANIDs[50];
int CANtxPeriod[50];
byte CANmessages[50][8];
unsigned long int previousCANmillis[50];
unsigned long int currentMillis = millis();
unsigned long int previousMillis100 = currentMillis;
unsigned long int previousMillis10;
unsigned long int startTime;

unsigned char len = 0;
unsigned char buf[8];
long unsigned int rxId;

int ignitionPin = 77;

//Set up digital potentiometer pins according to the schematic and pin mapping table
int VccSelectU1_0 = 22;
int VccSelectU1_1 = 23;
int VccSelectU1_2 = 24;
int VccSelectU1_3 = 25;
int GroundSelectU1_0 = 26;
int GroundSelectU1_1 = 27;
int GroundSelectU1_2 = 28;
int GroundSelectU1_3 = 29;

int VccSelectU2_0 = 15;
int VccSelectU2_1 = 14;
int VccSelectU2_2 = 70;
int VccSelectU2_3 = 71;
int GroundSelectU2_0 = 72;
int GroundSelectU2_1 = 73;
int GroundSelectU2_2 = 74;
int GroundSelectU2_3 = 75;

int VccSelectU3_0 = 37;
int VccSelectU3_1 = 36;
int VccSelectU3_2 = 35;
int VccSelectU3_3 = 34;
int GroundSelectU3_0 = 33;
int GroundSelectU3_1 = 32;
int GroundSelectU3_2 = 31;
int GroundSelectU3_3 = 30;

int VccSelectU4_0 = 49;
int VccSelectU4_1 = 48;
int VccSelectU4_2 = 47;
int VccSelectU4_3 = 46;
int GroundSelectU4_0 = 45;
int GroundSelectU4_1 = 44;
int GroundSelectU4_2 = 43;
int GroundSelectU4_3 = 42;

int VccSelectU5_0 = 62;
int VccSelectU5_1 = 63;
int VccSelectU5_2 = 64;
int VccSelectU5_3 = 65;
int GroundSelectU5_0 = 66;
int GroundSelectU5_1 = 67;
int GroundSelectU5_2 = 68;
int GroundSelectU5_3 = 39;

//Define slave select pins for SPI
int CSU1Pin = 81;
int CSU2Pin = 82;
int CSU3Pin = 83;
int CSU4Pin = 38;
int CSU5Pin = 40;

//Resistor Network Selections (pull Up or Pull down)
int E2WS1SelectPin = 10;
int E2WS2SelectPin = 11;
int E2WS3SelectPin = 12;
int E2WS4SelectPin = 13;
int V2WS1SelectPin = 60;
int V2WS2SelectPin = 61;

//CAN Termination Resistor Selections
int J1939Term1Pin = 54;
int J1939Term2Pin = 55;
int CAN2Term1Pin = 56;
int CAN2Term2Pin = 57;
int CAN3Term1Pin = 58;
int CAN3Term2Pin = 59;
int CAN2FrontEnablePin = 84;

float rAB_ohms = 10000.00; // 10k Ohm
MCP4261 Mcp4261_U1 = MCP4261( CSU1Pin, rAB_ohms );
MCP4261 Mcp4261_U2 = MCP4261( CSU2Pin, rAB_ohms );
MCP4261 Mcp4261_U3 = MCP4261( CSU3Pin, rAB_ohms );
MCP4261 Mcp4261_U4 = MCP4261( CSU4Pin, rAB_ohms );
MCP4261 Mcp4261_U5 = MCP4261( CSU5Pin, rAB_ohms );

boolean ignition = false;
boolean runOnce = true;
boolean ssState = true;

char command[100];
char *commandPointer = command;
String commandString;

char value[6];

// 4 bytes for ID, 2 bytes for time period, and 8 bytes for data
char CANmessage[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//CAN Message Structures
char ID[9] ={0,0,0,0,0,0,0,0,0};
char period[5] ={0,0,0,0,0};
char data1[9]={0,0,0,0,0,0,0,0,0};
char data2[9]={0,0,0,0,0,0,0,0,0};

unsigned long IDnumber = 0;
int periodNumber;

boolean validHex;
boolean displayCAN = false;


void setup()
{ 
  
  pinMode(ignitionPin, INPUT);
   
  //Initialize the SPI chip pins
  pinMode(CSU1Pin, OUTPUT);
  pinMode(CSU2Pin, OUTPUT);
  pinMode(CSU3Pin, OUTPUT);
  pinMode(CSU4Pin, OUTPUT);
  pinMode(CSU5Pin, OUTPUT);
  
  digitalWrite(CSU1Pin,HIGH);
  digitalWrite(CSU2Pin,HIGH);
  digitalWrite(CSU3Pin,HIGH);
  digitalWrite(CSU4Pin,HIGH);
  digitalWrite(CSU5Pin,HIGH);
  
 
  // initialize the digital pins.
  pinMode(PWMPin1, OUTPUT);
  pinMode(PWMPin2, OUTPUT);
  pinMode(PWMPin4, OUTPUT);
  pinMode(LDACPin, OUTPUT);

  digitalWrite(LDACPin,HIGH);
  
  //Resistor Network Modes
  pinMode(E2WS1SelectPin, OUTPUT);
  pinMode(E2WS2SelectPin, OUTPUT);
  pinMode(E2WS3SelectPin, OUTPUT);
  pinMode(E2WS4SelectPin, OUTPUT);
  pinMode(V2WS1SelectPin, OUTPUT);
  pinMode(V2WS2SelectPin, OUTPUT);

  //CAN Termination Resistor Modes
  pinMode(J1939Term1Pin, OUTPUT);
  pinMode(J1939Term2Pin, OUTPUT);
  pinMode(CAN2Term1Pin, OUTPUT);
  pinMode(CAN2Term2Pin, OUTPUT);
  pinMode(CAN3Term1Pin, OUTPUT);
  pinMode(CAN3Term2Pin, OUTPUT);
  
  //Initialize Resistor Network Switches
  //LOW ties all these devices to the Return lines.
  digitalWrite(E2WS1SelectPin, LOW);
  digitalWrite(E2WS2SelectPin, LOW);
  digitalWrite(E2WS3SelectPin, LOW);
  digitalWrite(E2WS4SelectPin, LOW);
  digitalWrite(V2WS1SelectPin, LOW);
  digitalWrite(V2WS2SelectPin, LOW);

  //Initialize CAN Termination Switches
  digitalWrite(J1939Term1Pin, LOW);
  digitalWrite(J1939Term2Pin, LOW);
  digitalWrite(CAN2Term1Pin, LOW);
  digitalWrite(CAN2Term2Pin, LOW);
  digitalWrite(CAN3Term1Pin, LOW);
  digitalWrite(CAN3Term2Pin, LOW);
  
  pinMode(GroundSelectU1_0,OUTPUT);
  pinMode(GroundSelectU1_1,OUTPUT);
  pinMode(GroundSelectU1_2,OUTPUT);
  pinMode(GroundSelectU1_3,OUTPUT);
  pinMode(GroundSelectU2_0,OUTPUT);
  pinMode(GroundSelectU2_1,OUTPUT);
  pinMode(GroundSelectU2_2,OUTPUT);
  pinMode(GroundSelectU2_3,OUTPUT);
  pinMode(GroundSelectU3_0,OUTPUT);
  pinMode(GroundSelectU3_1,OUTPUT);
  pinMode(GroundSelectU3_2,OUTPUT);
  pinMode(GroundSelectU3_3,OUTPUT);
  pinMode(GroundSelectU4_0,OUTPUT);
  pinMode(GroundSelectU4_1,OUTPUT);
  pinMode(GroundSelectU4_2,OUTPUT);
  pinMode(GroundSelectU4_3,OUTPUT);
  pinMode(GroundSelectU5_0,OUTPUT);
  pinMode(GroundSelectU5_1,OUTPUT);
  pinMode(GroundSelectU5_2,OUTPUT);
  pinMode(GroundSelectU5_3,OUTPUT);
  
  pinMode(VccSelectU1_0,OUTPUT);
  pinMode(VccSelectU1_1,OUTPUT);
  pinMode(VccSelectU1_2,OUTPUT);
  pinMode(VccSelectU1_3,OUTPUT);
  pinMode(VccSelectU2_0,OUTPUT);
  pinMode(VccSelectU2_1,OUTPUT);
  pinMode(VccSelectU2_2,OUTPUT);
  pinMode(VccSelectU2_3,OUTPUT);
  pinMode(VccSelectU3_0,OUTPUT);
  pinMode(VccSelectU3_1,OUTPUT);
  pinMode(VccSelectU3_2,OUTPUT);
  pinMode(VccSelectU3_3,OUTPUT);
  pinMode(VccSelectU4_0,OUTPUT);
  pinMode(VccSelectU4_1,OUTPUT);
  pinMode(VccSelectU4_2,OUTPUT);
  pinMode(VccSelectU4_3,OUTPUT);
  pinMode(VccSelectU5_0,OUTPUT);
  pinMode(VccSelectU5_1,OUTPUT);
  pinMode(VccSelectU5_2,OUTPUT); 
  pinMode(VccSelectU5_3,OUTPUT);
  
  
  // Set up Serial Connections
  Serial.begin(115200); // Serial to the USB to UART bridge
  delay(50);
  Serial1.begin(115200); //Serial to the Secondary Processor
  delay(50);
  Serial2.begin(9600); //J1708 
  delay(50);
  
  Serial.println("Starting Up...");
  Serial.println("Synercon Technologies test program for the SSS Rev 9.");
  Serial.println("Written by Jeremy Daily on 6 Feb 2015");
  
  Serial.println("Loading Wire Library.");
  Wire.begin();
  Serial.println("Done.");
  
  Serial.println("Setting up CAN1..."); //J1939
  if(CAN1.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN1 init ok!!");
  else Serial.println("CAN0 init fail!!");
  delay(10);
  Serial.println("Setting up CAN3..."); //J1939
  if(CAN3.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN3 init ok!!");
  else Serial.println("CAN3 init fail!!");
  
  pinMode(CAN1Int,INPUT_PULLUP);
  pinMode(CAN3Int,INPUT_PULLUP);
  
  //TODO: This doesn't seem to work. I can't see CAN messages comming in from the other processor.
//  attachInterrupt(CAN1Int,MCP2515RX1Int, FALLING);
//  attachInterrupt(CAN3Int,MCP2515RX3Int, FALLING);
//  
  Mcp4261_U1.scale = 100;
  Mcp4261_U2.scale = 100;
  Mcp4261_U3.scale = 100;
  Mcp4261_U4.scale = 100;
  Mcp4261_U5.scale = 100;

 
  for (int h = 0; h<numCommands; h++){
    settings[h] = defaultSettings[h];
    adjustSetting(h);
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
  
  commandString = "CAN18FEF10b009900000000000000000"; // CCVS message from Brake (ABS) Controller
  commandString.toCharArray(command,32);
  processCommand(31);
  
  commandString = "CAN18F00503009900000000000000000"; // ETC2 message from Transmission Controller
  commandString.toCharArray(command,32);
  processCommand(31);
  
  commandString = "CAN18F0010b009900000000000000000"; // EBC1 message from Brake Controller
  commandString.toCharArray(command,32);
  processCommand(31);
  
  commandString = "CAN18F00121010000000000000000000"; // EBC1 message from Body Controller
  commandString.toCharArray(command,32);
  processCommand(31);
  
  commandString = "CAN18F00131010000000000000000000"; // EBC1 message from Cab  Controller
  commandString.toCharArray(command,32);
  processCommand(31);
  
  commandString = "CAN18F00117010000000000000000000"; // EBC1 message from instrument cluster
  commandString.toCharArray(command,32);
  processCommand(31);
 
  commandString = "CAN18F00128010000000000000000000"; // EBC1 message from Cab display
  commandString.toCharArray(command,32);
  processCommand(31);
  
  commandString = "CAN08FE6E0b0019000000000000000000"; // High Resolution Wheel Speed  message from Brake Controller
  commandString.toCharArray(command,32);
  processCommand(31);
  
//  delay(50);
  Serial.println("Finished Starting Up... Type a command:");

} //end startup()



void buildCANmessage(){
  
  Serial.print("Building CAN Message number ");
  Serial.print(numCANmsgs);

  CANIDs[numCANmsgs]= IDnumber & 0x1fffffff;
  
  Serial.print(", ID: ");
  Serial.print(CANIDs[numCANmsgs],HEX);
  
  CANtxPeriod[numCANmsgs] = periodNumber;
  Serial.print(", Period: ");
  Serial.print((unsigned int)CANtxPeriod[numCANmsgs]);
  
  CANchannel[numCANmsgs]=CANmessage[0] & 0x20; // either 32 or 0
  
  Serial.print(", Channel: ");
  Serial.print(CANchannel[numCANmsgs]);
  Serial.print(", Data: ");
  for (int j = 0; j<8; j++)
    {
      CANmessages[numCANmsgs][j] =  CANmessage[6+j];
      Serial.print(int(CANmessages[numCANmsgs][j]),HEX);
      Serial.print(' ');
    }
  Serial.println();
  numCANmsgs++;
}


void loop(){
  currentMillis = millis();
  ignition = digitalRead(ignitionPin);
  
  //Check for new commands on the serial bus
  if (Serial.available()>0) 
  {
    int nDataBytes = Serial.readBytesUntil(separatorChar,command,99);
    processCommand(nDataBytes);
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




void processCommand(int numDataBytes){
  Serial.print("Command: ");
  for (int t=0;t<numDataBytes;t++) Serial.print(command[t]);
  Serial.println();
  
  //displayCAN
  if (command[0] == '#' ) displayCAN = !displayCAN;
  
  else if (command[0] > 13 && command[0] < 39){
    for (int q = 0; q<numCommands; q++) adjustSetting(q);
  }
  
  //help text
  else if (command[0] == 'h' && command[1] == 'e') printHelp();
  
  
  //reset can message count
  else if (command[0] == 'r' && command[1] == 'e' ){
    numCANmsgs=0;
    for (int h = 0; h<numCommands; h++){
      settings[h] = defaultSettings[h];
      adjustSetting(h);
    } 
  }
  
  //Process CAN Instructions
  else if ( (command[0] == 'c' || command[0]=='C') && (command[1] == 'a' || command[1] == 'A') && (command[2] =='N' || command[2] == 'n')){
    Serial.println("CAN Command. Make first byte on ID = 0xF0 to reset CAN messages.");
    if (command[3]=='8'){
      Serial.print("Sending CAN Reset command.");
      Wire.beginTransmission(4); //Address the primary processor that is listening for 14 bytes
      for (int i=0; i<14;i++) Wire.write(0x80);
      int flag = Wire.endTransmission(); 
      numCANmsgs=0;
    } 
    if (numDataBytes >= 31){
      for (int q=3;q<31;q++){
        validHex=isxdigit(command[q]);
        if (!validHex) break;
      }
      if (validHex){
        
        for (int q= 3;q<11;q++) ID[q-3] = command[q];
        for (int q=11;q<15;q++) period[q-11] = command[q];
        for (int q=15;q<23;q++) data1[q-15] = command[q];
        for (int q=23;q<31;q++) data2[q-23] = command[q];
       
 
        IDnumber = strtoul(ID,0,16);
        Serial.print("ID: ");
        Serial.print(IDnumber,HEX);
        
        CANmessage[0] = (byte)((IDnumber & 0xFF000000) >> 24);
        CANmessage[1] = (byte)((IDnumber & 0xFF0000) >> 16);
        CANmessage[2] = (byte)((IDnumber & 0xFF00) >> 8);
        CANmessage[3] = (byte)((IDnumber & 0xFF));
        
        
        
        periodNumber = (int)(strtoul(period,0,10));
        if (periodNumber < 10) periodNumber = 10; //Don't allow the SSS to flood the bus
        Serial.print(" Period: ");
        Serial.print(periodNumber,DEC);
       

        CANmessage[4]=(byte)((periodNumber & 0x0000FF00) >> 8);
        CANmessage[5]=(byte)((periodNumber & 0x000000FF) >> 0);
        
        unsigned long  firstData = strtoul(data1,0,16);
        unsigned long  secondData = strtoul(data2,0,16);
        Serial.print(" Data: ");
        Serial.print(firstData,HEX);
        Serial.print(secondData,HEX);
        Serial.println();
         
        CANmessage[6]=(byte)((firstData & 0xFF000000) >> 24) ;
        CANmessage[7]=(byte)((firstData & 0x00FF0000) >> 16);
        CANmessage[8]=(byte)((firstData & 0x0000FF00) >> 8);
        CANmessage[9]=(byte)((firstData & 0x000000FF) >> 0);
        CANmessage[10]=(byte)((secondData & 0xFF000000) >> 24);
        CANmessage[11]=(byte)((secondData & 0x00FF0000) >> 16);
        CANmessage[12]=(byte)((secondData & 0x0000FF00) >> 8);
        CANmessage[13]=(byte)((secondData & 0x000000FF) >> 0);
        
        Serial.print("Built CAN message structure: ");
        for (int g=0;g<14;g++) Serial.print((byte)CANmessage[g],HEX);
        Serial.println();
 
        
        if ((CANmessage[0] & 0x40) == 0x40) sendCANi2c(); 
        else buildCANmessage();
        
       }
       else{
         Serial.println("INPUT ERROR: invalid Hex Data.");
         Serial.println(command);
       }
    }
       else {
         Serial.println("INPUT ERROR: Not enough data for creating a CAN message.");
         Serial.println(command);
       }
     
      }
      else if ((command[0] == 's' || command[0]=='S') && (command[1] == 'e' || command[1] == 'E') && (command[2] =='t' || command[2] == 'T')){ 
        int g = lookupIndex(command[3]);
        for (int w=4;w<numDataBytes;w++) value[w-4] = command[w];
        settings[g]=atoi(value);
        adjustSetting(g);
      }

      else if (command[0] =='^'){ 
        int g = lookupIndex(command[1]) + 38;
        settings[g]= !settings[g];
        adjustSetting(g);
      }
      else if (command[0] =='_'){ 
        int g = lookupIndex(command[1]) + 38 + 20;
        settings[g]= !settings[g];
        adjustSetting(g);
      }
      else if (command[0] < 91 && command[0] > 64 ){ //upper case letters
        int b = lookupIndex(command[0]);
        if (b>19 && b<24){
           settings[b]+=100;
           if (settings[b] > 5000) settings[b]=5000;
        } 
        else {
           settings[b]+=10;
           if (settings[b] > 100) settings[b]=100;
        }
        adjustSetting(b);
      }
      else if (command[0] < 123 && command[0] > 96 ){ //Lower case letters
        int d = lookupIndex(command[0]);
        if (d>19 && d<24){
               settings[d]-=100;
               if (settings[d] > 5000) settings[d]=0;
        } 
        else {
           settings[d]-=10;
           if (settings[d] > 100) settings[d]=0;
        }
        adjustSetting(d);
      }
      else if (lookupIndex(command[0]) > 25){
        Serial.print("Command for lookupIndex = ");
        Serial.println(command[0]);
        int ind = lookupIndex(command[0]);
        Serial.print("Result from lookupIndex = ");
        Serial.println(ind);
        settings[ind] = !settings[ind];  
        adjustSetting(ind);
      }
      
      else 
      {
       Serial.println("INPUT ERROR: Command not recognized.");
      }
      
}
  

void sendCANi2c(){
    Wire.beginTransmission(4); //Address the primary processor that is listening for 14 bytes
    for (int i=0; i<14;i++) Wire.write(CANmessage[i]);
    int flag = Wire.endTransmission(); 
    if (flag == 0) Serial.println("Finished writing over i2c to generate a CAN Entry on the other processor. Done with no errors.");
    else
    {
      Serial.print("Sending CAN Entry over i2C Returned error flag of ");
      Serial.println(flag); 
    }
}

void adjustSetting(int i){
 
  Serial.print(i);
  Serial.print(" ");
  Serial.print(decrementCommands[i]);
  Serial.print(" ");
 
  switch (i){
    case 0:
      Mcp4261_U1.wiper0(settings[i]);
      Serial.print("U1-P0 (J16-9): ");
      Serial.println(settings[i]);
      break;
    case 1:
      Mcp4261_U1.wiper1(settings[i]);
      Serial.print("U1-P1 (J16-10): ");
      Serial.println(settings[i]);
      break;
    case 2:
      Mcp4261_U1.wiper2(settings[i]);
      Serial.print("U1-P2 (J16-11): ");
      Serial.println(settings[i]);
      break;
    case 3:
      Mcp4261_U1.wiper3(settings[i]);
      Serial.print("U1-P3 (J16-13)");
      Serial.println(settings[i]);
      break;
    case 4:
      Mcp4261_U2.wiper0(settings[i]);
      Serial.print("U2-P0 (J18-2): ");
      Serial.println(settings[i]);
      break;
    case 5:
      Mcp4261_U2.wiper1(settings[i]);
      Serial.print("U2-P1 (J18-3): ");
      Serial.println(settings[i]);
      break;
    case 6:
      Mcp4261_U2.wiper2(settings[i]);
      Serial.print("U2-P2 (J18-6): ");
      Serial.println(settings[i]);
      break;
    case 7:
      Mcp4261_U2.wiper3(settings[i]);
      Serial.print("U2-P3 (J18-7): ");
      Serial.println(settings[i]);
      break;
    case 8:
      Mcp4261_U3.wiper0(settings[i]);
      Serial.print("U3-P0 (J10-4): ");
      Serial.println(settings[i]);
      break;
    case 9:
      Mcp4261_U3.wiper1(settings[i]);
      Serial.print("U3-P1 (J10-9): ");
      Serial.println(settings[i]);
      break;
    case 10:
      Mcp4261_U3.wiper2(settings[i]);
      Serial.print("U3-P2 (J10-5): ");
      Serial.println(settings[i]);
      break;
    case 11:
      Mcp4261_U3.wiper3(settings[i]);
      Serial.print("U3-P3 (J10-10): ");
      Serial.println(settings[i]);
      break;
    case 12:
      Mcp4261_U4.wiper0(settings[i]);
      Serial.print("U4-P0 (J24-15): ");
      Serial.println(settings[i]);
      break;
    case 13:
      Mcp4261_U4.wiper1(settings[i]);
      Serial.print("U4-P1 (J24-16): ");
      Serial.println(settings[i]);
      break;
    case 14:
      Mcp4261_U4.wiper2(settings[i]);
      Serial.print("U4-P2 (J24-17): ");
      Serial.println(settings[i]);
      break;
    case 15:
      Mcp4261_U4.wiper3(settings[i]);
      Serial.print("U4-P3 (J24-18): ");
      Serial.println(settings[i]);
      break;
    case 16:
      Mcp4261_U5.wiper0(settings[i]);
      Serial.print("U5-P0 (J10-7): ");
      Serial.println(settings[i]);
      break;
    case 17:
      Mcp4261_U5.wiper1(settings[i]);
      Serial.print("U5-P1 (J24-9): ");
      Serial.println(settings[i]);
      break;
    case 18:
      Mcp4261_U5.wiper2(settings[i]);
      Serial.print("U5-P2 (J24-10): ");
      Serial.println(settings[i]);
      break;
    case 19:
      Mcp4261_U5.wiper3(settings[i]);
      Serial.print("U5-P3 (J24-14): ");
      Serial.println(settings[i]);
      break;
    case 20:
      setDAC();
      Serial.print("VoutA (J24-19): ");
      Serial.println(settings[i]);
      break;
    case 21:
      setDAC();
      Serial.print("VoutB (J24-20): ");
      Serial.println(settings[i]);
      break;
    case 22:
      setDAC();
      Serial.print("VoutC (J24-21): ");
      Serial.println(settings[i]);
      break;
    case 23:
      setDAC();
      Serial.print("VoutD (J24-22): ");
      Serial.println(settings[i]);
      break;
    case 24:
      pwm1 = map(settings[i],0,100,0,255);
      analogWrite(PWMPin1,pwm1); //192 = 3.70 volts on PWM1
      Serial.print("PWM1 (J24-23): ");
      Serial.println(settings[i]);
      break;
    case 25:
      pwm2 = map(settings[i],0,100,0,255);
      analogWrite(PWMPin2,pwm2); //192 = 3.70 volts on PWM1
      Serial.print("PWM2 (J24-24): ");
      Serial.println(settings[i]);
      break;
    case 26:
      digitalWrite(J1939Term1Pin,settings[i]);
      Serial.print("J1939 Term 1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 27:
      digitalWrite(J1939Term2Pin,settings[i]);
      Serial.print("J1939 Term 2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 28:
      digitalWrite(CAN2Term1Pin,settings[i]);
      Serial.print("CAN2 Term 1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 29:
      Serial.print("CAN2 Term 2: ");
      Serial.print(settings[i]);
      digitalWrite(CAN2Term2Pin,settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 30:
      digitalWrite(CAN3Term1Pin,settings[i]);
      Serial.print("CAN3 Term 1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 31:
      digitalWrite(CAN3Term2Pin,settings[i]);
      Serial.print("CAN3 Term 2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 32:
      digitalWrite(V2WS1SelectPin,settings[i]);
      Serial.print("V2WS1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Vehicle Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Vehicle Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 33:
      digitalWrite(V2WS2SelectPin,settings[i]);
      Serial.print("V2WS2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Vehicle Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Vehicle Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 34:
      digitalWrite(E2WS1SelectPin,settings[i]);
      Serial.print("E2WS1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 35:
      digitalWrite(E2WS2SelectPin,settings[i]);
      Serial.print("E2WS2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 36:
      digitalWrite(E2WS3SelectPin,settings[i]);
      Serial.print("E2WS3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 37:
      digitalWrite(E2WS4SelectPin,settings[i]);
      Serial.print("E2WS4: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 38:
      digitalWrite(VccSelectU1_0,settings[i]);
      Serial.print("VccSelectU1_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 39:
      digitalWrite(VccSelectU1_1,settings[i]);
      Serial.print("VccSelectU1_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 40:
      digitalWrite(VccSelectU1_2,settings[i]);
      Serial.print("VccSelectU1_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 41:
      digitalWrite(VccSelectU1_3,settings[i]);
      Serial.print("VccSelectU1_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 42:
      digitalWrite(VccSelectU2_0,settings[i]);
      Serial.print("VccSelectU2_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 43:
      digitalWrite(VccSelectU2_1,settings[i]);
      Serial.print("VccSelectU2_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 44:
      digitalWrite(VccSelectU2_2,settings[i]);
      Serial.print("VccSelectU2_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 45:
      digitalWrite(VccSelectU2_3,settings[i]);
      Serial.print("VccSelectU2_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 46:
      digitalWrite(VccSelectU3_0,settings[i]);
      Serial.print("VccSelectU3_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 47:
      digitalWrite(VccSelectU3_1,settings[i]);
      Serial.print("VccSelectU3_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 48:
      digitalWrite(VccSelectU3_2,settings[i]);
      Serial.print("VccSelectU3_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 49:
      digitalWrite(VccSelectU3_3,settings[i]);
      Serial.print("VccSelectU3_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 50:
      digitalWrite(VccSelectU4_0,settings[i]);
      Serial.print("VccSelectU4_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 51:
      digitalWrite(VccSelectU4_1,settings[i]);
      Serial.print("VccSelectU4_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 52:
      digitalWrite(VccSelectU4_2,settings[i]);
      Serial.print("VccSelectU4_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 53:
      digitalWrite(VccSelectU4_3,settings[i]);
      Serial.print("VccSelectU4_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 54:
      digitalWrite(VccSelectU5_0,settings[i]);
      Serial.print("VccSelectU5_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 55:
      digitalWrite(VccSelectU5_1,settings[i]);
      Serial.print("VccSelectU5_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 56:
      digitalWrite(VccSelectU5_2,settings[i]);
      Serial.print("VccSelectU5_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 57:
      digitalWrite(VccSelectU5_3,settings[i]);
      Serial.print("VccSelectU5_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 58:
      digitalWrite(GroundSelectU1_0,settings[i]);
      Serial.print("GroundSelectU1_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 59:
      digitalWrite(GroundSelectU1_1,settings[i]);
      Serial.print("GroundSelectU1_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 60:
      digitalWrite(GroundSelectU1_2,settings[i]);
      Serial.print("GroundSelectU1_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 61:
      digitalWrite(GroundSelectU1_3,settings[i]);
      Serial.print("GroundSelectU1_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 62:
      digitalWrite(GroundSelectU2_0,settings[i]);
      Serial.print("GroundSelectU2_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 63:
      digitalWrite(GroundSelectU2_1,settings[i]);
      Serial.print("GroundSelectU2_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 64:
      digitalWrite(GroundSelectU2_2,settings[i]);
      Serial.print("GroundSelectU2_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 65:
      digitalWrite(GroundSelectU2_3,settings[i]);
      Serial.print("GroundSelectU2_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 66:
      digitalWrite(GroundSelectU3_0,settings[i]);
      Serial.print("GroundSelectU3_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 67:
      digitalWrite(GroundSelectU3_1,settings[i]);
      Serial.print("GroundSelectU3_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 68:
      digitalWrite(GroundSelectU3_2,settings[i]);
      Serial.print("GroundSelectU3_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 69:
      digitalWrite(GroundSelectU3_3,settings[i]);
      Serial.print("GroundSelectU3_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 70:
      digitalWrite(GroundSelectU4_0,settings[i]);
      Serial.print("GroundSelectU4_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 71:
      digitalWrite(GroundSelectU4_1,settings[i]);
      Serial.print("GroundSelectU4_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 72:
      digitalWrite(GroundSelectU4_2,settings[i]);
      Serial.print("GroundSelectU4_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 73:
      digitalWrite(GroundSelectU4_3,settings[i]);
      Serial.print("GroundSelectU4_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 74:
      digitalWrite(GroundSelectU5_0,settings[i]);
      Serial.print("GroundSelectU5_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 75:
      digitalWrite(GroundSelectU5_1,settings[i]);
      Serial.print("GroundSelectU5_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 76:
      digitalWrite(GroundSelectU5_2,settings[i]);
      Serial.print("GroundSelectU5_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 77:
      digitalWrite(GroundSelectU5_3,settings[i]);
      Serial.print("GroundSelectU5_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 78:
      digitalWrite(CAN2FrontEnablePin,settings[i]);
      Serial.print("CAN2 Panel Enable: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Not Connected");
      else if (settings[i] == 1) Serial.println(" = CAN2 on Front Panel Connected");
      else Serial.println(" Value out of bounds.");
      break;
  }
    
}

void MCP2515RX1Int(){
  CAN1Received = true;
}

void MCP2515RX3Int(){
  CAN3Received = true;
}


void setDAC(){  //Settings are in millivolts.
  digitalWrite(LDACPin,LOW);
  for (int j=20; j<24; j++)
  {
    if (settings[j]>5000) settings[j]=5000;
    if (settings[j]<0) settings[j]=0;
  }
    
  int VoutA = map(settings[20],0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutB = map(settings[21],0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutC = map(settings[22],0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutD = map(settings[23],0,3606,0,3000); //0x0ABB = 3.298V on VoutD

  VoutA = constrain(VoutA,0,4095);
  VoutB = constrain(VoutB,0,4095);
  VoutC = constrain(VoutC,0,4095);
  VoutD = constrain(VoutD,0,4095);
  
  Wire.beginTransmission(DACAddress);
  Wire.write(byte(0x50));             
  Wire.write(highByte(VoutA));             
  Wire.write(lowByte(VoutA));             
  Wire.write(highByte(VoutB));             
  Wire.write(lowByte(VoutB));             
  Wire.write(highByte(VoutC));             
  Wire.write(lowByte(VoutC));             
  Wire.write(highByte(VoutD));             
  Wire.write(lowByte(VoutD));             
  int flag = Wire.endTransmission(); 

  if (flag != 0) 
  {
    Serial.print("Setting DAC over I2C returned error flag of ");
    Serial.println(flag); 
  }
  delay(10);
  digitalWrite(LDACPin,HIGH);
}

int lookupIndex(char c){

    if      (c =='q') return 0;
    else if (c =='w') return 1;
    else if (c =='e') return 2;
    else if (c =='r') return 3;
    else if (c =='t') return 4;
    else if (c =='y') return 5;  
    else if (c =='u') return 6;
    else if (c =='i') return 7;
    else if (c =='a') return 8;
    else if (c =='s') return 9;
    else if (c =='d') return 10;
    else if (c =='f') return 11;
    else if (c =='g') return 12;
    else if (c =='h') return 13;
    else if (c =='j') return 14;
    else if (c =='k') return 15;
    else if (c =='z') return 16;
    else if (c =='x') return 17;
    else if (c =='c') return 18;
    else if (c =='v') return 19;
    else if (c =='b') return 20;
    else if (c =='n') return 21;
    else if (c =='m') return 22;
    else if (c =='o') return 23;
    else if (c =='p') return 24;
    else if (c =='l') return 25;
    else if (c =='Q') return 0;
    else if (c =='W') return 1;
    else if (c =='E') return 2;
    else if (c =='R') return 3;
    else if (c =='T') return 4;
    else if (c =='Y') return 5;  
    else if (c =='U') return 6;
    else if (c =='I') return 7;
    else if (c =='A') return 8;
    else if (c =='S') return 9;
    else if (c =='D') return 10;
    else if (c =='F') return 11;
    else if (c =='G') return 12;
    else if (c =='H') return 13;
    else if (c =='J') return 14;
    else if (c =='K') return 15;
    else if (c =='Z') return 16;
    else if (c =='X') return 17;
    else if (c =='C') return 18;
    else if (c =='V') return 19;
    else if (c =='B') return 20;
    else if (c =='N') return 21;
    else if (c =='M') return 22;
    else if (c =='O') return 23;
    else if (c =='P') return 24;
    else if (c =='L') return 25;
    else if (c == '`') return 26; 
    else if (c == '<') return 27;
    else if (c == '.') return 28;
    else if (c == '>') return 29;
    else if (c == '/') return 30;
    else if (c == '?') return 31;
    else if (c == '[') return 32;
    else if (c == '{') return 33;
    else if (c == ']') return 34;
    else if (c == '}') return 35;
    else if (c == '\\') return 36;
    else if (c == '|') return 37;
    else if (c == '=') return 78;

    else
      return -1;

}

void AccelCalibtration(){
  //Initialize and on-idle condition
   settings[44] = 1; //Off idle wire is U2-2/J18:6 to Vehicle Pin 1 - Vcc
   settings[45] = 1; //On idle wire is U2-3/J18:7 to Vehicle Pin 11 - Vcc
   settings[64] = 1; //Off idle wire is U2-2/J18:6 to Vehicle Pin 1 - Ground (through SW22 on SSS Rev 9)
   settings[65] = 0; //On idle wire is U2-3/J18:7 to Vehicle Pin 11 - Ground (through SW20 on SSS Rev 9)
   for (int trys = 0;trys < 3; trys++){ // cycle a pedal three times per the Cummins Trouble Shooting Faultcode giude No.551
     for (int d = 8; d<100; d++){
     settings[5]=d; // Throttle wire is hooked up to J18-3
     adjustSetting(5);
     if (d==10) {
       settings[64] = 0;//Ties off-idle to ground
       settings[65] = 1; // Opens on-idle
       adjustSetting(64);
       adjustSetting(65);
     }
     delay(10);
   }
   delay(250);
    for (int d = 100; d>8; d--){
     settings[5]=d; // Throttle wire is hooked up to J18-3
     adjustSetting(5);
     if (d==10) {
       
       settings[64] = 1;//Ties off-idle to ground
       settings[65] = 0; // Opens on-idle
       adjustSetting(64);
       adjustSetting(65);
   }
   delay(10);
    
  }
  delay(250);
  }
  
//  settings[5]=50; // Throttle wire is hooked up to J18-3
//  adjustSetting(5);settings[65] = 0;//Ties off-idle to ground
// settings[66] = 1; // Opens on-idle
// adjustSetting(65);
// adjustSetting(66);

 } 

void printHelp(){
  Serial.println("This is a helper message:");
}
