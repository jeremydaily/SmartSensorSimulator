/*
  Title: Smart Sensor Simulator Firmware for the ATMega2560 Processor on an SSS Daughter Board - Rev 6

*/

//Load these libraries
#include <Wire.h> // Digital to Analog converter
#include <mcp_can.h> // J1939 communications
#include <SPI.h> // CAN and Digital Pots
#include <EEPROM.h>
#include "sss_daughter_rev6_defs.h"

//declare instances of the CAN class
MCP_CAN CAN4 = MCP_CAN(CSCAN4Pin);
MCP_CAN CAN5 = MCP_CAN(CSCAN5Pin);


char componentID[47] = "SSS-TBD*Undefined*NoSerialNumber*";
uint8_t SAddress = 0xFB;
char command[100];
char separatorChar = ';';
boolean isIgnitionOn;


char value[6];
byte CANchannel[25];
byte CANmessages[25][8];
int numCANmsgs;
int CANtxPeriod[25];
int periodNumber;

//The following are characters to be converted into numbers. These come from serial commands.
char ID[9];
char period[5];
char data1[9];
char data2[9];
char CANmessage[14];
byte buf[8];
byte _rxBuf[8];
unsigned long CANIDs[25];
unsigned long previousCANmillis[25];
unsigned long IDnumber;
unsigned long rxId;
byte len;

uint8_t wiperValue = 0; //potentiometer wiper value
uint8_t tconValue = 0; //potentiometer terminal connection
uint8_t switchValue = 0;


boolean validHex;
boolean displayCAN;

void setup()
{
  SPI.begin(); // used to interface with the digital potentiometers
  Wire.begin(); //used to interface with the DAC chips

  // Set up Serial Connections
  Serial.begin(115200); // Serial to the USB to UART bridge
  Serial.println("Starting Up...");
  Serial.println("Synercon Technologies Smart Sensor Simulator");
  Serial.println("Program running for the ATmega2560 Processor on the daughterboard.");

  Serial.println("Setting up CAN4...");
  if (CAN4.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN4 init ok!!");
  else Serial.println("CAN0 init fail!!");

  Serial.println("Setting up CAN5...");
  if (CAN5.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN5 init ok!!");
  else Serial.println("CAN0 init fail!!");

  setEEPROM();

  //Set Pin Modes
  pinMode(CSCAN4Pin, OUTPUT);
  pinMode(CSCAN5Pin, OUTPUT);

  pinMode(CSU1Pin, OUTPUT);
  pinMode(CSU2Pin, OUTPUT);
  pinMode(CSU3Pin, OUTPUT);
  pinMode(CSU4Pin, OUTPUT);
  pinMode(CSU5Pin, OUTPUT);
  pinMode(CSU6Pin, OUTPUT);
  pinMode(CSU7Pin, OUTPUT);
  pinMode(CSU8Pin, OUTPUT);
  pinMode(CSU9Pin, OUTPUT);
  pinMode(CSU10Pin, OUTPUT);
  pinMode(CSU11Pin, OUTPUT);
  pinMode(CSU12Pin, OUTPUT);
  pinMode(CSU13Pin, OUTPUT);
  pinMode(CSU14Pin, OUTPUT);
  pinMode(CSU15Pin, OUTPUT);
  pinMode(CSU16Pin, OUTPUT);
  pinMode(CSU17Pin, OUTPUT);
  pinMode(CSU18Pin, OUTPUT);
  pinMode(CSU19Pin, OUTPUT);
  pinMode(CSU20Pin, OUTPUT);
  pinMode(CSU21Pin, OUTPUT);
  pinMode(CSU22Pin, OUTPUT);
  pinMode(CSU23Pin, OUTPUT);
  pinMode(CSU24Pin, OUTPUT);

  pinMode(Select12VPort1Pin, OUTPUT);
  pinMode(Select12VPort2Pin, OUTPUT);
  pinMode(Select12VPort3Pin, OUTPUT);
  pinMode(Select12VPort4Pin, OUTPUT);
  pinMode(Select12VPort7Pin, OUTPUT);
  pinMode(Select12VPort8Pin, OUTPUT);
  pinMode(Select12VPort9Pin, OUTPUT);
  pinMode(Select12VPort10Pin, OUTPUT);

  pinMode(ADC0Pin, INPUT);
  pinMode(IgnitionSensePin, INPUT);
  pinMode(Sense12VPin, INPUT);

  pinMode(CAN4Term1Pin, OUTPUT);
  pinMode(CAN4Term2Pin, OUTPUT);
  pinMode(CAN5Term1Pin, OUTPUT);
  pinMode(CAN5Term2Pin, OUTPUT);
  pinMode(J1939toCAN4Pin, OUTPUT);
  pinMode(J1939toCAN5Pin, OUTPUT);

  pinMode(LDAC0Pin, OUTPUT);
  pinMode(LDAC1Pin, OUTPUT);
  pinMode(LDAC2Pin, OUTPUT);

  pinMode(LINCSPin, OUTPUT);
  pinMode(LINWakePin, OUTPUT);

  pinMode(PWM1Pin, OUTPUT);
  pinMode(PWM2Pin, OUTPUT);
  pinMode(PWM3Pin, OUTPUT);

  pinMode(LowRSelectU1Pin, OUTPUT);
  pinMode(LowRSelectU2Pin, OUTPUT);
  pinMode(LowRSelectU3Pin, OUTPUT);
  pinMode(LowRSelectU4Pin, OUTPUT);
  pinMode(LowRSelectU5Pin, OUTPUT);
  pinMode(LowRSelectU6Pin, OUTPUT);
  pinMode(LowRSelectU7Pin, OUTPUT);
  pinMode(LowRSelectU8Pin, OUTPUT);

  pinMode(GroundSelectU9Pin, OUTPUT);
  pinMode(GroundSelectU10Pin, OUTPUT);
  pinMode(GroundSelectU11Pin, OUTPUT);
  pinMode(GroundSelectU12Pin, OUTPUT);

  pinMode(U1_U2_12VSelectPin, OUTPUT);
  pinMode(U3_U4_12VSelectPin, OUTPUT);
  pinMode(U5_U6_12VSelectPin, OUTPUT);
  pinMode(U7_U8_12VSelectPin, OUTPUT);
  pinMode(U9_U10_12VSelectPin, OUTPUT);
  pinMode(U11_U12_12VSelectPin, OUTPUT);
  pinMode(U13_U14_12VSelectPin, OUTPUT);
  pinMode(U15_U16_12VSelectPin, OUTPUT);
  pinMode(U17_U18_12VSelectPin, OUTPUT);

  //Set all constant pins to their initial values
  digitalWrite(CSCAN4Pin, HIGH);
  digitalWrite(CSCAN5Pin, HIGH);

  digitalWrite(CSU1Pin, HIGH);
  digitalWrite(CSU2Pin, HIGH);
  digitalWrite(CSU3Pin, HIGH);
  digitalWrite(CSU4Pin, HIGH);
  digitalWrite(CSU5Pin, HIGH);
  digitalWrite(CSU6Pin, HIGH);
  digitalWrite(CSU7Pin, HIGH);
  digitalWrite(CSU8Pin, HIGH);
  digitalWrite(CSU9Pin, HIGH);
  digitalWrite(CSU10Pin, HIGH);
  digitalWrite(CSU11Pin, HIGH);
  digitalWrite(CSU12Pin, HIGH);
  digitalWrite(CSU13Pin, HIGH);
  digitalWrite(CSU14Pin, HIGH);
  digitalWrite(CSU15Pin, HIGH);
  digitalWrite(CSU16Pin, HIGH);
  digitalWrite(CSU17Pin, HIGH);
  digitalWrite(CSU18Pin, HIGH);
  digitalWrite(CSU19Pin, HIGH);
  digitalWrite(CSU20Pin, HIGH);
  digitalWrite(CSU21Pin, HIGH);
  digitalWrite(CSU22Pin, HIGH);
  digitalWrite(CSU23Pin, HIGH);
  digitalWrite(CSU24Pin, HIGH);

  digitalWrite(LDAC0Pin, LOW);
  digitalWrite(LDAC1Pin, LOW);
  digitalWrite(LDAC2Pin, LOW);

  digitalWrite(LINCSPin, HIGH);
  digitalWrite(LINWakePin, HIGH);

  //Read from EEPROM and Write signals for the wiper of the digital potentiometers
  Serial.println();
  Serial.println("Setting Wiper Positions.");
  Serial.println("Addr. (DEC)\tAddr. Name\tChip\tPort\tWire\tValue");

  for (int i = 0; i < 18; i++) {
    EEPROM.get(wiperAddresses[i], wiperValue);
    adjustWiperValues(wiperAddresses[i], wiperValue);
  }

  //Read from EEPROM and Write signals for the terminal connections of the digital potentiometers
  Serial.println();
  Serial.println("Potentiometer Terminal Connection Legend for Values:");
  Serial.println("Value\tTerm. B (Low)\tWiper\tTerm. A (High)");
  Serial.println("8\tDisconnected\tDisconnected\tDisconnected");
  Serial.println("9\tConnected\tDisconnected\tDisconnected");
  Serial.println("10\tDisconnected\tConnected\tDisconnected");
  Serial.println("11\tConnected\tConnected\tDisconnected");
  Serial.println("12\tDisconnected\tDisconnected\tConnected");
  Serial.println("13\tConnected\tDisconnected\tConnected");
  Serial.println("14\tDisconnected\tConnected\tConnected");
  Serial.println("15\tConnected\tConnected\tConnected");

  Serial.println("Terminal Connection Value Table");
  Serial.println("Addr. (DEC)\tChip\tSchematic Port\tWire Connector\tValue");

  for (int i = 0; i < 18; i++) {
    EEPROM.get(tconAddresses[i], tconValue);
    adjustWiperValues(tconAddresses[i], tconValue);
  }

  Serial.println();
  Serial.println("Setting up analog to Digital Converters...");
  Serial.println("Addr. (DEC)\tAddr. Name\tChip-Pin\tSchematic Port\tWire Connector\tValue");

  EEPROM.get(VoutAaddress, VoutA);
  Serial.print(VoutAaddress);
  Serial.print("\tVoutAaddress\tU4-Pin6\tVoutA\tJ5-19\t");
  Serial.println(VoutA);

  EEPROM.get(VoutBaddress, VoutB);
  Serial.print(VoutBaddress);
  Serial.print("\tVoutBaddress\tU4-Pin7\tVoutB\tJ5-20\t");
  Serial.println(VoutB);

  EEPROM.get(VoutCaddress, VoutC);
  Serial.print(VoutCaddress);
  Serial.print("\tVoutCaddress\tU4-Pin8\tVoutC\tJ5-1\t");
  Serial.println(VoutC);

  EEPROM.get(VoutDaddress, VoutD);
  Serial.print(VoutDaddress);
  Serial.print("\tVoutDaddress\tU4-Pin9\tVoutD\tJ5-2\t");
  Serial.println(VoutD);

  EEPROM.get(VoutEaddress, VoutE);
  Serial.print(VoutEaddress);
  Serial.print("\tVoutEaddress\tU6-Pin6\tVoutE\tJ1-12\t");
  Serial.println(VoutE);

  EEPROM.get(VoutFaddress, VoutF);
  Serial.print(VoutFaddress);
  Serial.print("\tVoutFaddress\tU6-Pin7\tVoutF\tJ1-13\t");
  Serial.println(VoutF);

  EEPROM.get(VoutGaddress, VoutG);
  Serial.print(VoutGaddress);
  Serial.print("\tVoutGaddress\tU6-Pin8\tVoutG\tJ1-14\t");
  Serial.println(VoutG);

  EEPROM.get(VoutHaddress, VoutH);
  Serial.print(VoutHaddress);
  Serial.print("\tVoutHaddress\tU6-Pin9\tVoutH\tJ1-15\t");
  Serial.println(VoutH);

  EEPROM.get(VoutIaddress, VoutI);
  Serial.print(VoutIaddress);
  Serial.print("\tVoutIaddress\tU7-Pin6\tVoutI x 2.5\tJ1-16\t");
  Serial.println(VoutI);

  EEPROM.get(VoutJaddress, VoutJ);
  Serial.print(VoutJaddress);
  Serial.print("\tVoutJaddress\tU7-Pin7\tVoutJ x 2.5\tJ1-17\t");
  Serial.println(VoutJ);

  EEPROM.get(VoutKaddress, VoutK);
  Serial.print(VoutKaddress);
  Serial.print("\tVoutKaddress\tU7-Pin8\tVoutK x 2.5\tJ2-1\t");
  Serial.println(VoutK);

  EEPROM.get(VoutLaddress, VoutL);
  Serial.print(VoutLaddress);
  Serial.print("\tVoutLaddress\tU7-Pin9\tVoutL x 2.5\tJ2-2\t");
  Serial.println(VoutL);
  setDAC();

  Serial.println();
  Serial.println("Setting up Switches to +12V and GND");
  Serial.println("Addr. (DEC)\tAddr. Name\tChip or Resistor\tSchematic Port\tWire Connector\tValue");
  for (int i = 0; i < 38; i++) {
    EEPROM.get(switchAddresses[i], switchValue);
    adjustSwitchValues(switchAddresses[i], switchValue);
  }

  String commandString = "CAN0CFF00520020FFFFFFFFFFFFFFFF"; // SCR Outlet Nox Sensor Signal
  commandString.toCharArray(command, 32);
  processCommand(31);

  commandString = "CAN0CFF00510020FFFFFFFFFFFFFFFF"; // SCR Inlet Nox Sensor Signal
  commandString.toCharArray(command, 32);
  processCommand(31);



  EEPROM.get(componentIDaddress, componentID);
  Serial.print("Component ID: ");
  Serial.println(componentID);

  Serial.println("Daughter Board Program for ATmega2560 Processor.");
  Serial.println("Finished Starting Up... ");

}


void loop() {
  //Check for new commands on the serial bus
  if (Serial.available() > 0)
  {
    int nDataBytes = Serial.readBytesUntil(separatorChar, command, 99);
    processCommand(nDataBytes);
    memset(command, 0, sizeof(command));
  }

  //isIgnitionOn = digitalRead(ignitionPin);
  isIgnitionOn = true; //for testing
  sendCANmessages();

} //end loop()



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setDAC() { //Settings are close to millivolts.
  VoutA = map(VoutA, 0, 3606, 0, 3000); //0x0BBB or 3003 = 3.616V on VoutA
  VoutB = map(VoutB, 0, 3606, 0, 3000); //0x06BB or 1721 = 2.074V on VoutB
  VoutC = map(VoutC, 0, 3606, 0, 3000); //0x08BB = 2.682V on VoutC
  VoutD = map(VoutD, 0, 3606, 0, 3000); //0x0ABB = 3.298V on VoutD
  VoutE = map(VoutE, 0, 3606, 0, 3000);
  VoutF = map(VoutF, 0, 3606, 0, 3000);
  VoutG = map(VoutG, 0, 3606, 0, 3000);
  VoutH = map(VoutH, 0, 3606, 0, 3000);
  VoutI = map(VoutI, 0, 3606, 0, 3000);
  VoutJ = map(VoutJ, 0, 3606, 0, 3000);
  VoutK = map(VoutK, 0, 3606, 0, 3000);
  VoutL = map(VoutL, 0, 3606, 0, 3000);

  VoutA = constrain(VoutA, 0, 4095);
  VoutB = constrain(VoutB, 0, 4095);
  VoutC = constrain(VoutC, 0, 4095);
  VoutD = constrain(VoutD, 0, 4095);
  VoutE = constrain(VoutE, 0, 4095);
  VoutF = constrain(VoutF, 0, 4095);
  VoutG = constrain(VoutG, 0, 4095);
  VoutH = constrain(VoutH, 0, 4095);
  VoutI = constrain(VoutI, 0, 4095);
  VoutJ = constrain(VoutJ, 0, 4095);
  VoutK = constrain(VoutK, 0, 4095);
  VoutL = constrain(VoutL, 0, 4095);

  Wire.beginTransmission(0x60);
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
    Serial.print("Setting DAC 0x60 over I2C returned error flag of ");
    Serial.println(flag);
  }
  delay(1);

  Wire.beginTransmission(0x61);
  Wire.write(byte(0x50));
  Wire.write(highByte(VoutE));
  Wire.write(lowByte(VoutE));
  Wire.write(highByte(VoutF));
  Wire.write(lowByte(VoutF));
  Wire.write(highByte(VoutG));
  Wire.write(lowByte(VoutG));
  Wire.write(highByte(VoutH));
  Wire.write(lowByte(VoutH));
  int flag1 = Wire.endTransmission();

  if (flag1 != 0)
  {
    Serial.print("Setting DAC 0x61 over I2C returned error flag of ");
    Serial.println(flag1);
  }

  delay(1);


  Wire.beginTransmission(0x62);
  Wire.write(byte(0x50));
  Wire.write(highByte(VoutI));
  Wire.write(lowByte(VoutI));
  Wire.write(highByte(VoutJ));
  Wire.write(lowByte(VoutJ));
  Wire.write(highByte(VoutK));
  Wire.write(lowByte(VoutK));
  Wire.write(highByte(VoutL));
  Wire.write(lowByte(VoutL));
  int flag2 = Wire.endTransmission();

  if (flag2 != 0)
  {
    Serial.print("Setting DAC 0x62 over I2C returned error flag of ");
    Serial.println(flag2);
  }

}



void sendCANmessages()
{
  if (isIgnitionOn && numCANmsgs > 0 ) {
    for (int i = 0; i < numCANmsgs; i++) {
      unsigned long currentMillis = millis();
      if ((currentMillis - previousCANmillis[i]) > CANtxPeriod[i]) { // Do this on time.
        previousCANmillis[i] = currentMillis;
        if (CANchannel[i]==0) CAN4.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]);
        else CAN5.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]);
      } //end if
    } // end for
  } // end if
}


void processCommand(int numDataBytes) {
  Serial.print("Command: ");
  for (int t = 0; t < numDataBytes; t++) Serial.print(command[t]);
  Serial.println();

  boolean addressValid = true;
  boolean dataValid = true;
  //Example command: set,34,12 or set 34 12
  if  ((command[0] == 's' || command[0] == 'S') && (command[1] == 'e' || command[1] == 'E') && (command[2] == 't' || command[2] == 'T')) {
    char addressString[5];
    memset(addressString, 0, 5);
    uint8_t q = 4;
    uint8_t p = 0;
    while (isdigit(command[q]) && q < 8) {
      addressString[p] = command[q];
      q++;
      p++;
    }
    if (q > 4) {
      uint16_t address = atoi(addressString);
      char valueString[8];
      memset(valueString, 0, 8);
      p = 0;
      q += 1;
      while (isdigit(command[q]) && q < numDataBytes )
      {
        valueString[p] = command[q];
        q++;
        p++;
      }
      if (p > 0)
      {
        uint16_t value = atoi(valueString);
        if (value > 255) EEPROM.put(address, uint16_t(value));
        else EEPROM.update(address, uint8_t(value));
        Serial.print("Wrote ");
        Serial.print(value);
        Serial.print(" to address ");
        Serial.println(address);
      }
      else
      {
        Serial.println("The set command failed.");
      }
    }
  }
  else if ( (command[0] == 'r' || command[0] == 'R') && (command[1] == 'e' || command[1] == 'E') && (command[2] == 'S' || command[2] == 's'))
  {
    setup();
  }
  else if ( (command[0] == 'i' || command[0] == 'I') && (command[1] == 'D' || command[1] == 'd'))
  {
    memset(componentID, 0, sizeof(componentID));
    uint8_t q = 3;
    while (isprint(command[q]) && q < 35) {
      componentID[q - 3] = command[q];
      q++;
    }
    if (q > 3)
    {
      EEPROM.put(componentIDaddress, componentID);
      Serial.print("Wrote ");
      Serial.print(componentID);
      Serial.print(" to address ");
      Serial.println(componentIDaddress);
    }
    else
    {
      Serial.println("Component ID not set.");
    }
  }
  else if ( (command[0] == 'c' || command[0] == 'C') && (command[1] == 'a' || command[1] == 'A') && (command[2] == 'N' || command[2] == 'n'))
  {
    Serial.println("CAN Command. Make first byte on ID = 0xF0 to reset CAN messages.");
    if (numDataBytes >= 31) {
      for (int q = 3; q < 31; q++) {
        validHex = isxdigit(command[q]);
        if (!validHex) break;
      }
      if (validHex) {

        for (int q = 3; q < 11; q++) ID[q - 3] = command[q];
        for (int q = 11; q < 15; q++) period[q - 11] = command[q];
        for (int q = 15; q < 23; q++) data1[q - 15] = command[q];
        for (int q = 23; q < 31; q++) data2[q - 23] = command[q];


        IDnumber = strtoul(ID, 0, 16);
        Serial.print("ID: ");
        Serial.print(IDnumber, HEX);

        CANmessage[0] = (byte)((IDnumber & 0xFF000000) >> 24);
        CANmessage[1] = (byte)((IDnumber & 0xFF0000) >> 16);
        CANmessage[2] = (byte)((IDnumber & 0xFF00) >> 8);
        CANmessage[3] = (byte)((IDnumber & 0xFF));



        periodNumber = (int)(strtoul(period, 0, 10));
        if (periodNumber < 10) periodNumber = 10; //Don't allow the SSS to flood the bus
        Serial.print(" Period: ");
        Serial.print(periodNumber, DEC);


        CANmessage[4] = (byte)((periodNumber & 0x0000FF00) >> 8);
        CANmessage[5] = (byte)((periodNumber & 0x000000FF) >> 0);

        unsigned long  firstData = strtoul(data1, 0, 16);
        unsigned long  secondData = strtoul(data2, 0, 16);
        Serial.print(" Data: ");
        Serial.print(firstData, HEX);
        Serial.print(secondData, HEX);
        Serial.println();

        CANmessage[6] = (byte)((firstData & 0xFF000000) >> 24) ;
        CANmessage[7] = (byte)((firstData & 0x00FF0000) >> 16);
        CANmessage[8] = (byte)((firstData & 0x0000FF00) >> 8);
        CANmessage[9] = (byte)((firstData & 0x000000FF) >> 0);
        CANmessage[10] = (byte)((secondData & 0xFF000000) >> 24);
        CANmessage[11] = (byte)((secondData & 0x00FF0000) >> 16);
        CANmessage[12] = (byte)((secondData & 0x0000FF00) >> 8);
        CANmessage[13] = (byte)((secondData & 0x000000FF) >> 0);

        Serial.print("Built CAN message structure: ");
        for (int g = 0; g < 14; g++) Serial.print((byte)CANmessage[g], HEX);
        Serial.println();


        buildCANmessage();

      }
      else {
        Serial.println("INPUT ERROR: invalid Hex Data.");
        Serial.println(command);
      }
    }
    else
    {
      Serial.println("INPUT ERROR: Not enough data for creating a CAN message.");
      Serial.println(command);
    }
  }
  else if (isdigit(command[0]))
  {
    char addressString[5];
    memset(addressString, 0, 5);
    uint8_t q = 0;
    uint8_t p = 0;
    while (isdigit(command[q]) && q < 8) {
      addressString[p] = command[q];
      q++;
      p++;
    }
    if (q > 0) {
      uint16_t address = atoi(addressString);
      char valueString[8];
      memset(valueString, 0, 8);
      p = 0;
      q += 1;
      while (isdigit(command[q]) && q < numDataBytes )
      {
        valueString[p] = command[q];
        q++;
        p++;
      }
      if (p > 0)
      {
        uint16_t value = atoi(valueString);
        if (address < 128) adjustSwitchValues(address,value);
        else adjustWiperValues(address,value);
        
      }
      else
      {
        Serial.println("The fast set command failed.");
      }
    }
  }
  else
  {
    Serial.println("INPUT ERROR: Command not recognized.");
  }

}

void buildCANmessage()
{

  Serial.print("Building CAN Message number ");
  Serial.print(numCANmsgs);

  CANIDs[numCANmsgs] = IDnumber & 0x1fffffff;

  Serial.print(", ID: ");
  Serial.print(CANIDs[numCANmsgs], HEX);

  CANtxPeriod[numCANmsgs] = periodNumber;
  Serial.print(", Period: ");
  Serial.print((unsigned int)CANtxPeriod[numCANmsgs]);

  CANchannel[numCANmsgs] = CANmessage[0] & 0x20; // either 32 or 0

  Serial.print(", Channel: ");
  Serial.print(CANchannel[numCANmsgs]);
  Serial.print(", Data: ");
  for (int j = 0; j < 8; j++)
  {
    CANmessages[numCANmsgs][j] =  CANmessage[6 + j];
    Serial.print(int(CANmessages[numCANmsgs][j]), HEX);
    Serial.print(' ');
  }
  Serial.println();
  numCANmsgs++;
}

void setEEPROM() {
  if (EEPROM.read(0) != 1) {
    EEPROM.write(0, 1);

    EEPROM.update(wiperPort1Address, uint8_t(40));
    EEPROM.update(wiperPort2Address, uint8_t(60));
    EEPROM.update(wiperPort3Address, uint8_t(80));
    EEPROM.update(wiperPort4Address, uint8_t(100));
    EEPROM.update(wiperPort5Address, uint8_t(120));
    EEPROM.update(wiperPort6Address, uint8_t(140));
    EEPROM.update(wiperPort7Address, uint8_t(160));
    EEPROM.update(wiperPort8Address, uint8_t(180));
    EEPROM.update(wiperPort9Address, uint8_t(200));
    EEPROM.update(wiperPort10Address, uint8_t(220));
    EEPROM.update(wiperPort11Address, uint8_t(30));
    EEPROM.update(wiperPort12Address, uint8_t(50));
    EEPROM.update(wiperPort13Address, uint8_t(70));
    EEPROM.update(wiperPort14Address, uint8_t(90));
    EEPROM.update(wiperPort15Address, uint8_t(110));
    EEPROM.update(wiperPort16Address, uint8_t(130));
    EEPROM.update(wiperPort17Address, uint8_t(150));
    EEPROM.update(wiperPort18Address, uint8_t(170));

    EEPROM.update(tconPort1Address, uint8_t(15));
    EEPROM.update(tconPort2Address, uint8_t(15));
    EEPROM.update(tconPort3Address, uint8_t(15));
    EEPROM.update(tconPort4Address, uint8_t(15));
    EEPROM.update(tconPort5Address, uint8_t(15));
    EEPROM.update(tconPort6Address, uint8_t(15));
    EEPROM.update(tconPort7Address, uint8_t(15));
    EEPROM.update(tconPort8Address, uint8_t(15));
    EEPROM.update(tconPort9Address, uint8_t(15));
    EEPROM.update(tconPort10Address, uint8_t(15));
    EEPROM.update(tconPort11Address, uint8_t(15));
    EEPROM.update(tconPort12Address, uint8_t(15));
    EEPROM.update(tconPort13Address, uint8_t(15));
    EEPROM.update(tconPort14Address, uint8_t(15));
    EEPROM.update(tconPort15Address, uint8_t(15));
    EEPROM.update(tconPort16Address, uint8_t(15));
    EEPROM.update(tconPort17Address, uint8_t(15));
    EEPROM.update(tconPort18Address, uint8_t(15));

    EEPROM.put(VoutAaddress, uint16_t(500));
    EEPROM.put(VoutBaddress, uint16_t(800));
    EEPROM.put(VoutCaddress, uint16_t(1100));
    EEPROM.put(VoutDaddress, uint16_t(1400));
    EEPROM.put(VoutEaddress, uint16_t(1700));
    EEPROM.put(VoutFaddress, uint16_t(2000));
    EEPROM.put(VoutGaddress, uint16_t(2300));
    EEPROM.put(VoutHaddress, uint16_t(2600));
    EEPROM.put(VoutIaddress, uint16_t(100));
    EEPROM.put(VoutJaddress, uint16_t(400));
    EEPROM.put(VoutKaddress, uint16_t(800));
    EEPROM.put(VoutLaddress, uint16_t(1300));

    EEPROM.update(Select12VPort1Address, uint8_t(0));
    EEPROM.update(Select12VPort2Address, uint8_t(0));
    EEPROM.update(Select12VPort3Address, uint8_t(0));
    EEPROM.update(Select12VPort4Address, uint8_t(0));

    EEPROM.update(Select12VPort7Address, uint8_t(0));
    EEPROM.update(Select12VPort8Address, uint8_t(0));
    EEPROM.update(Select12VPort9Address, uint8_t(0));
    EEPROM.update(Select12VPort10Address, uint8_t(0));

    EEPROM.update(CAN4Term1Address, uint8_t(1));
    EEPROM.update(CAN4Term2Address, uint8_t(0));
    EEPROM.update(CAN5Term1Address, uint8_t(1));
    EEPROM.update(CAN5Term2Address, uint8_t(0));
    EEPROM.update(J1939toCAN4Address, uint8_t(0));
    EEPROM.update(J1939toCAN5Address, uint8_t(0));

    EEPROM.update(LowRSelectU1Address, uint8_t(0));
    EEPROM.update(LowRSelectU2Address, uint8_t(0));
    EEPROM.update(LowRSelectU3Address, uint8_t(0));
    EEPROM.update(LowRSelectU4Address, uint8_t(0));
    EEPROM.update(LowRSelectU5Address, uint8_t(0));
    EEPROM.update(LowRSelectU6Address, uint8_t(0));
    EEPROM.update(LowRSelectU7Address, uint8_t(0));
    EEPROM.update(LowRSelectU8Address, uint8_t(0));

    EEPROM.update(GroundSelectU12Address, uint8_t(0));
    EEPROM.update(GroundSelectU11Address, uint8_t(0));
    EEPROM.update(GroundSelectU10Address, uint8_t(0));
    EEPROM.update(GroundSelectU9Address, uint8_t(0));

    EEPROM.update(U1_U2_12VSelectAddress, uint8_t(1));
    EEPROM.update(U3_U4_12VSelectAddress, uint8_t(1));
    EEPROM.update(U5_U6_12VSelectAddress, uint8_t(1));
    EEPROM.update(U7_U8_12VSelectAddress, uint8_t(1));
    EEPROM.update(U9_U10_12VSelectAddress, uint8_t(1));
    EEPROM.update(U11_U12_12VSelectAddress, uint8_t(1));
    EEPROM.update(U13_U14_12VSelectAddress, uint8_t(1));
    EEPROM.update(U15_U16_12VSelectAddress, uint8_t(1));
    EEPROM.update(U17_U18_12VSelectAddress, uint8_t(1));



    EEPROM.update(PWM1Address, uint8_t(64));
    EEPROM.update(PWM2Address, uint8_t(127));
    EEPROM.update(PWM3Address, uint8_t(192));

    EEPROM.put(componentIDaddress, componentID);
    EEPROM.put(sssSAaddress, SAddress);

    Serial.println("New EEPROM values set.");
  }
  else
  {
    Serial.println("EEPROM values already set.");
  }
}



