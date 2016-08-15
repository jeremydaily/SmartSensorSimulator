/*
Title: Smart Sensor Simulator Firmware for the ATMega2560 Processor on an SSS Daughter Board - Rev 6

*/

//Load these libraries
#include <Wire.h> // Digital to Analog converter
#include <mcp_can.h> // J1939 communications
#include <SPI.h> // CAN and Digital Pots
#include <EEPROM.h>

  

//Define Pin Numbers
#define CSCAN4Pin 17
#define CSCAN5Pin 53

//declare instances of the CAN class
MCP_CAN CAN4 = MCP_CAN(CSCAN4Pin);  
MCP_CAN CAN5 = MCP_CAN(CSCAN5Pin);  
  
#define CSU1Pin 62
#define CSU2Pin 63
#define CSU3Pin 64
#define CSU4Pin 65
#define CSU5Pin 66
#define CSU6Pin 67
#define CSU7Pin 68
#define CSU8Pin 69
#define CSU9Pin 22
#define CSU10Pin 23
#define CSU11Pin 24
#define CSU12Pin 25
#define CSU13Pin 26
#define CSU14Pin 27
#define CSU15Pin 28
#define CSU16Pin 29
#define CSU17Pin 75
#define CSU18Pin 74
#define CSU19Pin 73
#define CSU20Pin 72
#define CSU21Pin 71
#define CSU22Pin 70
#define CSU23Pin 83
#define CSU24Pin 38

#define Reset1  18

#define Select12VPort1Pin 49
#define Select12VPort2Pin 48
#define Select12VPort3Pin 47
#define Select12VPort4Pin 46
#define Select12VPort7Pin 45
#define Select12VPort8Pin 44
#define Select12VPort9Pin 43
#define Select12VPort10Pin 42

#define ADC0Pin 54
#define Sense12VPin 56
#define IgnitionSensePin 55
#define IgnitionSelectPin 19


#define CAN4Term1Pin 85
#define CAN4Term2Pin 4
#define CAN5Term1Pin 39
#define CAN5Term2Pin 84
#define J1939toCAN4Pin 40
#define J1939toCAN5Pin 41

#define CAN4IntPin 3
#define CAN5IntPin 77

#define LDAC0Pin 79
#define LDAC1Pin 6
#define LDAC2Pin 7
#define LINCSPin  81
#define LINWakePin  82

#define LowRSelectU1Pin    37
#define LowRSelectU2Pin    36
#define LowRSelectU3Pin    35
#define LowRSelectU4Pin    34
#define LowRSelectU5Pin    33
#define LowRSelectU6Pin    32
#define LowRSelectU7Pin    9
#define LowRSelectU8Pin    80
#define GroundSelectU11Pin 16
#define GroundSelectU12Pin 13
#define GroundSelectU10Pin 31
#define GroundSelectU9Pin  30


#define PWM1Pin 5
#define PWM2Pin 2
#define PWM3Pin 8

#define U1_U2_12VSelectPin    76
#define U11_U12_12VSelectPin  61
#define U13_U14_12VSelectPin  10
#define U15_U16_12VSelectPin  11
#define U17_U18_12VSelectPin  12
#define U3_U4_12VSelectPin    57
#define U5_U6_12VSelectPin    58
#define U7_U8_12VSelectPin    59
#define U9_U10_12VSelectPin   60

//Define EEPROM Addresses for the Values

#define componentIDaddress 80
#define sssSAaddress       127

#define wiperPort1Address  128
#define wiperPort2Address  130
#define wiperPort3Address  132
#define wiperPort4Address  134
#define wiperPort5Address  136
#define wiperPort6Address  138
#define wiperPort7Address  140
#define wiperPort8Address  142
#define wiperPort9Address  144
#define wiperPort10Address 146
#define wiperPort11Address 148
#define wiperPort12Address 150
#define wiperPort13Address 152
#define wiperPort14Address 154
#define wiperPort15Address 156
#define wiperPort16Address 158
#define wiperPort17Address 160
#define wiperPort18Address 162

#define tconPort1Address  129
#define tconPort2Address  131
#define tconPort3Address  133
#define tconPort4Address  135
#define tconPort5Address  137
#define tconPort6Address  139
#define tconPort7Address  141
#define tconPort8Address  143
#define tconPort9Address  145
#define tconPort10Address 147
#define tconPort11Address 149
#define tconPort12Address 151
#define tconPort13Address 153
#define tconPort14Address 155
#define tconPort15Address 157
#define tconPort16Address 159
#define tconPort17Address 161
#define tconPort18Address 163


#define Select12VPort1Address   46
#define Select12VPort2Address   45
#define Select12VPort3Address   44
#define Select12VPort4Address   43
#define Select12VPort7Address   42
#define Select12VPort8Address   41
#define Select12VPort9Address   40
#define Select12VPort10Address  39


#define CAN4Term1Address  77
#define CAN4Term2Address  2
#define CAN5Term1Address  36
#define CAN5Term2Address  76
#define J1939toCAN4Address  37
#define J1939toCAN5Address  38

#define LINCSAddress   73
#define LINWakeAddress   74

#define LowRSelectU1Address  34
#define LowRSelectU2Address  33
#define LowRSelectU3Address  32
#define LowRSelectU4Address  31
#define LowRSelectU5Address  30
#define LowRSelectU6Address  29
#define LowRSelectU7Address  7
#define LowRSelectU8Address  72
#define GroundSelectU11Address 15
#define GroundSelectU12Address 12
#define GroundSelectU10Address 28
#define GroundSelectU9Address  27

#define PWM1Address  3
#define PWM2Address  1
#define PWM3Address  5

#define U1_U2_12VSelectAddress 70
#define U3_U4_12VSelectAddress 51
#define U5_U6_12VSelectAddress 52
#define U7_U8_12VSelectAddress 53
#define U9_U10_12VSelectAddress 54
#define U11_U12_12VSelectAddress 55
#define U13_U14_12VSelectAddress 9
#define U15_U16_12VSelectAddress 10
#define U17_U18_12VSelectAddress 11

#define VoutAaddress 164
#define VoutBaddress 166
#define VoutCaddress 168
#define VoutDaddress 170
#define VoutEaddress 172
#define VoutFaddress 174
#define VoutGaddress 176
#define VoutHaddress 178
#define VoutIaddress 180
#define VoutJaddress 182
#define VoutKaddress 184
#define VoutLaddress 186

char componentID[47] = "SSS-TBD*Undefined*NoSerialNumber*";
uint8_t SAddress = 0xFB;
char command[100];
char separatorChar = ';';
boolean isIgnitionOn;

uint8_t wiperValue = 0; //potentiometer wiper value
uint8_t tconValue = 0; //potentiometer terminal connection

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
  if(CAN4.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN4 init ok!!");
  else Serial.println("CAN0 init fail!!");
  
  Serial.println("Setting up CAN5..."); 
  if(CAN5.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN5 init ok!!");
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
  
  pinMode(ADC0Pin,INPUT);
  pinMode(IgnitionSensePin,INPUT);
  pinMode(Sense12VPin,INPUT);
  
  pinMode(CAN4Term1Pin, OUTPUT);
  pinMode(CAN4Term2Pin, OUTPUT);
  pinMode(CAN5Term1Pin, OUTPUT);
  pinMode(CAN5Term2Pin, OUTPUT);
  pinMode(J1939toCAN4Pin, OUTPUT);
  pinMode(J1939toCAN5Pin, OUTPUT);
  
  pinMode(LDAC0Pin, OUTPUT);
  pinMode(LDAC1Pin, OUTPUT);
  pinMode(LDAC2Pin, OUTPUT);
  
  pinMode(LINCSPin,OUTPUT);
  pinMode(LINWakePin,OUTPUT);
  
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
  
  digitalWrite(CSU1Pin,HIGH);
  digitalWrite(CSU2Pin,HIGH);
  digitalWrite(CSU3Pin,HIGH);
  digitalWrite(CSU4Pin,HIGH);
  digitalWrite(CSU5Pin,HIGH);
  digitalWrite(CSU6Pin,HIGH);
  digitalWrite(CSU7Pin,HIGH);
  digitalWrite(CSU8Pin,HIGH);
  digitalWrite(CSU9Pin,HIGH);
  digitalWrite(CSU10Pin,HIGH);
  digitalWrite(CSU11Pin,HIGH);
  digitalWrite(CSU12Pin,HIGH);
  digitalWrite(CSU13Pin,HIGH);
  digitalWrite(CSU14Pin,HIGH);
  digitalWrite(CSU15Pin,HIGH);
  digitalWrite(CSU16Pin,HIGH);
  digitalWrite(CSU17Pin,HIGH);
  digitalWrite(CSU18Pin,HIGH);
  digitalWrite(CSU19Pin,HIGH);
  digitalWrite(CSU20Pin,HIGH);
  digitalWrite(CSU21Pin,HIGH);
  digitalWrite(CSU22Pin,HIGH);
  digitalWrite(CSU23Pin,HIGH);
  digitalWrite(CSU24Pin,HIGH);

  digitalWrite(LDAC0Pin, LOW);
  digitalWrite(LDAC1Pin, LOW);
  digitalWrite(LDAC2Pin, LOW);
  
  digitalWrite(LINCSPin, HIGH);
  digitalWrite(LINWakePin,HIGH);

  //Read from EEPROM and Write signals for the wiper of the digital potentiometers
  Serial.println();
  Serial.println("Setting Wiper Positions.");
  Serial.println("Addr. (DEC)\tAddr. Name\tChip\tPort\tWire\tValue");
  
  EEPROM.get(wiperPort1Address,wiperValue);
  setWiper(CSU1Pin,wiperValue);
  Serial.print(wiperPort1Address);
  Serial.print("\twiperPort1Address\tU35-P0W\tPort 1\tJ1-1\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort2Address,wiperValue);
  setWiper(CSU2Pin,wiperValue);
  Serial.print(wiperPort2Address);
  Serial.print("\twiperPort2Address\tU38-P0W\tPort 2\tJ1-2\t");
  Serial.println(wiperValue);
 
  EEPROM.get(wiperPort3Address,wiperValue);
  setWiper(CSU3Pin,wiperValue);
  Serial.print(wiperPort3Address);
  Serial.print("\twiperPort3Address\tU36-P0W\tPort 3\tJ1-3\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort4Address,wiperValue);
  setWiper(CSU4Pin,wiperValue);
  Serial.print(wiperPort4Address);
  Serial.print("\twiperPort4Address\tU39-P0W\tPort 4\tJ1-4\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort5Address,wiperValue);
  setWiper(CSU5Pin,wiperValue);
  Serial.print(wiperPort5Address);
  Serial.print("\twiperPort4Address\tU37-P0W\tPort 5\tJ1-5\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort6Address,wiperValue);
  setWiper(CSU6Pin,wiperValue);
  Serial.print(wiperPort6Address);
  Serial.print("\twiperPort6Address\tU40-P0W\tPort 6\tJ1-6\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort7Address,wiperValue);
  setWiper(CSU7Pin,wiperValue);
  Serial.print(wiperPort7Address);
  Serial.print("\twiperPort7Address\tU45-P0W\tPort 7\tJ1-7\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort8Address,wiperValue);
  setWiper(CSU8Pin,wiperValue);
  Serial.print(wiperPort8Address);
  Serial.print("\twiperPort8Address\tU8-P0W\tPort 8\tJ1-8\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort9Address,wiperValue);
  setWiper(CSU9Pin,wiperValue);
  Serial.print(wiperPort9Address);
  Serial.print("\twiperPort9Address\tU9-P0W\tPort 9\tJ1-9\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort10Address,wiperValue);
  setWiper(CSU10Pin,wiperValue);
  Serial.print(wiperPort10Address);
  Serial.print("\twiperPort10Address\tU47-P0W\tPort 10\tJ1-10\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort11Address,wiperValue);
  setWiper(CSU11Pin,wiperValue);
  Serial.print(wiperPort11Address);
  Serial.print("\twiperPort11Address\tU46-P0W\tPort 11\tJ5-6\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort12Address,wiperValue);
  setWiper(CSU12Pin,wiperValue);
  Serial.print(wiperPort12Address);
  Serial.print("\twiperPort12Address\tU48-P0W\tPort 12\tJ5-12\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort13Address,wiperValue);
  setWiper(CSU13Pin,wiperValue);
  Serial.print(wiperPort13Address);
  Serial.print("\twiperPort13Address\tU53-P0W\tPort 13\tJ5-13\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort14Address,wiperValue);
  setWiper(CSU14Pin,wiperValue);
  Serial.print(wiperPort14Address);
  Serial.print("\twiperPort14Address\tU56-P0W\tPort 14\tJ5-14\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort15Address,wiperValue);
  setWiper(CSU15Pin,wiperValue);
  Serial.print(wiperPort15Address);
  Serial.print("\twiperPort15Address\tU36-P0W\tPort 15\tJ5-15\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort16Address,wiperValue);
  setWiper(CSU16Pin,wiperValue);
  Serial.print(wiperPort16Address);
  Serial.print("\twiperPort16Address\tU57-P0W\tPort 16\tJ5-16\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort17Address,wiperValue);
  setWiper(CSU17Pin,wiperValue);
  Serial.print(wiperPort17Address);
  Serial.print("\twiperPort17Address\tU55-P0W\tPort 17\tJ5-17\t");
  Serial.println(wiperValue);

  EEPROM.get(wiperPort18Address,wiperValue);
  setWiper(CSU18Pin,wiperValue);
  Serial.print(wiperPort18Address);
  Serial.print("\twiperPort18Address\tU58-P0W\tPort 18\tJ5-18\t");
  Serial.println(wiperValue);
  
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
  
  EEPROM.get(tconPort1Address,tconValue);
  setTCON(CSU1Pin,tconValue);
  Serial.print(tconPort1Address);
  Serial.print("\ttconPort1Address\tU35\tPort 1\tJ1-1\t");
  Serial.println(tconValue);

  EEPROM.get(tconPort2Address,tconValue);
  setTCON(CSU2Pin,tconValue);
  Serial.print(tconPort2Address);
  Serial.print("\ttconPort2Address\tU38\tPort 2\tJ1-2\t");
  Serial.println(tconValue);
 
  EEPROM.get(tconPort3Address,tconValue);
  setTCON(CSU3Pin,tconValue);
  Serial.print(tconPort3Address);
  Serial.print("\ttconPort3Address\tU36\tPort 3\tJ1-3\t");
  Serial.println(tconValue);

  EEPROM.get(tconPort4Address,tconValue);
  setTCON(CSU4Pin,tconValue);
  Serial.print(tconPort4Address);
  Serial.print("\ttconPort4Address\tU39\tPort 4\tJ1-4\t");
  Serial.println(tconValue);

  EEPROM.get(tconPort5Address,tconValue);
  setTCON(CSU5Pin,tconValue);
  Serial.print(tconPort5Address);
  Serial.print("\ttconPort5Address\tU37\tPort 5\tJ1-5\t");
  Serial.println(tconValue);

  EEPROM.get(tconPort6Address,tconValue);
  setTCON(CSU6Pin,tconValue);
  Serial.print(tconPort6Address);
  Serial.print("\ttconPort6Address\tU40\tPort 6\tJ1-6\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort7Address,tconValue);
  setTCON(CSU7Pin,tconValue);
  Serial.print(tconPort7Address);
  Serial.print("\ttconPort7Address\tU45\tPort 7\tJ1-7\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort8Address,tconValue);
  setTCON(CSU8Pin,tconValue);
  Serial.print(tconPort8Address);
  Serial.print("\ttconPort8Address\tU8\tPort 8\tJ1-8\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort9Address,tconValue);
  setTCON(CSU9Pin,tconValue);
  Serial.print(tconPort9Address);
  Serial.print("\ttconPort9Address\tU9\tPort 9\tJ1-9\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort10Address,tconValue);
  setTCON(CSU10Pin,tconValue);
  Serial.print(tconPort10Address);
  Serial.print("\ttconPort10Address\tU47\tPort 10\tJ1-10\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort11Address,tconValue);
  setTCON(CSU11Pin,tconValue);
  Serial.print(tconPort11Address);
  Serial.print("\ttconPort11Address\tU46\tPort 11\tJ5-6\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort12Address,tconValue);
  setTCON(CSU12Pin,tconValue);
  Serial.print(tconPort12Address);
  Serial.print("\ttconPort12Address\tU48\tPort 12\tJ5-12\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort13Address,tconValue);
  setTCON(CSU13Pin,tconValue);
  Serial.print(tconPort13Address);
  Serial.print("\ttconPort13Address\tU53\tPort 13\tJ5-13\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort14Address,tconValue);
  setTCON(CSU14Pin,tconValue);
  Serial.print(tconPort14Address);
  Serial.print("\ttconPort14Address\tU56\tPort 14\tJ5-14\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort15Address,tconValue);
  setTCON(CSU15Pin,tconValue);
  Serial.print(tconPort15Address);
  Serial.print("\ttconPort15Address\tU36\tPort 15\tJ5-15\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort16Address,tconValue);
  setTCON(CSU16Pin,tconValue);
  Serial.print(tconPort16Address);
  Serial.print("\ttconPort16Address\tU57\tPort 16\tJ5-16\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort17Address,tconValue);
  setTCON(CSU17Pin,tconValue);
  Serial.print(tconPort17Address);
  Serial.print("\ttconPort17Address\tU55\tPort 17\tJ5-17\t");
  Serial.println(tconValue);


  EEPROM.get(tconPort18Address,tconValue);
  setTCON(CSU18Pin,tconValue);
  Serial.print(tconPort18Address);
  Serial.print("\ttconPort18Address\tU58\tPort 18\tJ5-18\t");
  Serial.println(tconValue);

  setDAC();

  Serial.println();
  Serial.println("Setting up Switches to +12V and GND");
  Serial.println("Addr. (DEC)\tAddr. Name\tChip or Resistor\tSchematic Port\tWire Connector\tValue");
  
  uint8_t switchValue = 0;
  
  EEPROM.get(Select12VPort1Address,switchValue);
  digitalWrite(Select12VPort1Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort1Address);
  Serial.print("\tSelect12VPort1Address\tR95\tPort 1\tJ1-1\t");
  Serial.println(switchValue);
  
  EEPROM.get(Select12VPort2Address,switchValue);
  digitalWrite(Select12VPort2Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort2Address);
  Serial.print("\tSelect12VPort2Address\tR96\tPort 2\tJ1-2\t");
  Serial.println(switchValue);
  
  EEPROM.get(Select12VPort3Address,switchValue);
  digitalWrite(Select12VPort3Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort3Address);
  Serial.print("\tSelect12VPort3Address\tR97\tPort 3\tJ1-3\t");
  Serial.println(switchValue);
 
  EEPROM.get(Select12VPort4Address,switchValue);
  digitalWrite(Select12VPort4Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort4Address);
  Serial.print("\tSelect12VPort4Address\tR98\tPort 4\tJ1-4\t");
  Serial.println(switchValue);

  EEPROM.get(Select12VPort7Address,switchValue);
  digitalWrite(Select12VPort7Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort7Address);
  Serial.print("\tSelect12VPort7Address\tR107\tPort 7\tJ1-7\t");
  Serial.println(switchValue);
 
  EEPROM.get(Select12VPort8Address,switchValue);
  digitalWrite(Select12VPort8Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort8Address);
  Serial.print("\tSelect12VPort8Address\tR108\tPort 8\tJ1-8\t");
  Serial.println(switchValue);
 
  EEPROM.get(Select12VPort9Address,switchValue);
  digitalWrite(Select12VPort9Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort9Address);
  Serial.print("\tSelect12VPort9Address\tR109\tPort 9\tJ1-9\t");
  Serial.println(switchValue);

  EEPROM.get(Select12VPort10Address,switchValue);
  digitalWrite(Select12VPort10Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort10Address);
  Serial.print("\tSelect12VPort10Address\tR110\tPort 10\tJ1-10\t");
  Serial.println(switchValue);

  EEPROM.get(CAN4Term1Address,switchValue);
  digitalWrite(CAN4Term1Pin,constrain(switchValue,0,1));
  Serial.print(CAN4Term1Address);
  Serial.print("\tCAN4Term1Address\tR54\tCAN 4P\tJ5-7\t");
  Serial.println(switchValue);

  EEPROM.get(CAN4Term2Address,switchValue);
  digitalWrite(CAN4Term2Pin,constrain(switchValue,0,1));
  Serial.print(CAN4Term2Address);
  Serial.print("\tCAN4Term2Address\tR55\tCAN 4N\tJ5-8\t");
  Serial.println(switchValue);

  EEPROM.get(CAN5Term1Address,switchValue);
  digitalWrite(CAN5Term1Pin,constrain(switchValue,0,1));
  Serial.print(CAN5Term1Address);
  Serial.print("\tCAN5Term1Address\tR56\tCAN 5P\tJ5-21\t");
  Serial.println(switchValue);

  EEPROM.get(CAN5Term2Address,switchValue);
  digitalWrite(CAN5Term2Pin,constrain(switchValue,0,1));
  Serial.print(CAN5Term2Address);
  Serial.print("\tCAN5Term2Address\tR57\tCAN 5N\tJ5-22\t");
  Serial.println(switchValue);

  EEPROM.get(J1939toCAN4Address,switchValue);
  digitalWrite(J1939toCAN4Pin,constrain(switchValue,0,1));
  Serial.print(J1939toCAN4Address);
  Serial.print("\tJ1939toCAN4Address\tU28\tCAN2\tJ5-4\t");
  Serial.println(switchValue);

  EEPROM.get(J1939toCAN5Address,switchValue);
  digitalWrite(J1939toCAN5Pin,constrain(switchValue,0,1));
  Serial.print(J1939toCAN5Address);
  Serial.print("\tJ1939toCAN5Address\tU28\tJ1939\tP1-3\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU1Address,switchValue);
  digitalWrite(LowRSelectU1Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU1Address);
  Serial.print("\tLowRSelectU1Address\tR86\tPort 1\tJ1-1\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU2Address,switchValue);
  digitalWrite(LowRSelectU2Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU2Address);
  Serial.print("\tLowRSelectU2Address\tR94\tPort 2\tJ1-2\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU3Address,switchValue);
  digitalWrite(LowRSelectU3Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU3Address);
  Serial.print("\tLowRSelectU3Address\tR87\tPort 3\tJ1-3\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU4Address,switchValue);
  digitalWrite(LowRSelectU4Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU4Address);
  Serial.print("\tLowRSelectU4Address\tR92\tPort 4\tJ1-4\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU5Address,switchValue);
  digitalWrite(LowRSelectU5Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU5Address);
  Serial.print("\tLowRSelectU5Address\tR88\tPort 4\tJ1-5\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU6Address,switchValue);
  digitalWrite(LowRSelectU6Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU6Address);
  Serial.print("\tLowRSelectU6Address\tR93\tPort 6\tJ1-6\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU7Address,switchValue);
  digitalWrite(LowRSelectU7Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU7Address);
  Serial.print("\tLowRSelectU7Address\tR102\tPort 7\tJ1-7\t");
  Serial.println(switchValue);

  EEPROM.get(LowRSelectU8Address,switchValue);
  digitalWrite(LowRSelectU8Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU8Address);
  Serial.print("\tLowRSelectU8Address\tR106\tPort 8\tJ1-8\t");
  Serial.println(switchValue);

  EEPROM.get(GroundSelectU12Address,switchValue);
  digitalWrite(GroundSelectU12Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU12Address);
  Serial.print("\tGroundSelectU12Address\tQ14B\tPort 12\tJ5-12\t");
  Serial.println(switchValue);

  EEPROM.get(GroundSelectU11Address,switchValue);
  digitalWrite(GroundSelectU11Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU11Address);
  Serial.print("\tGroundSelectU11Address\tQ14A\tPort 11\tJ5-6\t");
  Serial.println(switchValue);

  EEPROM.get(GroundSelectU10Address,switchValue);
  digitalWrite(GroundSelectU10Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU10Address);
  Serial.print("\tGroundSelectU10Address\tQ13A\tPort 10\tJ1-10\t");
  Serial.println(switchValue);

  EEPROM.get(GroundSelectU9Address,switchValue);
  digitalWrite(GroundSelectU9Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU9Address);
  Serial.print("\tGroundSelectU9Address\tQ13A\tPort 9\tJ1-9\t");
  Serial.println(switchValue);

  EEPROM.get(U1_U2_12VSelectAddress,switchValue);
  digitalWrite(U1_U2_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U1_U2_12VSelectAddress);
  Serial.print("\tU1_U2_12VSelectAddress\tU32-6\tPort 1\tPort 2\t");
  Serial.println(switchValue);

  EEPROM.get(U3_U4_12VSelectAddress,switchValue);
  digitalWrite(U3_U4_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U3_U4_12VSelectAddress);
  Serial.print("\tU3_U4_12VSelectAddress\tU33-6\tPort 3\tPort 4\t");
  Serial.println(switchValue);

  EEPROM.get(U5_U6_12VSelectAddress,switchValue);
  digitalWrite(U5_U6_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U5_U6_12VSelectAddress);
  Serial.print("\tU5_U6_12VSelectAddress\tU34-6\tPort 5\tPort 6\t");
  Serial.println(switchValue);

  EEPROM.get(U7_U8_12VSelectAddress,switchValue);
  digitalWrite(U7_U8_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U7_U8_12VSelectAddress);
  Serial.print("\tU7_U8_12VSelectAddress\tU42-6\tPort 7\tPort 8\t");
  Serial.println(switchValue);

  EEPROM.get(U9_U10_12VSelectAddress,switchValue);
  digitalWrite(U9_U10_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U9_U10_12VSelectAddress);
  Serial.print("\tU9_U10_12VSelectAddress\tU43-6\tPort 9\tPort 10\t");
  Serial.println(switchValue);

  EEPROM.get(U11_U12_12VSelectAddress,switchValue);
  digitalWrite(U11_U12_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U11_U12_12VSelectAddress);
  Serial.print("\tU11_U12_12VSelectAddress\tU44-6\tPort 11\tPort 12\t");
  Serial.println(switchValue);

  EEPROM.get(U13_U14_12VSelectAddress,switchValue);
  digitalWrite(U13_U14_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U13_U14_12VSelectAddress);
  Serial.print("\tU13_U14_12VSelectAddress\tU51-6\tPort 13\tPort 14\t");
  Serial.println(switchValue);

  EEPROM.get(U15_U16_12VSelectAddress,switchValue);
  digitalWrite(U15_U16_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U15_U16_12VSelectAddress);
  Serial.print("\tU15_U16_12VSelectAddress\tU52-6\tPort 15\tPort 16\t");
  Serial.println(switchValue);

  EEPROM.get(U17_U18_12VSelectAddress,switchValue);
  digitalWrite(U17_U18_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U17_U18_12VSelectAddress);
  Serial.print("\tU17_U18_12VSelectAddress\tU50-6\tPort 17\tPort 18\t");
  Serial.println(switchValue);

 
  //Set PWM Values
  Serial.println();
  Serial.println("Setting up PWM Values");
  Serial.println("Addr. (DEC)\tAddr. Name\tResistor\tPort Name\tWire Connector\tValue");
  uint8_t PWMValue = 0;
  
  EEPROM.get(PWM1Address,PWMValue);
  analogWrite(PWM1Pin,PWMValue);
  Serial.print(PWM1Address);
  Serial.print("\tPWM1Address\tR117\tPWM 1\tJ5-10\t");
  Serial.println(PWMValue);
  
  EEPROM.get(PWM2Address,PWMValue);
  analogWrite(PWM2Pin,PWMValue);
  Serial.print(PWM2Address);
  Serial.print("\tPWM2Address\tR40\tPWM 2\tJ1-18\t");
  Serial.println(PWMValue);
  
  EEPROM.get(PWM3Address,PWMValue);
  analogWrite(PWM3Pin,PWMValue);
  Serial.print(PWM3Address);
  Serial.print("\tPWM3Address\tR8\tPWM 3\tJ1-19\t");
  Serial.println(PWMValue);

  EEPROM.get(componentIDaddress,componentID);
  Serial.print("Component ID: ");
  Serial.println(componentID);
  
  Serial.println("Daughter Board Program for ATmega2560 Processor.");
  Serial.println("Finished Starting Up... ");
  
}


void loop(){
  //Check for new commands on the serial bus
  if (Serial.available()>0) 
  {
    int nDataBytes = Serial.readBytesUntil(separatorChar,command,99);
    processCommand(nDataBytes);
    memset(command,0,sizeof(command));
  }
  
  
} //end loop()


void setWiper(int CSPin,int wiperValue){
  //write to wiper register is all zeros for the 6 most significant bits in the address field
  uint8_t address = 0;
  address = address | ((wiperValue & 0b0000001100000000) >> 8); // use bits 9 and 10 for the values of the address.
  uint8_t data = wiperValue & 0x00FF;
  
  //Set the wiper value for a MCP41HV
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPin,LOW); 
  SPI.transfer(address);
  SPI.transfer(data);
  // take the SS pin high to de-select the chip:
  digitalWrite(CSPin,HIGH);
  // release control of the SPI port
  SPI.endTransaction();
}

void setTCON(int CSPin,int tconValue){
  //Set the terminal connection for a MCP41HV 
  uint8_t address = 0x40;
  address = address | ((wiperValue & 0b0000001100000000) >> 8); // use bits 9 and 10 for the values of the address.
  uint8_t data = tconValue & 0x00FF;
  
  //Set the wiper value for a MCP41HV
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPin,LOW); 
  SPI.transfer(address);
  SPI.transfer(data);
  // take the SS pin high to de-select the chip:
  digitalWrite(CSPin,HIGH);
  // release control of the SPI port
  SPI.endTransaction();
}
  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setDAC(){  //Settings are close to millivolts.
  Serial.println();
  Serial.println("Setting up analog to Digital Converters...");
  Serial.println("Addr. (DEC)\tAddr. Name\tChip-Pin\tSchematic Port\tWire Connector\tValue");
  
  uint16_t VoutA;
  uint16_t VoutB; 
  uint16_t VoutC;
  uint16_t VoutD; 
  uint16_t VoutE;
  uint16_t VoutF;
  uint16_t VoutG;
  uint16_t VoutH;
  uint16_t VoutI;
  uint16_t VoutJ;
  uint16_t VoutK;
  uint16_t VoutL;
  
  EEPROM.get(VoutAaddress,VoutA);
  Serial.print(VoutAaddress);
  Serial.print("\tVoutAaddress\tU4-Pin6\tVoutA\tJ5-19\t");
  Serial.println(VoutA);
  
  EEPROM.get(VoutBaddress,VoutB);
  Serial.print(VoutBaddress);
  Serial.print("\tVoutBaddress\tU4-Pin7\tVoutB\tJ5-20\t");
  Serial.println(VoutB);
  
  EEPROM.get(VoutCaddress,VoutC);
  Serial.print(VoutCaddress);
  Serial.print("\tVoutCaddress\tU4-Pin8\tVoutC\tJ5-1\t");
  Serial.println(VoutC);
  
  EEPROM.get(VoutDaddress,VoutD);
  Serial.print(VoutDaddress);
  Serial.print("\tVoutDaddress\tU4-Pin9\tVoutD\tJ5-2\t");
  Serial.println(VoutD);
  
  EEPROM.get(VoutEaddress,VoutE);
  Serial.print(VoutEaddress);
  Serial.print("\tVoutEaddress\tU6-Pin6\tVoutE\tJ1-12\t");
  Serial.println(VoutE);
  
  EEPROM.get(VoutFaddress,VoutF);
  Serial.print(VoutFaddress);
  Serial.print("\tVoutFaddress\tU6-Pin7\tVoutF\tJ1-13\t");
  Serial.println(VoutF);
  
  EEPROM.get(VoutGaddress,VoutG);
  Serial.print(VoutGaddress);
  Serial.print("\tVoutGaddress\tU6-Pin8\tVoutG\tJ1-14\t");
  Serial.println(VoutG);
  
  EEPROM.get(VoutHaddress,VoutH);
  Serial.print(VoutHaddress);
  Serial.print("\tVoutHaddress\tU6-Pin9\tVoutH\tJ1-15\t");
  Serial.println(VoutH);
  
  EEPROM.get(VoutIaddress,VoutI);
  Serial.print(VoutIaddress);
  Serial.print("\tVoutIaddress\tU7-Pin6\tVoutI x 2.5\tJ1-16\t");
  Serial.println(VoutI);
  
  EEPROM.get(VoutJaddress,VoutJ);
  Serial.print(VoutJaddress);
  Serial.print("\tVoutJaddress\tU7-Pin7\tVoutJ x 2.5\tJ1-17\t");
  Serial.println(VoutJ);
  
  EEPROM.get(VoutKaddress,VoutK);
  Serial.print(VoutKaddress);
  Serial.print("\tVoutKaddress\tU7-Pin8\tVoutK x 2.5\tJ2-1\t");
  Serial.println(VoutK);
  
  EEPROM.get(VoutLaddress,VoutL);
  Serial.print(VoutLaddress);
  Serial.print("\tVoutLaddress\tU7-Pin9\tVoutL x 2.5\tJ2-2\t");
  Serial.println(VoutL);
  
  
  
  
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


//
//void sendCANmessages()
//{
//  if (isIgnitionOn && numCANmsgs >0 ){
//    for (int i = 0; i < numCANmsgs; i++){
//      unsigned long currentMillis = millis();
//      if ((currentMillis - previousCANmillis[i]) > CANtxPeriod[i]) { // Do this on time.
//        previousCANmillis[i] = currentMillis;
//        CAN4.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
//     
//      } //end if
//    } // end for
//  } // end if
//}
//  

void processCommand(int numDataBytes){
  Serial.print("Command: ");
  for (int t=0;t<numDataBytes;t++) Serial.print(command[t]);
  Serial.println();
  
  boolean addressValid = true;
  boolean dataValid = true;
  //Example command: set,34,12 or set 34 12 
  if  ((command[0] == 's' || command[0]=='S') && (command[1] == 'e' || command[1] == 'E') && (command[2] =='t' || command[2] == 'T')){ 
    char addressString[5];
    memset(addressString,0,5);
    uint8_t q = 4;
    uint8_t p = 0;
    while (isdigit(command[q]) && q < 8){
      addressString[p] = command[q];
      q++;
      p++;
    } 
    if (q > 4){
      uint16_t address = atoi(addressString);
      char valueString[8];
      memset(valueString,0,8);
      p = 0;
      q+=1;
      while (isdigit(command[q]) && q < numDataBytes )
      {
        valueString[p] = command[q];
        q++;
        p++;
      }
      if (p > 0)
      {
        uint16_t value = atoi(valueString);
        if (value>255) EEPROM.put(address,uint16_t(value));
        else EEPROM.update(address,uint8_t(value));
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
  else if ( (command[0] == 'r' || command[0]=='R') && (command[1] == 'e' || command[1] == 'E') && (command[2] =='S' || command[2] == 's'))
  {
    setup();  
  }
  else if ( (command[0] == 'i' || command[0]=='I') && (command[1] == 'D' || command[1] == 'd'))
  {
    memset(componentID,0,sizeof(componentID));
    uint8_t q=3;
    while (isprint(command[q]) && q < 35){
      componentID[q-3] = command[q];
      q++;
    }
    if (q > 3)
    {
      EEPROM.put(componentIDaddress,componentID);
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
   
 /* 
  
  //Process CAN Instructions
  else if ( (command[0] == 'c' || command[0]=='C') && (command[1] == 'a' || command[1] == 'A') && (command[2] =='N' || command[2] == 'n')){
    Serial.println("CAN Command. Make first byte on ID = 0xF0 to reset CAN messages.");
    // if (command[3]=='8'){
      // Serial.print("Sending CAN Reset command.");
      // I2C.beginTransmission(4); //Address the primary processor that is listening for 14 bytes
      // for (int i=0; i<14;i++) I2C.write(0x80);
      // int flag = I2C.endTransmission(); 
      // numCANmsgs=0;
    // } 
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
 
        
        buildCANmessage();
        
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
  */
  
  else 
  {
    Serial.println("INPUT ERROR: Command not recognized.");
  }
      
}


void setEEPROM(){
  if (EEPROM.read(0)!=1){
    EEPROM.write(0,1);
  
    EEPROM.update(wiperPort1Address,uint8_t(40));
    EEPROM.update(wiperPort2Address,uint8_t(60));
    EEPROM.update(wiperPort3Address,uint8_t(80));
    EEPROM.update(wiperPort4Address,uint8_t(100));
    EEPROM.update(wiperPort5Address,uint8_t(120));
    EEPROM.update(wiperPort6Address,uint8_t(140));
    EEPROM.update(wiperPort7Address,uint8_t(160));
    EEPROM.update(wiperPort8Address,uint8_t(180));
    EEPROM.update(wiperPort9Address,uint8_t(200));
    EEPROM.update(wiperPort10Address,uint8_t(220));
    EEPROM.update(wiperPort11Address,uint8_t(30));
    EEPROM.update(wiperPort12Address,uint8_t(50));
    EEPROM.update(wiperPort13Address,uint8_t(70));
    EEPROM.update(wiperPort14Address,uint8_t(90));
    EEPROM.update(wiperPort15Address,uint8_t(110));
    EEPROM.update(wiperPort16Address,uint8_t(130));
    EEPROM.update(wiperPort17Address,uint8_t(150));
    EEPROM.update(wiperPort18Address,uint8_t(170));
  
    EEPROM.update(tconPort1Address,uint8_t(15));
    EEPROM.update(tconPort2Address,uint8_t(15));
    EEPROM.update(tconPort3Address,uint8_t(15));
    EEPROM.update(tconPort4Address,uint8_t(15));
    EEPROM.update(tconPort5Address,uint8_t(15));
    EEPROM.update(tconPort6Address,uint8_t(15));
    EEPROM.update(tconPort7Address,uint8_t(15));
    EEPROM.update(tconPort8Address,uint8_t(15));
    EEPROM.update(tconPort9Address,uint8_t(15));
    EEPROM.update(tconPort10Address,uint8_t(15));
    EEPROM.update(tconPort11Address,uint8_t(15));
    EEPROM.update(tconPort12Address,uint8_t(15));
    EEPROM.update(tconPort13Address,uint8_t(15));
    EEPROM.update(tconPort14Address,uint8_t(15));
    EEPROM.update(tconPort15Address,uint8_t(15));
    EEPROM.update(tconPort16Address,uint8_t(15));
    EEPROM.update(tconPort17Address,uint8_t(15));
    EEPROM.update(tconPort18Address,uint8_t(15));
    
    EEPROM.put(VoutAaddress,uint16_t(500));
    EEPROM.put(VoutBaddress,uint16_t(800));
    EEPROM.put(VoutCaddress,uint16_t(1100));
    EEPROM.put(VoutDaddress,uint16_t(1400));
    EEPROM.put(VoutEaddress,uint16_t(1700));
    EEPROM.put(VoutFaddress,uint16_t(2000));
    EEPROM.put(VoutGaddress,uint16_t(2300));
    EEPROM.put(VoutHaddress,uint16_t(2600));
    EEPROM.put(VoutIaddress,uint16_t(100));
    EEPROM.put(VoutJaddress,uint16_t(400));
    EEPROM.put(VoutKaddress,uint16_t(800));
    EEPROM.put(VoutLaddress,uint16_t(1300));
  
    EEPROM.update(Select12VPort1Address,uint8_t(0));
    EEPROM.update(Select12VPort2Address,uint8_t(0));
    EEPROM.update(Select12VPort3Address,uint8_t(0));
    EEPROM.update(Select12VPort4Address,uint8_t(0));
  
    EEPROM.update(Select12VPort7Address,uint8_t(0));
    EEPROM.update(Select12VPort8Address,uint8_t(0));
    EEPROM.update(Select12VPort9Address,uint8_t(0));
    EEPROM.update(Select12VPort10Address,uint8_t(0));
   
    EEPROM.update(CAN4Term1Address,uint8_t(1));
    EEPROM.update(CAN4Term2Address,uint8_t(0));
    EEPROM.update(CAN5Term1Address,uint8_t(1));
    EEPROM.update(CAN5Term2Address,uint8_t(0));
    EEPROM.update(J1939toCAN4Address,uint8_t(0));
    EEPROM.update(J1939toCAN5Address,uint8_t(0));
  
    EEPROM.update(LowRSelectU1Address,uint8_t(0));
    EEPROM.update(LowRSelectU2Address,uint8_t(0));
    EEPROM.update(LowRSelectU3Address,uint8_t(0));
    EEPROM.update(LowRSelectU4Address,uint8_t(0));
    EEPROM.update(LowRSelectU5Address,uint8_t(0));
    EEPROM.update(LowRSelectU6Address,uint8_t(0));
    EEPROM.update(LowRSelectU7Address,uint8_t(0));
    EEPROM.update(LowRSelectU8Address,uint8_t(0));
  
    EEPROM.update(GroundSelectU12Address,uint8_t(0));
    EEPROM.update(GroundSelectU11Address,uint8_t(0));
    EEPROM.update(GroundSelectU10Address,uint8_t(0));
    EEPROM.update(GroundSelectU9Address,uint8_t(0));
  
    EEPROM.update(U1_U2_12VSelectAddress,uint8_t(1));
    EEPROM.update(U3_U4_12VSelectAddress,uint8_t(1));
    EEPROM.update(U5_U6_12VSelectAddress,uint8_t(1));
    EEPROM.update(U7_U8_12VSelectAddress,uint8_t(1));
    EEPROM.update(U9_U10_12VSelectAddress,uint8_t(1));
    EEPROM.update(U11_U12_12VSelectAddress,uint8_t(1));
    EEPROM.update(U13_U14_12VSelectAddress,uint8_t(1));
    EEPROM.update(U15_U16_12VSelectAddress,uint8_t(1));
    EEPROM.update(U17_U18_12VSelectAddress,uint8_t(1));
  
  
  
    EEPROM.update(PWM1Address,uint8_t(64));
    EEPROM.update(PWM2Address,uint8_t(127));
    EEPROM.update(PWM3Address,uint8_t(192));

    EEPROM.put(componentIDaddress,componentID);
    EEPROM.put(sssSAaddress,SAddress);
    
    Serial.println("New EEPROM values set.");
  }
  else
  {
    Serial.println("EEPROM values already set.");
  }
}



