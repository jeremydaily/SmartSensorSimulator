//Define Pin Numbers for the ChipSelect for CAN
#define CSCAN4Pin 17
#define CSCAN5Pin 53

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

const uint16_t wiperAddresses[18] = {
    wiperPort1Address, 
    wiperPort2Address,
    wiperPort3Address,
    wiperPort4Address,
    wiperPort5Address,
    wiperPort6Address,
    wiperPort7Address,
    wiperPort8Address,
    wiperPort9Address,
    wiperPort10Address,
    wiperPort11Address,
    wiperPort12Address,
    wiperPort13Address,
    wiperPort14Address,
    wiperPort15Address,
    wiperPort16Address,
    wiperPort17Address,
    wiperPort18Address};

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

const uint16_t tconAddresses[18] = {
    tconPort1Address, 
    tconPort2Address,
    tconPort3Address,
    tconPort4Address,
    tconPort5Address,
    tconPort6Address,
    tconPort7Address,
    tconPort8Address,
    tconPort9Address,
    tconPort10Address,
    tconPort11Address,
    tconPort12Address,
    tconPort13Address,
    tconPort14Address,
    tconPort15Address,
    tconPort16Address,
    tconPort17Address,
    tconPort18Address};

    
#define LINCSAddress   73
#define LINWakeAddress   74


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

#define U1_U2_12VSelectAddress 70
#define U3_U4_12VSelectAddress 51
#define U5_U6_12VSelectAddress 52
#define U7_U8_12VSelectAddress 53
#define U9_U10_12VSelectAddress 54
#define U11_U12_12VSelectAddress 55
#define U13_U14_12VSelectAddress 9
#define U15_U16_12VSelectAddress 10
#define U17_U18_12VSelectAddress 11

#define PWM1Address  3
#define PWM2Address  1
#define PWM3Address  5

#define CANmessageAddressStart 1024
#define CANmessageLength 31

const uint16_t switchAddresses[38] = {
    Select12VPort1Address,
    Select12VPort2Address,
    Select12VPort3Address,
    Select12VPort4Address,
    Select12VPort7Address,
    Select12VPort8Address,
    Select12VPort9Address,
    Select12VPort10Address,
    CAN4Term1Address,
    CAN4Term2Address,
    CAN5Term1Address,
    CAN5Term2Address,
    J1939toCAN4Address,
    J1939toCAN5Address,
    LowRSelectU1Address,
    LowRSelectU2Address,
    LowRSelectU3Address,
    LowRSelectU4Address,
    LowRSelectU5Address,
    LowRSelectU6Address,
    LowRSelectU7Address,
    LowRSelectU8Address,
    GroundSelectU11Address,
    GroundSelectU12Address,
    GroundSelectU10Address,
    GroundSelectU9Address,
    U1_U2_12VSelectAddress,
    U3_U4_12VSelectAddress,
    U5_U6_12VSelectAddress,
    U7_U8_12VSelectAddress,
    U9_U10_12VSelectAddress,
    U11_U12_12VSelectAddress,
    U13_U14_12VSelectAddress,
    U15_U16_12VSelectAddress,
    U17_U18_12VSelectAddress,
    PWM1Address,
    PWM2Address,
    PWM3Address};

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




void setWiper(int CSPin,uint8_t wiperValue){
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

void setTCON(int CSPin,uint8_t tconValue){
  //Set the terminal connection for a MCP41HV 
  uint8_t address = 0x40;
  address = address | ((tconValue & 0b0000001100000000) >> 8); // use bits 9 and 10 for the values of the address.
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

void adjustWiperValues(uint16_t address,uint8_t wiperValue){  
  
  if (address == wiperPort1Address){
  setWiper(CSU1Pin,wiperValue);
  Serial.print(wiperPort1Address);
  Serial.print("\twiperPort1Address\tU35-P0W\tPort 1\tJ1-1\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort2Address){
  setWiper(CSU2Pin,wiperValue);
  Serial.print(wiperPort2Address);
  Serial.print("\twiperPort2Address\tU38-P0W\tPort 2\tJ1-2\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort3Address){
  setWiper(CSU3Pin,wiperValue);
  Serial.print(wiperPort3Address);
  Serial.print("\twiperPort3Address\tU36-P0W\tPort 3\tJ1-3\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort4Address){
  setWiper(CSU4Pin,wiperValue);
  Serial.print(wiperPort4Address);
  Serial.print("\twiperPort4Address\tU39-P0W\tPort 4\tJ1-4\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort5Address){
  setWiper(CSU5Pin,wiperValue);
  Serial.print(wiperPort5Address);
  Serial.print("\twiperPort4Address\tU37-P0W\tPort 5\tJ1-5\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort6Address){
  setWiper(CSU6Pin,wiperValue);
  Serial.print(wiperPort6Address);
  Serial.print("\twiperPort6Address\tU40-P0W\tPort 6\tJ1-6\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort7Address){
  setWiper(CSU7Pin,wiperValue);
  Serial.print(wiperPort7Address);
  Serial.print("\twiperPort7Address\tU45-P0W\tPort 7\tJ1-7\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort8Address){
  setWiper(CSU8Pin,wiperValue);
  Serial.print(wiperPort8Address);
  Serial.print("\twiperPort8Address\tU8-P0W\tPort 8\tJ1-8\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort9Address){
  setWiper(CSU9Pin,wiperValue);
  Serial.print(wiperPort9Address);
  Serial.print("\twiperPort9Address\tU9-P0W\tPort 9\tJ1-9\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort10Address){
  setWiper(CSU10Pin,wiperValue);
  Serial.print(wiperPort10Address);
  Serial.print("\twiperPort10Address\tU47-P0W\tPort 10\tJ1-10\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort11Address){
  setWiper(CSU11Pin,wiperValue);
  Serial.print(wiperPort11Address);
  Serial.print("\twiperPort11Address\tU46-P0W\tPort 11\tJ5-6\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort12Address){
  setWiper(CSU12Pin,wiperValue);
  Serial.print(wiperPort12Address);
  Serial.print("\twiperPort12Address\tU48-P0W\tPort 12\tJ5-12\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort13Address){
  setWiper(CSU13Pin,wiperValue);
  Serial.print(wiperPort13Address);
  Serial.print("\twiperPort13Address\tU53-P0W\tPort 13\tJ5-13\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort14Address){
  setWiper(CSU14Pin,wiperValue);
  Serial.print(wiperPort14Address);
  Serial.print("\twiperPort14Address\tU56-P0W\tPort 14\tJ5-14\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort15Address){
  setWiper(CSU15Pin,wiperValue);
  Serial.print(wiperPort15Address);
  Serial.print("\twiperPort15Address\tU36-P0W\tPort 15\tJ5-15\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort16Address){
  setWiper(CSU16Pin,wiperValue);
  Serial.print(wiperPort16Address);
  Serial.print("\twiperPort16Address\tU57-P0W\tPort 16\tJ5-16\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort17Address){
  setWiper(CSU17Pin,wiperValue);
  Serial.print(wiperPort17Address);
  Serial.print("\twiperPort17Address\tU55-P0W\tPort 17\tJ5-17\t");
  Serial.println(wiperValue);
  }
  else if (address == wiperPort18Address){
  setWiper(CSU18Pin,wiperValue);
  Serial.print(wiperPort18Address);
  Serial.print("\twiperPort18Address\tU58-P0W\tPort 18\tJ5-18\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort1Address){
  setTCON(CSU1Pin,wiperValue);
  Serial.print(tconPort1Address);
  Serial.print("\ttconPort1Address\tU35\tPort 1\tJ1-1\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort2Address){
  setTCON(CSU2Pin,wiperValue);
  Serial.print(tconPort2Address);
  Serial.print("\ttconPort2Address\tU38\tPort 2\tJ1-2\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort3Address){
  setTCON(CSU3Pin,wiperValue);
  Serial.print(tconPort3Address);
  Serial.print("\ttconPort3Address\tU36\tPort 3\tJ1-3\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort4Address){
  setTCON(CSU4Pin,wiperValue);
  Serial.print(tconPort4Address);
  Serial.print("\ttconPort4Address\tU39\tPort 4\tJ1-4\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort5Address){
  setTCON(CSU5Pin,wiperValue);
  Serial.print(tconPort5Address);
  Serial.print("\ttconPort5Address\tU37\tPort 5\tJ1-5\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort6Address){
  setTCON(CSU6Pin,wiperValue);
  Serial.print(tconPort6Address);
  Serial.print("\ttconPort6Address\tU40\tPort 6\tJ1-6\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort7Address){
  setTCON(CSU7Pin,wiperValue);
  Serial.print(tconPort7Address);
  Serial.print("\ttconPort7Address\tU45\tPort 7\tJ1-7\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort8Address){
  setTCON(CSU8Pin,wiperValue);
  Serial.print(tconPort8Address);
  Serial.print("\ttconPort8Address\tU8\tPort 8\tJ1-8\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort9Address){
  setTCON(CSU9Pin,wiperValue);
  Serial.print(tconPort9Address);
  Serial.print("\ttconPort9Address\tU9\tPort 9\tJ1-9\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort10Address){
  setTCON(CSU10Pin,wiperValue);
  Serial.print(tconPort10Address);
  Serial.print("\ttconPort10Address\tU47\tPort 10\tJ1-10\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort11Address){
  setTCON(CSU11Pin,wiperValue);
  Serial.print(tconPort11Address);
  Serial.print("\ttconPort11Address\tU46\tPort 11\tJ5-6\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort12Address){
  setTCON(CSU12Pin,wiperValue);
  Serial.print(tconPort12Address);
  Serial.print("\ttconPort12Address\tU48\tPort 12\tJ5-12\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort13Address){
  setTCON(CSU13Pin,wiperValue);
  Serial.print(tconPort13Address);
  Serial.print("\ttconPort13Address\tU53\tPort 13\tJ5-13\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort14Address){
  setTCON(CSU14Pin,wiperValue);
  Serial.print(tconPort14Address);
  Serial.print("\ttconPort14Address\tU56\tPort 14\tJ5-14\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort15Address){
  setTCON(CSU15Pin,wiperValue);
  Serial.print(tconPort15Address);
  Serial.print("\ttconPort15Address\tU36\tPort 15\tJ5-15\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort16Address){
  setTCON(CSU16Pin,wiperValue);
  Serial.print(tconPort16Address);
  Serial.print("\ttconPort16Address\tU57\tPort 16\tJ5-16\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort17Address){
  setTCON(CSU17Pin,wiperValue);
  Serial.print(tconPort17Address);
  Serial.print("\ttconPort17Address\tU55\tPort 17\tJ5-17\t");
  Serial.println(wiperValue);
  }
  else if (address == tconPort18Address){
  setTCON(CSU18Pin,wiperValue);
  Serial.print(tconPort18Address);
  Serial.print("\ttconPort18Address\tU58\tPort 18\tJ5-18\t");
  Serial.println(wiperValue);
  }
  else {
    Serial.println("Address not found in adjustWiperValues function.");
  }
}







void adjustSwitchValues(uint16_t address,uint8_t switchValue){  
  if (address == Select12VPort1Address){
  digitalWrite(Select12VPort1Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort1Address);
  Serial.print("\tSelect12VPort1Address\tR95\tPort 1\tJ1-1\t");
  Serial.println(switchValue);
  }
  else if (address == Select12VPort2Address){
  digitalWrite(Select12VPort2Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort2Address);
  Serial.print("\tSelect12VPort2Address\tR96\tPort 2\tJ1-2\t");
  Serial.println(switchValue);
  }
  else if (address == Select12VPort3Address){
  digitalWrite(Select12VPort3Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort3Address);
  Serial.print("\tSelect12VPort3Address\tR97\tPort 3\tJ1-3\t");
  Serial.println(switchValue);
  }
  else if (address == Select12VPort4Address){
  digitalWrite(Select12VPort4Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort4Address);
  Serial.print("\tSelect12VPort4Address\tR98\tPort 4\tJ1-4\t");
  Serial.println(switchValue);
  }
  else if (address == Select12VPort7Address){
  digitalWrite(Select12VPort7Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort7Address);
  Serial.print("\tSelect12VPort7Address\tR107\tPort 7\tJ1-7\t");
  Serial.println(switchValue);
  }
  else if (address == Select12VPort8Address){
  digitalWrite(Select12VPort8Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort8Address);
  Serial.print("\tSelect12VPort8Address\tR108\tPort 8\tJ1-8\t");
  Serial.println(switchValue);
  }
  else if (address == Select12VPort9Address){
  digitalWrite(Select12VPort9Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort9Address);
  Serial.print("\tSelect12VPort9Address\tR109\tPort 9\tJ1-9\t");
  Serial.println(switchValue);
  }
  else if (address == Select12VPort10Address){
  digitalWrite(Select12VPort10Pin,constrain(switchValue,0,1));
  Serial.print(Select12VPort10Address);
  Serial.print("\tSelect12VPort10Address\tR110\tPort 10\tJ1-10\t");
  Serial.println(switchValue);
  }
  else if (address == CAN4Term1Address){
  digitalWrite(CAN4Term1Pin,constrain(switchValue,0,1));
  Serial.print(CAN4Term1Address);
  Serial.print("\tCAN4Term1Address\tR54\tCAN 4P\tJ5-7\t");
  Serial.println(switchValue);
  }
  else if (address == CAN4Term2Address){
  digitalWrite(CAN4Term2Pin,constrain(switchValue,0,1));
  Serial.print(CAN4Term2Address);
  Serial.print("\tCAN4Term2Address\tR55\tCAN 4N\tJ5-8\t");
  Serial.println(switchValue);
  }
  else if (address == CAN5Term1Address){
  digitalWrite(CAN5Term1Pin,constrain(switchValue,0,1));
  Serial.print(CAN5Term1Address);
  Serial.print("\tCAN5Term1Address\tR56\tCAN 5P\tJ5-21\t");
  Serial.println(switchValue);
  }
  else if (address == CAN5Term2Address){
  digitalWrite(CAN5Term2Pin,constrain(switchValue,0,1));
  Serial.print(CAN5Term2Address);
  Serial.print("\tCAN5Term2Address\tR57\tCAN 5N\tJ5-22\t");
  Serial.println(switchValue);
  }
  else if (address == J1939toCAN4Address){
  digitalWrite(J1939toCAN4Pin,constrain(switchValue,0,1));
  Serial.print(J1939toCAN4Address);
  Serial.print("\tJ1939toCAN4Address\tU28\tCAN2\tJ5-4\t");
  Serial.println(switchValue);
  }
  else if (address == J1939toCAN5Address){
  digitalWrite(J1939toCAN5Pin,constrain(switchValue,0,1));
  Serial.print(J1939toCAN5Address);
  Serial.print("\tJ1939toCAN5Address\tU28\tJ1939\tP1-3\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU1Address){
  digitalWrite(LowRSelectU1Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU1Address);
  Serial.print("\tLowRSelectU1Address\tR86\tPort 1\tJ1-1\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU2Address){
  digitalWrite(LowRSelectU2Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU2Address);
  Serial.print("\tLowRSelectU2Address\tR94\tPort 2\tJ1-2\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU3Address){
  digitalWrite(LowRSelectU3Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU3Address);
  Serial.print("\tLowRSelectU3Address\tR87\tPort 3\tJ1-3\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU4Address){
  digitalWrite(LowRSelectU4Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU4Address);
  Serial.print("\tLowRSelectU4Address\tR92\tPort 4\tJ1-4\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU5Address){
  digitalWrite(LowRSelectU5Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU5Address);
  Serial.print("\tLowRSelectU5Address\tR88\tPort 4\tJ1-5\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU6Address){
  digitalWrite(LowRSelectU6Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU6Address);
  Serial.print("\tLowRSelectU6Address\tR93\tPort 6\tJ1-6\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU7Address){
  digitalWrite(LowRSelectU7Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU7Address);
  Serial.print("\tLowRSelectU7Address\tR102\tPort 7\tJ1-7\t");
  Serial.println(switchValue);
  }
  else if (address == LowRSelectU8Address){
  digitalWrite(LowRSelectU8Pin,constrain(switchValue,0,1));
  Serial.print(LowRSelectU8Address);
  Serial.print("\tLowRSelectU8Address\tR106\tPort 8\tJ1-8\t");
  Serial.println(switchValue);
  }
  else if (address == GroundSelectU12Address){
  digitalWrite(GroundSelectU12Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU12Address);
  Serial.print("\tGroundSelectU12Address\tQ14B\tPort 12\tJ5-12\t");
  Serial.println(switchValue);
  }
  else if (address == GroundSelectU11Address){
  digitalWrite(GroundSelectU11Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU11Address);
  Serial.print("\tGroundSelectU11Address\tQ14A\tPort 11\tJ5-6\t");
  Serial.println(switchValue);
  }
  else if (address == GroundSelectU10Address){
  digitalWrite(GroundSelectU10Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU10Address);
  Serial.print("\tGroundSelectU10Address\tQ13A\tPort 10\tJ1-10\t");
  Serial.println(switchValue);
  }
  else if (address == GroundSelectU9Address){
  digitalWrite(GroundSelectU9Pin,constrain(switchValue,0,1));
  Serial.print(GroundSelectU9Address);
  Serial.print("\tGroundSelectU9Address\tQ13A\tPort 9\tJ1-9\t");
  Serial.println(switchValue);
  }
  else if (address == U1_U2_12VSelectAddress){
  digitalWrite(U1_U2_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U1_U2_12VSelectAddress);
  Serial.print("\tU1_U2_12VSelectAddress\tU32-6\tPort 1\tPort 2\t");
  Serial.println(switchValue);
  }
  else if (address == U3_U4_12VSelectAddress){
  digitalWrite(U3_U4_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U3_U4_12VSelectAddress);
  Serial.print("\tU3_U4_12VSelectAddress\tU33-6\tPort 3\tPort 4\t");
  Serial.println(switchValue);
  }
  else if (address == U5_U6_12VSelectAddress){
  digitalWrite(U5_U6_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U5_U6_12VSelectAddress);
  Serial.print("\tU5_U6_12VSelectAddress\tU34-6\tPort 5\tPort 6\t");
  Serial.println(switchValue);
  }
  else if (address == U7_U8_12VSelectAddress){
  digitalWrite(U7_U8_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U7_U8_12VSelectAddress);
  Serial.print("\tU7_U8_12VSelectAddress\tU42-6\tPort 7\tPort 8\t");
  Serial.println(switchValue);
  }
  else if (address == U9_U10_12VSelectAddress){
  digitalWrite(U9_U10_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U9_U10_12VSelectAddress);
  Serial.print("\tU9_U10_12VSelectAddress\tU43-6\tPort 9\tPort 10\t");
  Serial.println(switchValue);
  }
  else if (address == U11_U12_12VSelectAddress){
  digitalWrite(U11_U12_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U11_U12_12VSelectAddress);
  Serial.print("\tU11_U12_12VSelectAddress\tU44-6\tPort 11\tPort 12\t");
  Serial.println(switchValue);
  }
  else if (address == U13_U14_12VSelectAddress){
  digitalWrite(U13_U14_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U13_U14_12VSelectAddress);
  Serial.print("\tU13_U14_12VSelectAddress\tU51-6\tPort 13\tPort 14\t");
  Serial.println(switchValue);
  }
  else if (address == U15_U16_12VSelectAddress){
  digitalWrite(U15_U16_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U15_U16_12VSelectAddress);
  Serial.print("\tU15_U16_12VSelectAddress\tU52-6\tPort 15\tPort 16\t");
  Serial.println(switchValue);
  }
  else if (address == U17_U18_12VSelectAddress){
  digitalWrite(U17_U18_12VSelectPin,constrain(switchValue,0,1));
  Serial.print(U17_U18_12VSelectAddress);
  Serial.print("\tU17_U18_12VSelectAddress\tU50-6\tPort 17\tPort 18\t");
  Serial.println(switchValue);
  }
  else if (address == PWM1Address){
  analogWrite(PWM1Pin,switchValue);
  Serial.print(PWM1Address);
  Serial.print("\tPWM1Address\tR117\tPWM 1\tJ5-10\t");
  Serial.println(switchValue);
  }
  else if (address == PWM2Address){
  analogWrite(PWM2Pin,switchValue);
  Serial.print(PWM2Address);
  Serial.print("\tPWM2Address\tR40\tPWM 2\tJ1-18\t");
  Serial.println(switchValue);
  }
  else if (address == PWM3Address){
  analogWrite(PWM3Pin,switchValue);
  Serial.print(PWM3Address);
  Serial.print("\tPWM3Address\tR8\tPWM 3\tJ1-19\t");
  Serial.println(switchValue);
  }
  else 
  {
    Serial.println("Address not found in adjustSwitchValues() function.");
  }
}

