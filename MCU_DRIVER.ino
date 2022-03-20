//YTU-08 MCU INITIALIZE WITH ARDUINO
//ANY QUESTIONS: candestan13@gmail.com

/*
 * NOTE: This code only works on Arduino UNO & NANO.
 * This is because of the library which we use. If we edit the libary, it can also work
 * for everything.
*/

/* CABLE MANAGEMENT (Arduino UNO&NANO ~ MCP2515)
 * VCC -> 5V
 * GND -> GND
 * CS  -> D10
 * SO  -> D12
 * SI  -> D11
 * SCK -> D13
 * INT -> D2 (Optional on UNO; Necessary on NANO)
 */

//#####################################
//          USER SETTINGS
#define DEBUGMODE 1

#define SENSOR1_PIN A1
#define SENSOR2_PIN A5

#define CAN_TEST_CHANNEL_ID 0x13

#define MOTOR_NODE_ID 0x01
//#####################################

//#####################################
//          CAN DEFINING AREA
#include <SPI.h>
#include "mcp2515_can.h"
#define SPI_CS_PIN 10 //On older arduinos, it's Digital 10. Older ones use Digital 9.
#define CAN_INT_PIN 2 //Int pin should be empty on UNO but With Nano, it's connected to D2.
#define CAN_SERIAL_BAND 115200 //To use the CAN_BUS, Serial should be 115200 allocated.

mcp2515_can CAN(SPI_CS_PIN);

#define CAN_DATA_LENGTH 8 //It's CAN Protocol's Constant. Can't be changed.
//#####################################

static char MotorNodeID = 0x01;
void setup() {
  Serial.begin(CAN_SERIAL_BAND);
  InitializeCAN();
  while(!Serial)
    delay(100);
  MotorNodeID = CheckMCU();
  SetPreOperationalMode(true);
  SetOperationalMode(true);
  SetPWMMode(true);
}

unsigned char CAN_DATA[CAN_DATA_LENGTH] = {0,0,0,0,0,0,0,0};
void loop() {
  char SENSOR1_DATA = ReadSensor(SENSOR1_PIN);
  char SENSOR2_DATA = ReadSensor(SENSOR2_PIN);
  char TORQUE_DATA1 = SensorDataToTorque(SENSOR1_DATA);
  char TORQUE_DATA2 = SensorDataToTorque(SENSOR2_DATA);
  CAN_DATA[CAN_DATA_LENGTH - 1] = TORQUE_DATA1 - TORQUE_DATA2;
  SendCANDATA(CAN_TEST_CHANNEL_ID, CAN_DATA); 
  delay(100);
}

void InitializeCAN(){
  while (CAN_OK != CAN.begin(CAN_500KBPS)){delay(100);};
  #if DEBUGMODE //Using COMPILER'S IF BECAUSE OF PERFORMANCE INCREASING.
    Serial.println("[MCU_MASTER] CAN    -> Initialized.");
  #endif
}

char ReadSensor(int PIN){
  /* TODO LIST:                                              
   * - Find the maximum/minimum of the sensor
   * - Map the coming data
   * - Fix overflow
   */
   char _buf = analogRead(PIN); 
   #if DEBUGMODE
    Serial.println("[MCU_MASTER] SENSOR -> ");
    Serial.print(PIN);
    Serial.print(" ");
    Serial.print(_buf, HEX);
   #endif
   return _buf;
}

bool SendCANDATA(char MESSAGE_ID, unsigned char MESSAGE_DATA_ARRAY, int MESSAGE_LENGTH){
  #if DEBUGMODE
    Serial.println("[MCU_MASTER] CAN    -> Can sent a message!");
  #endif
  return CAN.sendMsgBuf(MESSAGE_ID, 0, MESSAGE_LENGTH, MESSAGE_DATA_ARRAY);
}

bool SendCANDATA(char MESSAGE_ID, unsigned char MESSAGE_DATA_ARRAY){
  #if DEBUGMODE
    Serial.println("[MCU_MASTER] CAN    -> Can sent a message!");
  #endif
  return CAN.sendMsgBuf(MESSAGE_ID, 0, CAN_DATA_LENGTH, MESSAGE_DATA_ARRAY);
}

char SensorDataToTorque(char DATA){
  /*  TODO LIST:
   *  - Calculations about sensor data to Motor Torque.
   */
  char _buf = DATA * 1; //Just test calculation.
  return _buf;
}





//NOTE: THIS SECTION IS GOT FROM OLDER YEARS. JUST CONVERTED A BIT.
void NODE_RESET(){
  unsigned char _buf[2] = {0x81, 0x00};
  SendCANDATA(0x0, _buf, sizeof(_buf));
  SendCANDATA(0x0, _buf, sizeof(_buf));
  free(_buf);
}

char CheckMCU(){
  NODE_RESET();
  unsigned char _buf[1];
  unsigned char len = 0;
  while(true){
    if(CAN.checkReceive() == CAN_MSGAVAIL){
      CAN.readMsgBuf(&len, _buf);
      if(_buf[0] == 0 && len == 1 && ((CAN.getCanId() - 0x700) == MOTOR_NODE_ID)){
        return MOTOR_NODE_ID;
        break;
      }
    }
  }
}

bool SetPreOperationalMode(bool ENABLE){
  #if DEBUGMODE
    Serial.println("[MCU_MASTER] MODE   -> MCU Pre-Operational Mode Switch START...");
  #endif
  unsigned char _buf[2] = {0x80, 0x00};
  bool _bBuf[2]; //To store result of the CAN MESSAGE.
  _bBuf[0] = SendCANDATA(0x0, _buf, sizeof(_buf));
  if(ENABLE){
    _buf[1] = MotorNodeID;
    _bBuf[1] = SendCANDATA(0x0, _buf, sizeof(_buf));
    #if DEBUGMODE
      Serial.println("[MCU_MASTER] MODE   -> MCU Switched to Pre-Operational Mode.");
    #endif
    free(_buf); //Freeing buffer for memory saving puporses
    free(_bBuf); //Freeing buffer for memory saving puporses
    return true;
  }
  else{
    //TODO - Close Pre-Operational Mode.
  }
}

bool SetOperationalMode(bool ENABLE){
  #if DEBUGMODE
    Serial.println("[MCU_MASTER] MODE   -> MCU Operational Mode Switch START...");
  #endif
  unsigned char _buf[2] = {0x01, 0x00};
  bool _bBuf[2]; //To store result of the CAN MESSAGE.
  _bBuf[0] = SendCANDATA(0x0, _buf, sizeof(_buf));
  if(ENABLE){
    _buf[1] = MotorNodeID;
    _bBuf[1] = SendCANDATA(0x0, _buf, sizeof(_buf));
    #if DEBUGMODE
      Serial.println("[MCU_MASTER] MODE   -> MCU Switched to Operational Mode.");
    #endif
    free(_buf); //Freeing buffer for memory saving puporses
    free(_bBuf); //Freeing buffer for memory saving puporses
    return true;
  }
  else{
    //TODO - Close Operational Mode.
  }
}

bool SetPWMMode(bool ENABLE){
  #if DEBUGMODE
    Serial.println("[MCU_MASTER] MODE   -> MCU PWM Mode Switch START...");
  #endif
  unsigned char _buf[8] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  if(ENABLE){
    SendCANDATA(0x200 + MotorNodeID, _buf);
    delay(1500);
    _buf[0] = 0x07;
    SendCANDATA(0x200 + MotorNodeID, _buf);
    delay(1500);
    _buf[0] = 0x0F;
    SendCANDATA(0x200 + MotorNodeID, _buf);
    delay(1500);
    #if DEBUGMODE
      Serial.println("[MCU_MASTER] MODE   -> MCU Switched to PWM Mode.");
    #endif
    free(_buf);
    return true;
  }
  else{
    //TODO - Close PWM Mode.
  }
}
