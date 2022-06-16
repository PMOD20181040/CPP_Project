# CPP_Project
Robots to Combat Harmful Wildlife Using Ropeways
#include <DynamixelShield.h>

#define DXL_SERIAL   Serial
#define DEBUG_SERIAL Serial3

const uint8_t LS_DXL_ID = 102;
const uint8_t LE_DXL_ID = 104;
const uint8_t LW_DXL_ID = 106;
const uint8_t RS_DXL_ID = 103;
const uint8_t RE_DXL_ID = 105;
const uint8_t RW_DXL_ID = 107;

const uint8_t RWH1_DXL_ID = 6;
const uint8_t RWH2_DXL_ID = 4;
const uint8_t LWH2_DXL_ID = 1;
const uint8_t LWH1_DXL_ID = 3;

const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

int DXL_ID_ARM[]={LS_DXL_ID,LE_DXL_ID,LW_DXL_ID,RS_DXL_ID,RE_DXL_ID,RW_DXL_ID};
int DXL_ID_WH[]={RWH1_DXL_ID,RWH2_DXL_ID,LWH2_DXL_ID,LWH1_DXL_ID};

void setup() {
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  for(int i = 0 ; i<6 ; i++){
    dxl.torqueOff(DXL_ID_ARM[i]);
    dxl.setOperatingMode(DXL_ID_ARM[i],OP_POSITION);
    dxl.torqueOn(DXL_ID_ARM[i]);
    DEBUG_SERIAL.println(dxl.ping(DXL_ID_ARM[i]));         //ping 함수로 LS 연결 체크, 안써도 됨 
  }
  
  for(int i = 0 ; i<4; i++){
    dxl.torqueOff(DXL_ID_WH[i]);
    dxl.setOperatingMode(DXL_ID_WH[i],OP_VELOCITY);
    dxl.torqueOn(DXL_ID_WH[i]);
    DEBUG_SERIAL.println(dxl.ping(DXL_ID_WH[i]));         //ping 함수로 LS 연결 체크, 안써도 됨 
  }

}

void loop() { 
  
  dxl.setGoalVelocity(RWH1_DXL_ID, -5, UNIT_PERCENT); 
  dxl.setGoalVelocity(RWH2_DXL_ID, 30, UNIT_PERCENT);
  dxl.setGoalVelocity(LWH1_DXL_ID, -5, UNIT_PERCENT);  
  dxl.setGoalVelocity(LWH2_DXL_ID, 30, UNIT_PERCENT);
  
  dxl.setGoalPosition(LS_DXL_ID,5,UNIT_DEGREE);
  DEBUG_SERIAL.print("LS Present position : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(LS_DXL_ID,UNIT_DEGREE));
  dxl.setGoalPosition(LE_DXL_ID,30,UNIT_DEGREE);
  DEBUG_SERIAL.print("LE Present position : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(LE_DXL_ID,UNIT_DEGREE));
  dxl.setGoalPosition(LW_DXL_ID,173,UNIT_DEGREE);
  DEBUG_SERIAL.print("LW Present position : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(LW_DXL_ID,UNIT_DEGREE));
  dxl.setGoalPosition(RS_DXL_ID,295,UNIT_DEGREE);
  DEBUG_SERIAL.print("RS Present position : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(RS_DXL_ID,UNIT_DEGREE));
  dxl.setGoalPosition(RE_DXL_ID,270,UNIT_DEGREE);
  DEBUG_SERIAL.print("RE Present position : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(RE_DXL_ID,UNIT_DEGREE));
  dxl.setGoalPosition(RW_DXL_ID,127,UNIT_DEGREE);
  DEBUG_SERIAL.print("RW Present position : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(RW_DXL_ID,UNIT_DEGREE));
  DEBUG_SERIAL.println(" ");
  delay(5000);
  dxl.setGoalVelocity(RWH1_DXL_ID, 30, UNIT_PERCENT);  
  dxl.setGoalVelocity(RWH2_DXL_ID, -5, UNIT_PERCENT);
  dxl.setGoalVelocity(LWH1_DXL_ID, 30, UNIT_PERCENT);  
  dxl.setGoalVelocity(LWH2_DXL_ID, -5, UNIT_PERCENT);
  delay(5000);


}
