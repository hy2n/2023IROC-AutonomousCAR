/* 2023 IROC Autonomous Car CODE

By Donghyun / https://donghyun.cc
*/


//허스키렌즈   **다이나믹셀 라이브러리하고 출돌해서 먼저 선언 필수**
#include "HUSKYLENS.h"
HUSKYLENS huskylens;
#include <string>
using namespace std;
//=========================================================//
//Dynamixel System
#include <DynamixelWorkbench.h>
#define DXL_BUS_SERIAL1 "1"
#define BAUDRATE 1000000
#define DXL_ID1 1
#define DXL_ID2 2
#define DXL_ID3 3
DynamixelWorkbench dxl_wb;
const uint8_t handler_index = 0;
uint8_t dxl_id[3] = { DXL_ID1, DXL_ID2, DXL_ID3 };  // 3개의 모터 선언


//=========================================================//
//DONGHYUN SCPU [ Super Camera Processing Unit ] INIT
void printResult(HUSKYLENSResult result);

//Block 1
int X_root_D = 1;
int ObjectnotDected = 1;
int Line_not_Dected = 1;

//OpenCM 9.04
#define _OPENCM_NO_ERRORS = 0x000000

#define _DNX_SERVICE 

#define _MY_EEPROM

#define _EEPROM_SECURE

#define _HY2N_SCPU_UNIT

//Line
int Line_select = 0;                                //선택된 라인
int Line_count = 0;                                 //감지된 라인
int Line_X[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //X값변수
int Line_Y[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //Y값변수
int Line_Height[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int Line_Width[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int X_LR = 0;         //왼쪽오른쪽구분
int X_root = 0;       //양모서리거리값
int X_io = 0;         //서보회전값
int X_MinValue = 20;  //차선허용범위 (양옆값)
int X_notTurn = 1;    //90도 회전 여부
int X_SV = 0;         //회전각함수
int X_SV_Back = 0;    //회전후진함수
int X_Trun_time = 1000;//회전시간

//=========================================================//

#include <OLLO.h>
OLLO myOLLO;
int BT_status = 0;  //버튼

void setup() {
  Serial.begin(115200);  //PC
  Serial2.begin(9600);   //렌즈
  Serial.println("Ai-Car Sofware Begin..");
  delay(2000);
  myOLLO.begin(1, IR_SENSOR);
  while (!huskylens.begin(Serial2)) {
    Serial.println(F("Begin failed!"));
    delay(100);
  }
  //Dynamixel Setup
  uint16_t model_number = 0;
  dxl_wb.init(DXL_BUS_SERIAL1, BAUDRATE);
  dxl_wb.ping(dxl_id[0], &model_number);  //1번 수신
  dxl_wb.ping(dxl_id[1], &model_number);  //2번 수신
  dxl_wb.ping(dxl_id[2], &model_number);  //3번 수신
  dxl_wb.jointMode(dxl_id[0], 100, 32);   //Id,속도,가속도 설정
  dxl_wb.jointMode(dxl_id[1], 100, 32);   // 조향모터
  dxl_wb.wheelMode(dxl_id[2]);            //ID_3 뒷모터
  dxl_wb.addSyncWriteHandler(dxl_id[2], "Goal_Position");
  //카메라 설정
  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
  huskylens.customText("Vison Booster ON", 0, 0);
  delay(100);
  ChangeSpeed(1);
  dxl_wb.goalPosition(DXL_ID2, 720);  //camera
  dxl_wb.goalPosition(DXL_ID1, 512);  //servo
  huskylens.customText("System Ready!", 90, 90);
  huskylens.customText("Press Button", 100, 120);
}


void loop() {
  ButtonWait();  //버튼대기
  huskylens.clearCustomText();

  //dxl_wb.goalPosition(DXL_ID2, 590);  //camera
  //delay(1000);
  //WaitObject();                       //[테스트] 오브젝트 대기
  //dxl_wb.goalPosition(DXL_ID2, 710);  //camera
  //delay(1000); //차단봉 테스트

  while (1) {
    int lcnt = 0; 

    X_Trun_time = 1400;
    while (X_notTurn) linetrace_turn(350, 2);  //1번째
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    X_Trun_time = 1700;
    while (X_notTurn) linetrace_turn(350, 2);  //2번째
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    X_Trun_time = 1400;
    while (X_notTurn) linetrace_turn(350, 1);  //3번째
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    X_Trun_time = 1600;
    while (X_notTurn) linetrace_turn(350, 1);  //4번째
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    for (lcnt = 1; lcnt <= 6; lcnt++) linetrace(300);
    Stop();
    delay(3000);
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(3000);

    while (X_notTurn) linetrace_turn(400, 1);  //6번째
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    while (X_notTurn) linetrace_turn(400, 2);  //7번째 오른쪽
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    while (X_notTurn) linetrace_turn(400, 2);  //8번째 오른쪽
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    for (lcnt = 1; lcnt <= 15; lcnt++) linetrace(400); //파란색

    while (X_notTurn) linetrace_turn(400, 1);  //10번
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();

    while (X_notTurn) linetrace_turn(400, 1);  //11번째
    X_notTurn = 1;
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(300);
    delay(400);
    Stop();



    //while (X_notTurn) linetrace_turn(400, 1);  //코너회전활성화, 회전값확인**
    //X_notTurn = 1;

    while (1) linetrace(400);  //정지
  }
}


void ChangeSpeed(int a) {
  uint16_t v = 0;
  const char *log;
  int32_t low_speed[2] = { 100, 100 };
  int32_t high_speed[2] = { 400, 400 };
  bool result_d = false;
  if (a == 0) {
    for (v = 0; v < 2; v++) {
      result_d = dxl_wb.itemWrite(dxl_id[v], "Moving_Speed", high_speed[v], &log);
    }
  } else {
    for (v = 0; v < 2; v++) {
      result_d = dxl_wb.itemWrite(dxl_id[v], "Moving_Speed", high_speed[v], &log);
    }
  }
}

void linetrace(int spd) {
  Forward(spd);
  ScanBlock();                                              //Line ID번의 객체 스캔
  if (Line_count > 1) {                                     // 1개 이상의 객체가 발견돼면
    if (Line_count == 2) Line_select = DefineObject(1, 2);  //2개 발견하면 일반 Define해서 선택
    else Line_select = AdvancedDefine2();                    //3개이상 발견하면 고급 Define해서 선택
  } else {
    Line_select = 1;  //1개 발견시 그거로 선택
  }
  X_range_define(Line_select);
  Serial.println(X_root);
  //DebugLine(Line_select);
  PrintS1();
  X_io = 1.9 * (X_root - X_MinValue);  //계산값
  X_io = (int)X_io;                    //계산값 반올림 (다이나믹셀전달위한)

  if (X_io > 128) X_io = 128;  //모터과부화방지 최댓값
  if (Line_Width[Line_select] > 80) {
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(200);
    delay(100);
  }
  if (X_root > X_MinValue) {
    if (X_LR == 2) {
      dxl_wb.goalPosition(DXL_ID1, 512 - X_io);
    } else {
      dxl_wb.goalPosition(DXL_ID1, 512 + X_io);
    }
    delay(200);
  } else {
    dxl_wb.goalPosition(DXL_ID1, 512);
    delay(100);
  }
}

void linetrace_turn(int spd, int tr) {
  Forward(spd);
  ScanBlock();                                              //Line ID번의 객체 스캔
  if (Line_count > 1) {                                     // 1개 이상의 객체가 발견돼면
    if (Line_count == 2) Line_select = DefineObject(1, 2);  //2개 발견하면 일반 Define해서 선택
    else Line_select = AdvancedDefine();                    //3개이상 발견하면 고급 Define해서 선택
  } else {
    Line_select = 1;  //1개 발견시 그거로 선택
  }
  X_range_define(Line_select);
  Serial.println(X_root);
  //DebugLine(Line_select);
  PrintS1();
  X_io = 1.7 * (X_root - X_MinValue);  //계산값
  X_io = (int)X_io;                    //계산값 반올림 (다이나믹셀전달위한)
  if (X_io > 128) X_io = 128;          //모터과부화방지 최댓값
  if ((Line_Y[Line_select] < 120) && (Line_Width[Line_select] > 160)) {
    if (tr == 1) {  //왼쪽으로 회전
      X_SV = 384;
      X_SV_Back = 680;  //후진값
    } else {
      X_SV = 680;
      X_SV_Back = 384;  //후진값
    }
    if (Line_Height[Line_select] > 50) {
      dxl_wb.goalPosition(DXL_ID1, X_SV_Back);
      Backward(500);
      delay(500);
      Stop();
    }
    dxl_wb.goalPosition(DXL_ID1, 512);
    Forward(400);
    delay(450);
    Stop();
    delay(300);
    dxl_wb.goalPosition(DXL_ID1, X_SV);
    delay(100);
    Forward(400);
    delay(X_Trun_time);
    X_notTurn = 0;
  } else if (X_root > X_MinValue) {
    if (X_LR == 2) {
      dxl_wb.goalPosition(DXL_ID1, 512 - X_io);
    } else {
      dxl_wb.goalPosition(DXL_ID1, 512 + X_io);
    }
    delay(200);
  } else {
    dxl_wb.goalPosition(DXL_ID1, 512);
    delay(100);
  }
}

void Forward(int ID3_Speed) {
  ID3_Speed = ID3_Speed + 1023;  //모터반대로껴서 계산해서 작동 ;;
  dxl_wb.goalSpeed(DXL_ID3, ID3_Speed);
}

void Backward(int ID3_Speed) {
  dxl_wb.goalSpeed(DXL_ID3, ID3_Speed);
}

void Stop() {
  dxl_wb.goalSpeed(DXL_ID3, 0);
}

void ScanBlock() {
  if (huskylens.request(1)) {
    Line_count = huskylens.count(1);
    int i = 1;
    while (i < (huskylens.count(1) + 1)) {
      HUSKYLENSResult result = huskylens.get(1, (i - 1));
      printResult(result);         //결과값 출력
      Line_X[i] = result.xCenter;  //변수에 값 저장
      Line_Y[i] = result.yCenter;
      Line_Width[i] = result.width;
      Line_Height[i] = result.height;
      i++;
    }
  } else {
    Line_count = 0;
  }
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {  //result is a block
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {  //result is an arrow
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {  //result is unknown.
    Serial.println("Object unknown!");
  }
}

int DefineObject(int object1, int object2) {
  if (((Line_Width[object1] < 100) || (Line_Width[object2] < 100)) && ((Line_Height[object1] < 60) || (Line_Height[object2] < 60))) {
  } else if (Line_Y[object1] > Line_Y[object2]) return object1;
  else if (Line_Y[object1] < Line_Y[object2]) return object2;
}

int DebugLine(int object1) {
  Serial.println(String() + F("Dected Objects : ") + Line_count + F(", Selected Object : ") + Line_select);
  Serial.println(String() + F("Object info: X=") + Line_X[1] + F(", Y=") + Line_Y[1]);
  Serial.println(String() + F("Object info: X=") + Line_X[2] + F(", Y=") + Line_Y[2]);
}

int AdvancedDefine() {  //가장 큰 오브젝트로 선택한다
  int k = 1;
  int max = 1;
  int maxID = 1;
  while (k < (Line_count + 1)) {
    if (Line_X[k] > 160) {
      X_root_D = 320 - Line_X[k];
    }  // 오른쪽 축 계산
    else {
      X_root_D = Line_X[k];
    }  //왼쪽 축 계산
    if ((Line_Width[k] < 100) && (Line_Height[k] < 60)) {

    } else if (Line_Height[k] > max) {
      max = Line_Height[k];
      maxID = k;
    }
    k++;
  }
  return maxID;
}

int AdvancedDefine2() {  //가장 큰 오브젝트로 선택한다
  int k = 1;
  int max = 1;
  int maxID = 1;
  while (k < (Line_count + 1)) {
    if (Line_X[k] > 160) {
      X_root_D = 320 - Line_X[k];
    }  // 오른쪽 축 계산
    else {
      X_root_D = Line_X[k];
    }  //왼쪽 축 계산
    if ((Line_Width[k] < 100) && (Line_Height[k] < 60)) {

    } else if (Line_Width[k] > max) {
      max = Line_Width[k];
      maxID = k;
    }
    k++;
  }
  return maxID;
}

int X_range_define(int objID) {
  if (Line_X[objID] > 160) {
    X_root = 320 - Line_X[objID];
    X_LR = 2;
  }  // 오른쪽 축 계산
  else {
    X_root = Line_X[objID];
    X_LR = 1;
  }  //왼쪽 축 계산
}

void ButtonWait() {
  BT_status = 1;
  while (BT_status) {
    if (myOLLO.read(1, IR_SENSOR) < 600) BT_status = 0;
    else BT_status = 1;
  }
}

void PrintS1() {
  if (X_LR == 1) huskylens.customText("Selected Block L", 0, 0);
  else huskylens.customText("Selected Block R", 0, 0);
}

void WaitObject() {
  ObjectnotDected = 1;  //초기화
  while (ObjectnotDected)
    if (huskylens.request(2))
      if (huskylens.count() < 1) {  //오브젝트가 없다면
        delay(500);                 //감지시 정확성을 위해 5초후 재검증
        if (huskylens.request(2))
          if (huskylens.count() < 1) ObjectnotDected = 0;
      }
}
