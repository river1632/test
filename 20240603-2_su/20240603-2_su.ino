#include "Arduino_NineAxesMotion.h"  // Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <Steppino.h>
#include <FlexiTimer2.h>

NineAxesMotion mySensor;  // Object for the sensor

const int CLK1 = 19;
const int DIR1 = 18;
const int ENA1 = 17;

const int CLK2 = 16;
const int DIR2 = 15;
const int ENA2 = 14;

const int CLK3 = 35;
const int DIR3 = 34;
const int ENA3 = 33;

const int CLK4 = 30;
const int DIR4 = 29;
const int ENA4 = 28;

Steppino MyMotor1(md_1CK, 19, 18, 17, 0 ); // 1Clock 드라이버인 경우 CLK, DIR, ENA
Steppino MyMotor2(md_1CK, 16, 15, 14, 0 ); // 1Clock 드라이버인 경우 CLK, DIR, ENA
Steppino MyMotor3(md_1CK, 35, 34, 33, 0 ); // 1Clock 드라이버인 경우 CLK, DIR, ENA
Steppino MyMotor4(md_1CK, 30, 29, 28, 0 ); // 1Clock 드라이버인 경우 CLK, DIR, ENA

float rollDeg, pitchDeg, Heading;
float pi = 3.14159265358979323846;

void onTimer()
{
  MyMotor1.motor_Timer(); // Steppino 의 motor_Timer.호출
  MyMotor2.motor_Timer(); // 복수 모터시 모두 넣어 주어야 합니다.
  MyMotor3.motor_Timer(); // 복수 모터시 모두 넣어 주어야 합니다.
  MyMotor4.motor_Timer(); // 복수 모터시 모두 넣어 주어야 합니다.
}

void setup() {
  Serial.begin(9600);  // Initialize the Serial Port to view information on the Serial Monitor
  Wire.begin();  // Initialize I2C communication to let the library communicate with the sensor

  // Sensor Initialization
  mySensor.initSensor();  // The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);  // Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  // Changing to MANUAL requires calling the relevant update functions prior to calling the read functions

  getRPY();
  delay(1000);

  // 스텝모터 초기화
 /* stepper1.setMaxSpeed(10000);
  stepper1.setAcceleration(3000);
  stepper2.setMaxSpeed(10000);
  stepper2.setAcceleration(3000);
  stepper3.setMaxSpeed(10000);
  stepper3.setAcceleration(3000);
  stepper4.setMaxSpeed(10000);
  stepper4.setAcceleration(3000);*/

  // ENA 핀을 출력으로 설정하고 활성화
pinMode(CLK1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(ENA1, OUTPUT);

  pinMode(CLK2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ENA2, OUTPUT);

  pinMode(CLK3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(ENA3, OUTPUT);

  pinMode(CLK4, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(ENA4, OUTPUT);
  
  
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, HIGH);
  digitalWrite(DIR4, HIGH);

  MyMotor1.setDelay( 80, 24, 40 ); // Unipola, Bipola 이면 24 정도 1CK, 2CK 는 6정도
  MyMotor2.setDelay( 80, 24, 40 ); // 1CK, 2CK 의 경우는 6 정도도 가능
  MyMotor3.setDelay( 80, 24, 40 ); // 1CK, 2CK 의 경우는 6 정도도 가능
  MyMotor4.setDelay( 80, 24, 40 ); // Unipola, Bipola 이면 24 정도 1CK, 2CK 는 6정도
 
}

void loop() {
  mySensor.updateEuler();  // Update the Euler data into the structure of the object
  mySensor.updateCalibStatus();  // Update the Calibration Status

  float alpha = (mySensor.readEulerRoll() - rollDeg) * pi / 180;
  float beta = (mySensor.readEulerPitch() - pitchDeg) * pi / 180;

  // X축 기울기에 따라 스텝모터 1과 2 제어 (반대 방향으로 균형 조정)
  if (alpha > 0.1)
  {  FlexiTimer2::set( 1, 0.09 / 1000, onTimer ); // 16 micro seconds.
    FlexiTimer2::start();
    MyMotor1.Start(1, 1000 ); // 역방향 두바퀴
    MyMotor2.Start(1, 1000 ); // 역방향 두바퀴
   
  } else if (alpha < -0.1) 
  {
    FlexiTimer2::set( 1, 0.09 / 1000, onTimer ); // 16 micro seconds.
    FlexiTimer2::start();
    MyMotor1.Start(-1, 1000 ); // 역방향 두바퀴
    MyMotor2.Start(-1, 1000 ); // 역방향 두바퀴
  
  }

  // Y축 기울기에 따라 스텝모터 3과 4 제어 (반대 방향으로 균형 조정)
  if (beta > 0.1)
   {
     FlexiTimer2::set( 1, 0.09 / 1000, onTimer ); // 16 micro seconds.
    FlexiTimer2::start();
    MyMotor3.Start(1, 1000 ); // 역방향 두바퀴
    MyMotor4.Start(1, 1000 ); // 역방향 두바퀴
  
  } else if (beta < -0.1)
  {
    FlexiTimer2::set( 1, 0.09 / 1000, onTimer ); // 16 micro seconds.
    FlexiTimer2::start();
    MyMotor3.Start(-1, 1000 ); // 역방향 두바퀴
    MyMotor4.Start(-1, 1000 ); // 역방향 두바퀴
  }

  

  // 시리얼 모니터에 데이터 출력
  Serial.print("Roll: "); Serial.print(mySensor.readEulerRoll());
  Serial.print(" | Pitch: "); Serial.print(mySensor.readEulerPitch());
  Serial.print(" | Heading: "); Serial.println(mySensor.readEulerHeading());

  delay(300);
}
//alpha....x
//beta......y
void getRPY() {
  mySensor.updateEuler();  // Update the Euler data into the structure of the object
  mySensor.updateCalibStatus();  // Update the Calibration Status
  rollDeg = mySensor.readEulerRoll();
  pitchDeg = mySensor.readEulerPitch();
  Heading = mySensor.readEulerHeading();
}
