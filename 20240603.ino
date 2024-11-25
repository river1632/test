#include "Arduino_NineAxesMotion.h"  // Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <AccelStepper.h>

NineAxesMotion mySensor;  // Object for the sensor

// 스텝 모터 핀 설정 (DIR, STEP, ENA 핀)
AccelStepper stepper1(AccelStepper::DRIVER, 19, 18);
AccelStepper stepper2(AccelStepper::DRIVER, 16, 15);
AccelStepper stepper3(AccelStepper::DRIVER, 35, 34);
AccelStepper stepper4(AccelStepper::DRIVER, 30, 29);

// ENA 핀 설정
const int ENA1 = 17;
const int ENA2 = 14;
const int ENA3 = 33;
const int ENA4 = 28;

float rollDeg, pitchDeg, Heading;
float pi = 3.14159265358979323846;

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
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(500);

  // ENA 핀을 출력으로 설정하고 활성화
  pinMode(ENA1, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(ENA3, OUTPUT);
  pinMode(ENA4, OUTPUT);
  digitalWrite(ENA1, LOW);
  digitalWrite(ENA2, LOW);
  digitalWrite(ENA3, LOW);
  digitalWrite(ENA4, LOW);
}

void loop() {
  mySensor.updateEuler();  // Update the Euler data into the structure of the object
  mySensor.updateCalibStatus();  // Update the Calibration Status

  float alpha = (mySensor.readEulerRoll() - rollDeg) * pi / 180;
  float beta = (mySensor.readEulerPitch() - pitchDeg) * pi / 180;

  // X축 기울기에 따라 스텝모터 1과 2 제어 (반대 방향으로 균형 조정)
  if (alpha > 0.1) {  // robot lean to left side
    stepper1.move(-100);  // 기울어진 방향의 반대
    stepper2.move(-100);
  } else if (alpha < -0.1) {
    stepper1.move(100);
    stepper2.move(100);
  }

  // Y축 기울기에 따라 스텝모터 3과 4 제어 (반대 방향으로 균형 조정)
  if (beta > 0.1) {
    stepper3.move(-100);
    stepper4.move(-100);
  } else if (beta < -0.1) {
    stepper3.move(100);
    stepper4.move(100);
  }

  // 스텝모터 실행
  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();

  // 시리얼 모니터에 데이터 출력
  Serial.print("Roll: "); Serial.print(mySensor.readEulerRoll());
  Serial.print(" | Pitch: "); Serial.print(mySensor.readEulerPitch());
  Serial.print(" | Heading: "); Serial.println(mySensor.readEulerHeading());

  delay(300);
}

void getRPY() {
  mySensor.updateEuler();  // Update the Euler data into the structure of the object
  mySensor.updateCalibStatus();  // Update the Calibration Status
  rollDeg = mySensor.readEulerRoll();
  pitchDeg = mySensor.readEulerPitch();
  Heading = mySensor.readEulerHeading();
}
