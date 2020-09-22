#include <MsTimer2.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <BalanbotMotor.h>
#include <BalanbotEncoder.h>
#include <BalanbotController.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include<Kalman.h>
#include<Math.h>
//------------------------------------------------
//PID variable(phi)
float reference = -0.174;//0.475 //0.3568 //-0.178
float kp = 22;
float ki = 297;
float kd = 0.40;
//PID variable(position)
float preference = 0;
float pkp = 0.145;
float pki = 0;
float pkd = 0.015;
//PID direction
float dreference = 0;
float dkp = 0.45;
float dki = 0;
float dkd = 0;
//-------------------------
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kalmanX; // Create the Kalman instances
double accX, accY, accZ;
double gyroX;
double kalAngleX; // Calculated angle using a Kalman filter
uint32_t timer,btTimer;
uint8_t i2cData[14]; // Buffer for I2C data
//--------------------------------------------------------------
// 定義連接藍牙模組的序列埠 
SoftwareSerial BTSerial(12,13); // 接收腳, 傳送腳
char val;
String receiveData = "";   
bool startRecieve = false;  

BalanbotMotor motorA;
BalanbotMotor motorB;
PIDController directionController;
float wheel_angA = 0;
float wheel_angB = 0;
float maxPowerRemap = 128/100;
float dT = 0.008;
float lj = 0;
float rj = 0;

void timerInterrupt(){
    sei();
    double phi = getPhi();
    motorA.Update(phi-lj);
    motorB.Update(phi-lj);
     if(lj!=0 || rj!=0)
      motorA.reset();
    int speed_d_out = directionController.Update(motorA.GetSpeed() - motorB.GetSpeed());
    motorA.Rotate(motorA.getEffort()+speed_d_out);
    motorB.Rotate(motorA.getEffort()-speed_d_out);
}
void encoder1Interrupt(){
//Call the encoder counting function here.
  motorA.UpdateEncoder();
}

void encoder2Interrupt(){
//Call the encoder counting function here.
  motorB.UpdateEncoder();
}
//-----------------------------------------------------

void setup(){
  Serial.begin(9600);  // 與電腦序列埠連線
  BTSerial.begin(38400);
  setupMotor();        //初始化Motor
  setupMPU6050();     //設置MPU6050
  //Iitial MsTimer2 here 
  MsTimer2::set(dT*1000, timerInterrupt); //設置Timer中斷
  MsTimer2::start();                    //開始MsTimer2
  btTimer=micros();
}

void loop(){
//UART and Bluetooth communication function 
  updateBT();
  //senInfo();
}

