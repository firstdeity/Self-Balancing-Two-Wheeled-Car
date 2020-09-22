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
float reference = 0.3;
float kp = 22;
float ki = 297;
float kd = 0.40;
//PID variable(position)
float preference = 0;
float pkp = 0; //0.12
float pki = 0;
float pkd = 0;//0.035
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
String recieveData = "";   
bool startRecieve = false;  
//------------------------

BalanbotMotor motorA;
BalanbotMotor motorB;
float maxPowerRemap = 128/100;
float dT = 0.008;
//----------------------------------------------------------------
void timerInterrupt(){
    sei();
    double phi = getPhi();
    motorA.Update(phi);
    motorB.Update(phi);
    motorA.Rotate(motorA.getEffort());
    motorB.Rotate(motorA.getEffort());
    //float speed_right = motor1.GetSpeed();
    //float speed_left = motor2.GetSpeed();
    //Serial.println(phi);
    //Serial.print(speed_right);Serial.print("\t");
    //Serial.print(speed_left);Serial.print("\t");
    //Serial.println();
    //BTSerial.println(phi);
    //BTSerial.println(motorA.GetSpeed());
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
  if((micros()-btTimer) > 100000){
    btTimer = micros();
    String info = String(kalAngleX) + ",";
    info += String(motorA.getPosError()) + ",";
    info += String(motorB.getPosError());
    //Serial.println(info);
    BTSerial.println(info);
  }
  
}
