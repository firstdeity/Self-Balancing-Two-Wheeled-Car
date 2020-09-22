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
float pkp = 0.13;  
float pki = 0;
float pkd = 0.02;
//PID direction
float dreference = 0;
float dkp = 5;
float dki = 0;
float dkd = 0;
//PID straight
float skp = 0.01;
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
//------------------------

BalanbotMotor motorA;
BalanbotMotor motorB;
PIDController directionController;
float wheel_angA = 0;
float wheel_angB = 0;
float maxPowerRemap = 128/100;
float dT = 0.008;
float rj = 0;
bool pos_ctl = true;
bool turn_ctl = false;
//----------------------------------------------------------------
typedef struct{
  bool pos;
  bool turn;
  float ang;
  float goal;
  float ang1;
  float goal1;
}command;
int state=0;
#define CMD_SIZE 9
command cmd[CMD_SIZE];
int n=1;
float right_speed=0, left_speed=0;
float tspeedA=0, tspeedB=0;
float ecoA=0, ecoB=0;


void timerInterrupt(){
    /*sei();
    double phi = getPhi();
    motorA.Update(phi);
    motorB.Update(phi);
    int speed_d_out = directionController.Update(motorA.GetSpeed()-motorB.GetSpeed());
    motorA.Rotate(motorA.getEffort()-speed_d_out);
    motorB.Rotate(motorA.getEffort()+speed_d_out);
    
    wheel_angA = motorA.GetAngle();
    wheel_angB = motorB.GetAngle();*/
    sei();
    double Phi = getPhi();
    right_speed = motorA.GetSpeed();                      //右輪角速度
    left_speed = motorB.GetSpeed();                       //左輪角速度
    motorA.Update(Phi);
    motorB.Update(Phi);  
  
    switch(n)
    {
      case 1:
      {
        tspeedA = 0;
        tspeedB = -4.6;
        if((abs(right_speed)<0.01 && abs(right_speed)<0.01))
          reference += 0.012;
        if((reference < 1) && (motorA.GetAngle() < 32.5))
          reference += 0.012;
        if((reference > 0.8) && (motorA.GetAngle() >31.2))
          reference -= 0.01;
        if(motorA.GetAngle() > 33)
        {
          reference = 0.3;
          motorA.reset();
          motorB.reset();
          n = 2;
        }
        if(motorA.getEffort()>0)
        {
          tspeedB = 4.7;
        }
        break;
      }

      case 2:
      {
        tspeedA = 25;
        tspeedB = -25;
        if(abs(motorA.GetAngle()-motorB.GetAngle() +8.5)< 1)
        {
          motorA.reset();
          motorB.reset();
          n = 3;
        }     
        break;
      }

    case 3:
    {
      reference = 0.3;
      motorA.reset();
      motorB.reset();
      tspeedA = 0;
      tspeedB = 0;
      n = 3;
      break;
    }
  }
  
  
    motorA.SetControl(0,reference,kp,ki,kd);
    motorB.SetControl(0,reference,kp,ki,kd);
    motorA.Rotate(motorA.getEffort()+tspeedA);
    motorB.Rotate(motorA.getEffort()+tspeedB);
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
  /*cmd[0] = (command){true,false,0.55,10,0,0};
  cmd[1] = (command){false,true,70,8,0,0};
  cmd[2] = (command){true,false,0.55,10,0,0};
  cmd[3] = (command){false,true,70,8,0,0};*/
  /*cmd[2] = (command){true,false,0.65,18,0,0}; 
  cmd[3] = (command){false,true,70,7,0,0};     
  cmd[4] = (command){true,false,0.85,22,0,0};  
  cmd[5] = (command){false,true,-80,-16,0,0};  
  cmd[6] = (command){true,true,0.6,42,-13,0};
  cmd[7] = (command){false,true,-80,-9,0,0};    
  cmd[8] = (command){true,true,0.8,20,-13,0};  */
  /*
   cmd[0] = (command){true,false,0.6,16.5,0,0};
  cmd[1] = (command){false,true,60,7,0,0};   
  cmd[2] = (command){true,false,0.6,13.5,0,0}; 
  cmd[3] = (command){false,true,75,7,0,0};     
  cmd[4] = (command){true,false,0.85,21.5,0,0};  
  cmd[5] = (command){false,true,-80,-18,0,0};  
  cmd[6] = (command){true,true,0.6,33,-30,0};
  cmd[7] = (command){false,true,-80,-9,0,0};    
  cmd[8] = (command){true,true,0.8,20,-16,0};  
   */
}

void loop(){
//UART and Bluetooth communication function 
  //updateBT();
  //senInfo();
  //FSM();
}
/*void FSM(){
  if(state<CMD_SIZE){
      pos_ctl = cmd[state].pos;
      turn_ctl = cmd[state].turn;
      if(pos_ctl && !turn_ctl){
      //directionController.SetPID(0,0,0);
        motorA.positionController.SetReference(cmd[state].goal);
        float bound = cmd[state].ang;
        motorA.positionController.SetBound(bound,-bound);
      }
      else if(!pos_ctl && turn_ctl){
        motorA.positionController.SetPID(0,0,0);
        rj = cmd[state].ang * 0.4;
        directionController.SetReference(dreference + rj);
      }
      else{
        motorA.positionController.SetReference(cmd[state].goal);
        float bound = cmd[state].ang;
        motorA.positionController.SetBound(bound,-bound);
        rj = cmd[state].ang1 * 0.4;
        directionController.SetReference(dreference + rj);
      }
      
      if(next_state()){
        pause();
        state++;
        delay(500);
      }
  }
}
void pause(){
  motorA.reset();
  motorB.reset();
  motorA.positionController.SetPID(pkp,pki,pkd);
  wheel_angB = 0;
  rj = 0;
  directionController.SetPID(dkp,dki,dkd);
  directionController.SetReference(dreference);
}
bool next_state(){
  bool next;
  if(pos_ctl && !turn_ctl){
    next = abs(wheel_angB-cmd[state].goal) < 2;
  }
  else if(!pos_ctl && turn_ctl){
    next = abs(wheel_angA-wheel_angB-cmd[state].goal) < 1;
  }else{
    next = (abs(wheel_angB-cmd[state].goal)<2);
  }
  return next;
}*/
