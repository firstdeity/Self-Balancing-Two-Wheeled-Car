void setupMotor()
{
  const int PWMA = 5, PWMB = 9;               // Speed control 
  const int AIN1 = 6, BIN1 = 11;              // Direction +
  const int AIN2 = 4, BIN2 = 10;              // Direction -
  const int STBY = 7;                        // standby(停止)
  const int C1_A = A3 , C2_A = 2;             //中斷腳位
  const int C1_B = 8 , C2_B = 3;
  
  motorA.SetMotorPins(PWMA,AIN1,AIN2,STBY);     //設置motorA PIN腳
  motorB.SetMotorPins(PWMB,BIN1,BIN2,STBY);     //設置motorB PIN腳
  
  motorA.InverseRotationDirectionDefinition(false); //motorB輪子轉動方向相反(經測試得知)
  motorB.InverseRotationDirectionDefinition(true); 

  motorA.SetControllerBound(200,-200,1.8,-1.8);
  motorB.SetControllerBound(200,-200,1.8,-1.8);

  motorA.SetControl(0,reference,kp,ki,kd);
  motorB.SetControl(0,reference,kp,ki,kd);
    
  motorA.SetControl(1,preference,pkp,pki,pkd);
  motorB.SetControl(1,preference,pkp,pki,pkd);

  directionController.SetBound(100,-100);
  directionController.SetPID(dkp,dki,dkd);
  directionController.SetReference(dreference);

  motorA.SetEncoderPins(C2_A,C1_A);             //設定motorA中斷腳位
  motorB.SetEncoderPins(C2_B,C1_B);             //設定motorB中斷腳位

//設定中斷處理函式
  attachInterrupt(digitalPinToInterrupt(motorA.GetEncoderInterruptPin())
                    ,encoder1Interrupt,RISING);
  attachInterrupt(digitalPinToInterrupt(motorB.GetEncoderInterruptPin())
                    ,encoder2Interrupt,RISING);
}


