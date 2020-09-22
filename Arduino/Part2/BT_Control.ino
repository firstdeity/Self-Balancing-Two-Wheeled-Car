
void updateBT(){
    while(BTSerial.available()){  
        startRecieve = true;  
        val=BTSerial.read();
        receiveData += val;
    }  
    if(startRecieve){  
        startRecieve = false;  
       // Serial.println(recieveData);
        remoteControl(receiveData);
        receiveData = "";  
    }
}

void remoteControl(String data) {
    for(int i=0;i<data.length();i++){
        if(data[i]<48 || data[i]>57){
          if(/*data[i]!='r' && data[i]!='p' && data[i]!='i' && data[i]!='d' && data[i]!='e' &&  data[i]!='o' && data[i]!='u' && data[i]!='s' &&*/ data[i]!='.' && data[i]!='-' && data[i]!='L' && data[i]!='R')
            return;
        }
    }

    String data_PID="";
    for(int i=1;i<data.length();i++)
        data_PID += data[i];
    Serial.println(data_PID);
    switch(data[0]){
     /* case 'p':
        kp = data_PID.toFloat();
        break;
      case 'i':
        ki = data_PID.toFloat();
        break;
      case 'd':
        kd = data_PID.toFloat();
        break;
      case 'r':
        reference = data_PID.toFloat();
        break;
      case 'o':
        pkp = data_PID.toFloat();
        break;
      case 'u':
        pki = data_PID.toFloat();
        break;
      case 's':
        pkd = data_PID.toFloat();
        break;
      case 'e':
        preference = data_PID.toFloat();
        break;*/
      case 'L':
        String lstr="",rstr="";
        bool f=true;
        for(int i=0;i<data_PID.length();i++){
          if(data_PID[i]!='R'){
            if(f){
              lstr += data_PID[i];
            }
            else{
              rstr += data_PID[i];
            }
          }
          else{
            f = false;
          }
        }

        lj = lstr.toInt()/65.0;
        rj = rstr.toInt()*1.44;
        directionController.SetReference(rj);  
    }  
    motorA.SetControl(0,reference,kp,ki,kd);
    motorB.SetControl(0,reference,kp,ki,kd);
    motorA.SetControl(1,preference,pkp,pki,pkd);
    motorB.SetControl(1,preference,pkp,pki,pkd); 
}
void sendInfo()
{
  if((micros()-btTimer) > 100000){
    btTimer = micros();
    String info = String(kalAngleX) + ",";
    info += String(motorA.getPosError()) + ",";
    info += String(motorB.getPosError());
    BTSerial.println(info);
  }
} 
