void setupMPU6050()
{
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68)
    { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }
    
   //MPU6050 accelgyro;
   MPU6050 accelgyro(0x68); // <-- use for AD0 high
   //int16_t ax, ay, az,gx, gy, gz;
   // reset offsets
   accelgyro.setXAccelOffset(-5171);
   accelgyro.setYAccelOffset(-2497);
   accelgyro.setZAccelOffset(1409);
   accelgyro.setXGyroOffset(78);
   accelgyro.setYGyroOffset(-38);
   accelgyro.setZGyroOffset(-44);
  
  delay(100); // Wait for sensor to stabilize
  
   /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);           //X方向加速度
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);           //Y方向加速度
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);           //Z方向加速度
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;   //傾角
  kalmanX.setAngle(roll); // Set starting angle
  timer=micros();
}
    


double getPhi()                                             //得到平衡車傾角
{
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;   //Roll角
    double gyroXrate = gyroX / 131.0;           //32767/250，預設精度為+-250
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    return kalAngleX;

    
}

