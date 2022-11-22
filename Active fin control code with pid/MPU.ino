void MPUSetup() {
  Wire.setClock(400000);
  Wire.begin();
  delay(10);
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void getAngle(double &A, double &B, double &C) {
  int16_t Gyro_X, Gyro_Y, Gyro_Z;// Ac_X, Ac_Y, Ac_Z;
  // int minV = 265;
  // int maxV = 402;
  Wire.beginTransmission(0x69);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x69, 6);

  Gyro_X = Wire.read() << 8 | Wire.read();
  Gyro_Y = Wire.read() << 8 | Wire.read();
  Gyro_Z = Wire.read() << 8 | Wire.read();

 

  // A = RAD_TO_DEG * (atan2(-Gyro_Y, -Gyro_Z) + PI);
  // B = RAD_TO_DEG * (atan2(-Gyro_X, -Gyro_Z) + PI);
  // C = RAD_TO_DEG * (atan2(-Gyro_Y, -Gyro_X) + PI);
A = Gyro_X / 131.0;
B = Gyro_Y / 131.0;
C = Gyro_Z / 131.0;


  
}