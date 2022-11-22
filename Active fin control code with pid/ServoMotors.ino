void MOTORSetup() {
  Servo1.setPeriodHertz(50);
  Servo2.setPeriodHertz(50);
  Servo3.setPeriodHertz(50);
  Servo4.setPeriodHertz(50);

  Servo1.attach(servopin_1, 500, 2400);
  Servo2.attach(servopin_2, 500, 2400);
  Servo3.attach(servopin_3, 500, 2400);
  Servo4.attach(servopin_4, 500, 2400);
}

void MotorWrite(int a, int b){
  Servo1.write(a);
  Servo2.write(a);
  Servo3.write(b);
  Servo4.write(b);  
}