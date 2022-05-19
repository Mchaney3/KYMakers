void initMotors() {
  // sets the Right motor pins as outputs:
  pinMode(Rt_Motor_IN3, OUTPUT);
  pinMode(Rt_Motor_IN4, OUTPUT);
  pinMode(Rt_Motor_Speed, OUTPUT);
  
  // sets the Left motor pins as outputs:
  pinMode(Lt_Motor_IN1, OUTPUT);
  pinMode(Lt_Motor_IN2, OUTPUT);
  pinMode(Lt_Motor_Speed, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannelRtMotor, freq, resolution);
  ledcSetup(pwmChannelLtMotor, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Rt_Motor_Speed, pwmChannelRtMotor);
  Serial.println("Right Motor Initialized");
  ledcAttachPin(Lt_Motor_Speed, pwmChannelLtMotor);
  Serial.println("Left Motor Initialized");
}

void motorsStop() {
  setMotors(0,0);
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, LOW);
  digitalWrite(Rt_Motor_IN3, LOW);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Motors stopped");
}

void motorsForward() {
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, HIGH);
  digitalWrite(Rt_Motor_IN3, HIGH);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Moving Forward");
  setMotors(dutyCycle, dutyCycle);
}

void motorsBackward() {
  digitalWrite(Lt_Motor_IN1, HIGH);
  digitalWrite(Lt_Motor_IN2, LOW); 
  digitalWrite(Rt_Motor_IN3, LOW);
  digitalWrite(Rt_Motor_IN4, HIGH);
  Serial.println("Moving Backwards");
  setMotors(dutyCycle, dutyCycle);
}

void turnRight() {
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, HIGH); 
  digitalWrite(Rt_Motor_IN3, LOW);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Turning Right");
  setMotors(dutyCycle, dutyCycle);
}

void turnLeft() {
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, LOW); 
  digitalWrite(Rt_Motor_IN3, HIGH);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Turnin Layuft");
  setMotors(dutyCycle, dutyCycle);
}


void setMotors(float left, float right) {
  ledcWrite(pwmChannelRtMotor, right);   
  ledcWrite(pwmChannelLtMotor, left);
}
