void Motor_Init() {
  pinMode(PWM_MOTOR_ONE, OUTPUT);
  pinMode(PWM_MOTOR_TWO, OUTPUT);
  pinMode(DIR_MOTOR_ONE, OUTPUT);
  pinMode(DIR_MOTOR_TWO, OUTPUT);

  analogWrite(PWM_MOTOR_ONE, LOW);
  analogWrite(PWM_MOTOR_TWO, LOW);
}

void selfBalance() {
//  last_error = 0 - tan(aaReal.y/aaReal.x);
  u8mappedMotorOnePWM = abs(control_signal);
  u8mappedMotorOnePWM = map(u8mappedMotorOnePWM, NUM_ZERO, MAX_DEG_ERROR, NUM_ZERO, MAX_PWM_VAL);
  u8mappedMotorTwoPWM = abs(control_signal);
  u8mappedMotorTwoPWM = map(u8mappedMotorTwoPWM, NUM_ZERO, MAX_DEG_ERROR, NUM_ZERO, MAX_PWM_VAL);
  
  control_signal > NUM_ZERO ? b8MotorOneDirection = b8MotorTwoDirection = LOW : b8MotorOneDirection = b8MotorTwoDirection = HIGH;
//  last_error > NUM_ZERO ? b8MotorTwoDirection = LOW : b8MotorTwoDirection = HIGH;

  analogWrite(PWM_MOTOR_ONE, u8mappedMotorOnePWM);
  analogWrite(PWM_MOTOR_TWO, u8mappedMotorTwoPWM);
  digitalWrite(DIR_MOTOR_ONE, b8MotorOneDirection);
  digitalWrite(DIR_MOTOR_TWO, b8MotorTwoDirection);
}
