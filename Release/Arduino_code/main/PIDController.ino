void PID_Control(){
  unsigned long current_time = millis();
  int delta_time = current_time - last_time; //delta time interval 
  
  if (delta_time >= T){
    double error = 0 - tan(aaReal.y/aaReal.x);
    
    total_error += error; //accumalates the error - integral term
    if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;
    
    double delta_error = error - last_error; //difference of error for derivative term
    
    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; //PID control compute
    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;
    
    last_error = error;
    last_time = current_time;
  } 
}
