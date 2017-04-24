
void motor_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - motor_prev_interrupt_time;
  motor_prev_interrupt_time = m;
  // Locking out in error state has bad results -- cannot switch off motor manually if it is on.
  // if (state == STATE_ERROR) return;
  if (dt>MOTOR_MIN && dt<MOTOR_MAX) {
    motor_pwm_value = dt;
    motor.writeMicroseconds(motor_pwm_value);
    written_motor_pwm_value = motor_pwm_value;
  } 
}



