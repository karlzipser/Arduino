
void servo_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - servo_prev_interrupt_time;
  servo_prev_interrupt_time = m;
  if (state == STATE_ERROR) return; // no action if in error state
  if (dt>SERVO_MIN && dt<SERVO_MAX) {
    servo_pwm_value = dt;
    servo.writeMicroseconds(servo_pwm_value);
    written_servo_pwm_value = servo_pwm_value;
  }
}

