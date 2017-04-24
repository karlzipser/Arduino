

void button_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - button_prev_interrupt_time;
  button_prev_interrupt_time = m;

  if (dt>BUTTON_MIN && dt<BUTTON_MAX) {
    
    button_pwm_value = dt;

    int i;
    
    for (i = 0; i < 4; i=i+1) {
      if (abs(button_pwm_value - ButtonFreqs[i])<BUTTON_DELTA) {
            if (state == STATE_ERROR) return;
            if (state != ButtonStates[i]) {
              state = ButtonStates[i];
              state_transition_time_ms = m/1000.0;
              previous_state = state;
            }
      }
    }
  }
}

