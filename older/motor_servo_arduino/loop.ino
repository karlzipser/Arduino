
////////////////////////////////////////
//



int safe_pwm_range(int p) {
  if (p < SERVO_MIN) return 0;
  if (p > SERVO_MAX) return 0;
  return(1);
}
int safe_percent_range(int p) {
  // There is drift in the pwm values. We will allow a bit of slack, otherwise the car gets into
  // error state too often.
  if (p > 105) return 0; //if (p > 99) return 0;
  if (p < -5) return 0; //if (p < 0) return 0;
  return(1);
}


int check_for_error_conditions(void) {
// Check state of all of these variables for out-of-bound conditions
  // If in calibration state, ignore potential errors in order to attempt to correct.
  if (state == STATE_D) return(1);
  if (
    safe_pwm_range(servo_null_pwm_value) &&
    safe_pwm_range(servo_max_pwm_value) &&
    safe_pwm_range(servo_min_pwm_value) &&
    safe_pwm_range(motor_null_pwm_value) &&
    safe_pwm_range(motor_max_pwm_value) &&
    safe_pwm_range(motor_min_pwm_value) &&
    safe_pwm_range(servo_pwm_value) &&
    safe_pwm_range(motor_pwm_value) &&
    safe_pwm_range(button_pwm_value) &&
    button_prev_interrupt_time >= 0 &&
    servo_prev_interrupt_time >= 0 &&
    motor_prev_interrupt_time >= 0 &&
    state_transition_time_ms >= 0 && 
    safe_percent_range(caffe_servo_percent) &&
    safe_percent_range(caffe_motor_percent) &&
    safe_percent_range(servo_percent) &&
    safe_percent_range(motor_percent) &&
    safe_pwm_range(caffe_servo_pwm_value) &&
    safe_pwm_range(caffe_motor_pwm_value) &&
    caffe_last_int_read_time >= 0 &&
    caffe_mode >= -3 && caffe_mode <= 9 &&
    state >= -1 && state <= 100 &&
    previous_state >= -1 && previous_state <= 100
    
  ) return(1);
  else {
    if (state != STATE_ERROR) {
      // On first entering error state, attempt to null steering and motor
      servo_pwm_value = servo_null_pwm_value;
      motor_pwm_value = motor_null_pwm_value;
      servo.writeMicroseconds(servo_null_pwm_value);
      motor.writeMicroseconds(motor_null_pwm_value);
    }
    state = STATE_ERROR;
    return(0);
  }
}


void loop() {
  check_for_error_conditions();
  motor_speed_limit_pwm_value = 0.35*(motor_max_pwm_value - motor_null_pwm_value) + motor_null_pwm_value;
  // Try to read the "caffe_int" sent by the host system (there is a timeout on serial reads, so the Arduino
  // doesn't wait long to get one -- in which case the caffe_int is set to zero.)
  int caffe_int = Serial.parseInt();
  // If it is received, decode it to yield three control values (i.e., caffe_mode, caffe_servo_percent, and caffe_motor_percent)
  if (caffe_int > 0) {
      caffe_last_int_read_time = micros()/1000;
      caffe_mode = caffe_int/10000;
      caffe_servo_percent = (caffe_int-caffe_mode*10000)/100;
      caffe_motor_percent = (caffe_int-caffe_servo_percent*100-caffe_mode*10000);
    } else if (caffe_int < 0) {
      caffe_mode = caffe_int/10000;
  }
  // Turn the caffe_servo_percent and caffe_motor_percent values from 0 to 99 values into PWM values that can be sent 
  // to ESC (motor) and servo.
  if (caffe_mode > 0) {
    if (caffe_servo_percent >= 49) {
      caffe_servo_pwm_value = (caffe_servo_percent-49)/50.0 * (servo_max_pwm_value - servo_null_pwm_value) + servo_null_pwm_value;
    }
    else {
      caffe_servo_pwm_value = (caffe_servo_percent - 50)/50.0 * (servo_null_pwm_value - servo_min_pwm_value) + servo_null_pwm_value;
    }
    if (caffe_motor_percent >= 49) {
      caffe_motor_pwm_value = (caffe_motor_percent-49)/50.0 * (motor_max_pwm_value - motor_null_pwm_value) + motor_null_pwm_value;
    }
    else {
      caffe_motor_pwm_value = (caffe_motor_percent - 50)/50.0 * (motor_null_pwm_value - motor_min_pwm_value) + motor_null_pwm_value;
    }
  }
  else {
    caffe_servo_pwm_value = servo_null_pwm_value;
    caffe_motor_pwm_value = motor_null_pwm_value;
  }
  // Compute command signal percents from signals from the handheld radio controller
  // to be sent to host computer, which doesn't bother with PWM values
  if (written_servo_pwm_value >= servo_null_pwm_value) {
    servo_percent = 49+50.0*(written_servo_pwm_value-servo_null_pwm_value)/(servo_max_pwm_value-servo_null_pwm_value);
  }
  else {
    servo_percent = 49 - 49.0*(servo_null_pwm_value-written_servo_pwm_value)/(servo_null_pwm_value-servo_min_pwm_value);
  }
  if (written_motor_pwm_value >= motor_null_pwm_value) {
    motor_percent = 49+50.0*(written_motor_pwm_value-motor_null_pwm_value)/(motor_max_pwm_value-motor_null_pwm_value);
  }
  else {
    motor_percent = 49 - 49.0*(motor_null_pwm_value-written_motor_pwm_value)/(motor_null_pwm_value-motor_min_pwm_value);
  }

  int debug = false;
  if (debug) {
    Serial.print("(");
    Serial.print(state);
    Serial.print(",[");
    Serial.print(button_pwm_value);
    Serial.print(",");
    Serial.print(servo_pwm_value);
    Serial.print(",");
    Serial.print(motor_pwm_value);
    Serial.print("],[");
    
    Serial.print(written_servo_pwm_value);
    Serial.print(",");
    Serial.print(servo_null_pwm_value);
    Serial.print(",");
    Serial.print(servo_max_pwm_value);
    Serial.print(",");
    Serial.print(servo_null_pwm_value);
    
    Serial.print("],[");

    Serial.print(servo_min_pwm_value);
    Serial.print(",");
    Serial.print(servo_null_pwm_value);
    Serial.print(",");
    Serial.print(servo_max_pwm_value);  
    Serial.print("],[");
    Serial.print(motor_min_pwm_value);
    Serial.print(",");
    Serial.print(motor_null_pwm_value);
    Serial.print(",");
    Serial.print(motor_max_pwm_value);
    Serial.print("],");
    Serial.print(caffe_mode);
    Serial.print(",");
    Serial.print(caffe_servo_percent);
    Serial.print(",");
    Serial.print(caffe_motor_percent);
    Serial.print(",");
    Serial.print(caffe_servo_pwm_value);
    Serial.print(",");
    Serial.print(caffe_motor_pwm_value);
    Serial.print(",");
    Serial.print(servo_percent);
    Serial.print(",");
    Serial.print(motor_percent);
    Serial.print(",");
    Serial.print(millis() - state_transition_time_ms);
    Serial.println(")");
  }
  else {
    // Send data string which looks like a python tuple.
    Serial.print("('motor',");
    Serial.print(state);
    Serial.print(",");
    if (servo_percent > 99) { // allowing for slack because of drift.
      servo_percent = 99;
    }
    if (servo_percent < 0) { // allowing for slack because of drift.
      servo_percent = 0;
    }
    Serial.print(servo_percent);
    Serial.print(",");
    if (motor_percent > 99) { // allowing for slack because of drift.
      motor_percent = 99;
    }
    if (motor_percent < 0) { // allowing for slack because of drift.
      motor_percent = 0;
    }
    Serial.print(motor_percent);
    Serial.print(",");
    Serial.print(rate_1);
    Serial.print(",");
    Serial.print((millis() - state_transition_time_ms)/1000); //one second resolution
    Serial.print(",");
    Serial.print(caffe_int);
    Serial.print(",");
    Serial.print(caffe_mode);
    Serial.println(")");
    
  }
  delay(10); // How long should this be? Note, this is in ms, whereas most other times are in micro seconds.
  // Blink LED if in error state.
  if (state == STATE_ERROR) {
    digitalWrite(PIN_LED_OUT, HIGH);
    delay(100);
    digitalWrite(PIN_LED_OUT, LOW);
    delay(100);
  }

  //encoder_loop();

  
  if ((state == ButtonStates[0]) && (caffe_mode == 1)) {
    human_left();
  } else if ((state == ButtonStates[0]) && (caffe_mode == 3)) {
    network_left();
  } else if ((state == ButtonStates[1]) && (caffe_mode == 1)) {
    human_straight();
  } else if ((state == ButtonStates[1]) && (caffe_mode == 3)) {
    network_straight();
  } else if ((state == ButtonStates[2]) && (caffe_mode == 1)) {
    human_right();
  } else if ((state == ButtonStates[2]) && (caffe_mode == 3)) {
    network_right();
  } else if ((state == ButtonStates[3]) && (caffe_mode == 1)) {
    no_data();
  } else if ((state == ButtonStates[3]) && (caffe_mode == -3)) {
    calibrate();
  } else if ((state == ButtonStates[1]) && (caffe_mode == 2)) {
    network();
  }

}


//
////////////////////////////////////////
