
/*
Code written for Arduino Uno.

The purpose is to read PWM signals coming out of a radio receiver
and either relay them unchanged to ESC/servo or substitute signals from a host system.

The steering servo and ESC(motor) are controlled with PWM signals. These meaning of these signals
may vary with time, and across devices. To get uniform behavior, the user calibrates with each session.
The host system deals only in percent control signals that are assumed to be based on calibrated PWM
signals. Thus, 0 should always mean 'extreme left', 49 should mean 'straight ahead', and 99 'extreme right'
in the percent signals, whereas absolute values of the PWM can vary for various reasons.

24 April 2016
*/
#include "constants.h"
#include "PinChangeInterrupt.h" // Adafruit library
#include <Servo.h> // Arduino library

// These come from the radio receiver via three black-red-white ribbons.
#define PIN_SERVO_IN 11
#define PIN_MOTOR_IN 10
#define PIN_BUTTON_IN 12

// These go out to ESC (motor controller) and steer servo via black-red-white ribbons.
#define PIN_SERVO_OUT 9
#define PIN_MOTOR_OUT 8

// On-board LED, used to signal error state
#define PIN_LED_OUT 13
//
/////////////////////


// Below are variables that hold ongoing signal data. I try to initalize them to
// to sensible values, but they will immediately be reset in the running program.
//
// These volatile variables are set by interrupt service routines
// tied to the servo and motor input pins. These values will be reset manually in the
// STATE_D state, so conservative values are given here.
volatile int servo_null_pwm_value = 1500;
volatile int servo_max_pwm_value  = 1600;
volatile int servo_min_pwm_value  = 1400;
volatile int motor_null_pwm_value = 1528;
volatile int motor_max_pwm_value  = 1600;
volatile int motor_min_pwm_value  = 1400;
int motor_speed_limit_pwm_value = motor_max_pwm_value;
// These are three key values indicating current incoming signals.
// These are set in interrupt service routines.
volatile int button_pwm_value = 1210;
volatile int servo_pwm_value = servo_null_pwm_value;
volatile int motor_pwm_value = motor_null_pwm_value;
// These are used to interpret interrupt signals.
volatile unsigned long int button_prev_interrupt_time = 0;
//volatile unsigned long int prev_button_prev_interrupt_time = 0;
volatile unsigned long int servo_prev_interrupt_time  = 0;
volatile unsigned long int motor_prev_interrupt_time  = 0;
volatile unsigned long int state_transition_time_ms = 0;
// Some intial conditions, putting the system in lock state.
volatile int state = STATE_B;
volatile int previous_state = 0;
volatile int prev_previous_state = 0;
// Variable to receive caffe data and format it for output.
unsigned long int caffe_last_int_read_time;
int caffe_mode = -3;
int caffe_servo_percent = 49;
int caffe_motor_percent = 49;
int caffe_servo_pwm_value = servo_null_pwm_value;
int caffe_motor_pwm_value = motor_null_pwm_value;
//int caffe_last_written_servo_pwm_value = servo_null_pwm_value;
//int caffe_last_written_motor_pwm_value = motor_null_pwm_value;
// Values written to serial
volatile int written_servo_pwm_value = servo_null_pwm_value;
volatile int written_motor_pwm_value = motor_null_pwm_value;
// The host computer is not to worry about PWM values. These variables hold percent values
// that are passed up to host.
int servo_percent = 49;
int motor_percent = 49;

/*
volatile unsigned int in_button_A = 0;
volatile unsigned int in_button_B = 0;
volatile unsigned int in_button_C = 0;
volatile unsigned int in_button_D = 0;
volatile unsigned int in_button_D2 = 0;
*/
// Servo classes. ESC (motor) is treated as a servo for signaling purposes.
Servo servo;
Servo motor; 





