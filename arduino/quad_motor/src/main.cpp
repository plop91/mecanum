#include <Arduino.h>
#include <PID_v1.h>
#include <util/atomic.h>

// Pin assignments
// Motor 1
#define MOTOR1_PWM 0
#define MOTOR1_IN1 1
#define MOTOR1_IN2 2
#define MOTOR1_CHA 3
#define MOTOR1_CHB 4

// Motor 2
#define MOTOR2_PWM 5
#define MOTOR2_IN1 6
#define MOTOR2_IN2 7
#define MOTOR2_CHA 8
#define MOTOR2_CHB 9

// Motor 3
#define MOTOR3_PWM 10
#define MOTOR3_IN1 11
#define MOTOR3_IN2 12
#define MOTOR3_CHA 13
#define MOTOR3_CHB 14

// Motor 4
#define MOTOR4_PWM 15
#define MOTOR4_IN1 16
#define MOTOR4_IN2 17
#define MOTOR4_CHA 18
#define MOTOR4_CHB 19


// Maths
#define WHEEL_DIAMETER 80 // mm
#define WHEEL_RADIUS WHEEL_DIAMETER / 2
#define WHEEL_CIRCUMFERENCE 2 * PI * WHEEL_RADIUS
#define ENCODER_STEPS_PER_REVOLUTION 4
#define ENCODER_TO_MM 0.0005









// Settings

// - Loop settings
bool led_state = false;
static unsigned long last_update_time = 0;
unsigned long loop_interval = 30; // ms

// - Motor settings

unsigned long last_motor_update_time = 0;

// -- MOTOR1
bool motor1_mode = false; // false = open loop, true = closed loop
double motor1_setpoint = 0;
double motor1_input = 0;
double motor1_output;
volatile int motor1_posi = 0;
int motor1_pos = 0;
int motor1_last_pos = 0;
int motor1_pwm = 0;
int motor1_speed = 0;
double motor1_p = 2;
double motor1_i = 5;
double motor1_d = 1;
PID motor1_PID(&motor1_input, &motor1_output, &motor1_setpoint, motor1_p,
               motor1_i, motor1_d, DIRECT);

// -- MOTOR2
bool motor2_mode = false; // false = open loop, true = closed loop
double motor2_setpoint = 0;
double motor2_input = 0;
double motor2_output;
volatile int motor2_posi = 0;
int motor2_pos = 0;
int motor2_last_pos = 0;
int motor2_pwm = 0;
int motor2_speed = 0;
double motor2_p = 2;
double motor2_i = 5;
double motor2_d = 1;
PID motor2_PID(&motor2_input, &motor2_output, &motor2_setpoint, motor2_p,
               motor2_i, motor2_d, DIRECT);

// -- MOTOR3
bool motor3_mode = false; // false = open loop, true = closed loop
double motor3_setpoint = 0;
double motor3_input = 0;
double motor3_output;
volatile int motor3_posi = 0;
int motor3_pos = 0;
int motor3_last_pos = 0;
int motor3_pwm = 0;
int motor3_speed = 0;
double motor3_p = 2;
double motor3_i = 5;
double motor3_d = 1;
PID motor3_PID(&motor3_input, &motor3_output, &motor3_setpoint, motor3_p,
               motor3_i, motor3_d, DIRECT);

// -- MOTOR4
bool motor4_mode = false; // false = open loop, true = closed loop
double motor4_setpoint = 0;
double motor4_input = 0;
double motor4_output;
volatile int motor4_posi = 0;
int motor4_pos = 0;
int motor4_last_pos = 0;
int motor4_pwm = 0;
int motor4_speed = 0;
double motor4_p = 2;
double motor4_i = 5;
double motor4_d = 1;
PID motor4_PID(&motor4_input, &motor4_output, &motor4_setpoint, motor4_p,
               motor4_i, motor4_d, DIRECT);

// Function declarations
void handle_serial();
void update_motors();
void set_motor_speed(int motor, int speed);
void print_motor_pos();
void print_motor_speed();
void open_loop_control(int motor, int speed);
void closed_loop_control(int motor, int pos);
void stop_all_motors();
void motor1_encoder();
void motor2_encoder();
void motor3_encoder();
void motor4_encoder();
void calculate_motor_speed();

void setup() {
  Serial.begin(115200);

  // Motor 1
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_CHA, INPUT);
  pinMode(MOTOR1_CHB, INPUT);
  attachInterrupt(MOTOR1_CHA, motor1_encoder, RISING);
  motor1_PID.SetMode(AUTOMATIC);

  // Motor 2
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_CHA, INPUT);
  pinMode(MOTOR2_CHB, INPUT);
  attachInterrupt(MOTOR2_CHA, motor2_encoder, RISING);
  motor2_PID.SetMode(AUTOMATIC);

  // Motor 3
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  pinMode(MOTOR3_CHA, INPUT);
  pinMode(MOTOR3_CHB, INPUT);
  attachInterrupt(MOTOR3_CHA, motor3_encoder, RISING);
  motor3_PID.SetMode(AUTOMATIC);

  // Motor 4
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_IN1, OUTPUT);
  pinMode(MOTOR4_IN2, OUTPUT);
  pinMode(MOTOR4_CHA, INPUT);
  pinMode(MOTOR4_CHB, INPUT);
  attachInterrupt(MOTOR4_CHA, motor4_encoder, RISING);
  motor4_PID.SetMode(AUTOMATIC);
}

void loop() {

  // If there is serial data available, read it:
  while (Serial.available() > 0) {
    handle_serial();
  }

  if (millis() - last_update_time > loop_interval) {
    last_update_time = millis();
    calculate_motor_speed();
  }
}

void calculate_motor_speed() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    motor1_pos = motor1_posi;
    motor2_pos = motor2_posi;
    motor3_pos = motor3_posi;
    motor4_pos = motor4_posi;
  }

  unsigned long current_time = millis();
  unsigned long delta_time = current_time - last_motor_update_time;

  motor1_speed = (motor1_pos - motor1_last_pos) * 1000 / delta_time;
  motor2_speed = (motor2_pos - motor2_last_pos) * 1000 / delta_time;
  motor3_speed = (motor3_pos - motor3_last_pos) * 1000 / delta_time;
  motor4_speed = (motor4_pos - motor4_last_pos) * 1000 / delta_time;

  motor1_last_pos = motor1_pos;
  motor2_last_pos = motor2_pos;
  motor3_last_pos = motor3_pos;
  motor4_last_pos = motor4_pos;

  last_motor_update_time = current_time;
}

void update_motors() {
  if (motor1_mode) {
    // closed loop control
    motor1_input = motor1_speed;
    motor1_PID.Compute();
    set_motor_speed(1, int(motor1_output));
  }
  if (motor2_mode) {
    // closed loop control
    motor2_input = motor2_speed;
    motor2_PID.Compute();
    set_motor_speed(2, int(motor2_output));
  }
  if (motor3_mode) {
    // closed loop control
    motor3_input = motor3_speed;
    motor3_PID.Compute();
    set_motor_speed(3, int(motor3_output));
  }
  if (motor4_mode) {
    // closed loop control
    motor4_input = motor4_speed;
    motor4_PID.Compute();
    set_motor_speed(4, int(motor4_output));
  }
}

void handle_serial() {
  int ix, iy;
  double dx, dy, dz;
  // read the incoming byte:
  char inChar = (char)Serial.read();

  switch (inChar) {
  case 'x':
    Serial.println("Stopping all motors");
    // stop all motors
    stop_all_motors();
    break;

  case 'o': {
    // Open loop control of motor speed

    // get motor and speed
    ix = Serial.parseInt(); // motor number (1-4)
    iy = Serial.parseInt(); // speed (-255 to 255)

    open_loop_control(ix, iy);

    break;
  }
  case 'c': {
    // Closed loop control of motor speed

    // get motor and desired speed
    ix = Serial.parseInt();
    iy = Serial.parseInt();

    closed_loop_control(ix, iy);
    break;
  }
  case 'r': {
    // reset the motor position
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      motor1_posi = 0;
      motor2_posi = 0;
      motor3_posi = 0;
      motor4_posi = 0;
    }
    Serial.println("OK");
    break;
  }
  case 'p': {
    ix = Serial.parseInt();
    dx = Serial.parseFloat();
    dy = Serial.parseFloat();
    dz = Serial.parseFloat();

    switch (ix) {
    case 1:
      motor1_p = dx;
      motor1_i = dy;
      motor1_d = dz;
      motor1_PID.SetTunings(motor1_p, motor1_i, motor1_d);
      break;
    case 2:
      motor2_p = dx;
      motor2_i = dy;
      motor2_d = dz;
      motor2_PID.SetTunings(motor2_p, motor2_i, motor2_d);
      break;
    case 3:
      motor3_p = dx;
      motor3_i = dy;
      motor3_d = dz;
      motor3_PID.SetTunings(motor3_p, motor3_i, motor3_d);
      break;
    case 4:
      motor4_p = dx;
      motor4_i = dy;
      motor4_d = dz;
      motor4_PID.SetTunings(motor4_p, motor4_i, motor4_d);
      break;

    default:
      break;
    }
  }
  case 'e': {
    // get the motor positions
    print_motor_pos();
    break;
  }
  case 's': {
    // print the motor speeds
    print_motor_speed();
    break;
  }
  default:
    break;
  }
}

void print_motor_speed() {
  // Serial.print("M1:");
  Serial.print(motor1_speed);
  Serial.print(" ");
  // Serial.print(", M2:");
  Serial.print(motor2_speed);
  Serial.print(" ");
  // Serial.print(", M3:");
  Serial.print(motor3_speed);
  Serial.print(" ");
  // Serial.print(", M4:");
  Serial.println(motor4_speed);
}

void print_motor_pos() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    motor1_pos = motor1_posi;
    motor2_pos = motor2_posi;
    motor3_pos = motor3_posi;
    motor4_pos = motor4_posi;
  }
  // Serial.print("M1:");
  Serial.print(motor1_pos);
  Serial.print(" ");
  // Serial.print(", M2:");
  Serial.print(motor2_pos);
  Serial.print(" ");
  // Serial.print(", M3:");
  Serial.print(motor3_pos);
  Serial.print(" ");
  // Serial.print(", M4:");
  Serial.println(motor4_pos);
}

void open_loop_control(int motor, int speed) {
  switch (motor) {
  case 1:
    motor1_mode = false;
    break;
  case 2:
    motor2_mode = false;
    break;
  case 3:
    motor3_mode = false;
    break;
  case 4:
    motor4_mode = false;
    break;
  default:
    Serial.println("Invalid motor number");
    break;
  }
  set_motor_speed(motor, speed);
  Serial.println("OK");
}

void closed_loop_control(int motor, int pos) {
  // set the motor to closed loop control and set the setpoint
  switch (motor) {
  case 1:
    motor1_mode = true;
    motor1_setpoint = pos;
    break;
  case 2:
    motor2_mode = true;
    motor2_setpoint = pos;
    break;
  case 3:
    motor3_mode = true;
    motor3_setpoint = pos;
    break;
  case 4:
    motor4_mode = true;
    motor4_setpoint = pos;
    break;
  default:
    Serial.println("Invalid motor number");
    break;
  }
  Serial.println("OK");
}

void set_motor_speed(int motor, int speed) {
  // set the raw speed of the motor
  switch (motor) {
  case 1:
    analogWrite(MOTOR1_PWM, abs(speed));
    if (speed == 0) {
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
    } else if (speed > 0) {
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
    } else {
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
    }
    break;
  case 2:
    analogWrite(MOTOR2_PWM, abs(speed));
    if (speed == 0) {
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, LOW);
    } else if (speed > 0) {
      digitalWrite(MOTOR2_IN1, HIGH);
      digitalWrite(MOTOR2_IN2, LOW);
    } else {
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, HIGH);
    }
    break;
  case 3:
    analogWrite(MOTOR3_PWM, abs(speed));
    if (speed == 0) {
      digitalWrite(MOTOR3_IN1, LOW);
      digitalWrite(MOTOR3_IN2, LOW);
    } else if (speed > 0) {
      digitalWrite(MOTOR3_IN1, HIGH);
      digitalWrite(MOTOR3_IN2, LOW);
    } else {
      digitalWrite(MOTOR3_IN1, LOW);
      digitalWrite(MOTOR3_IN2, HIGH);
    }
    break;
  case 4:
    analogWrite(MOTOR4_PWM, abs(speed));
    if (speed == 0) {
      digitalWrite(MOTOR4_IN1, LOW);
      digitalWrite(MOTOR4_IN2, LOW);
    } else if (speed > 0) {
      digitalWrite(MOTOR4_IN1, HIGH);
      digitalWrite(MOTOR4_IN2, LOW);
    } else {
      digitalWrite(MOTOR4_IN1, LOW);
      digitalWrite(MOTOR4_IN2, HIGH);
    }
    break;
  default:
    break;
  }
}

void stop_all_motors() {
  motor1_mode = false;
  motor2_mode = false;
  motor3_mode = false;
  motor4_mode = false;
  set_motor_speed(1, 0);
  set_motor_speed(2, 0);
  set_motor_speed(3, 0);
  set_motor_speed(4, 0);
}

void motor1_encoder() {
  if (digitalRead(MOTOR1_CHA) == digitalRead(MOTOR1_CHB)) {
    motor1_posi++;
  } else {
    motor1_posi--;
  }
}

void motor2_encoder() {
  if (digitalRead(MOTOR2_CHA) == digitalRead(MOTOR2_CHB)) {
    motor2_posi++;
  } else {
    motor2_posi--;
  }
}

void motor3_encoder() {
  if (digitalRead(MOTOR3_CHA) == digitalRead(MOTOR3_CHB)) {
    motor3_posi++;
  } else {
    motor3_posi--;
  }
}

void motor4_encoder() {
  if (digitalRead(MOTOR4_CHA) == digitalRead(MOTOR4_CHB)) {
    motor4_posi++;
  } else {
    motor4_posi--;
  }
}