#include <Servo.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

Servo gripper;

#define INTERRUPT_PIN 2 //Compass INT

#define motor_a_direct_pin 12
#define motor_a_brake_pin 9
#define motor_a_speed 3

#define motor_b_direct_pin 13
#define motor_b_brake_pin 8
#define motor_b_speed 11

#define gripper_pin 7


//compass start.........................................................................................
double degree_ref;//for control only
double heading_degree; //the current orientation
double degree_difference; //relative angle to the heading_degree
double bias = 0; //for pid control
double turn_bias = 0;
int North = 180;
int South = 90;
int West = 0;
int East = -90;

//pid...................................................................................................
//Specify the links and initial tuning parameters
double zero = 0;
double Kp = 1.5, Ki = 0, Kd = 0;
PID xyPID(&degree_difference, &bias, &zero, Kp, Ki, Kd, DIRECT);
double Kpp = 0.8, Kii = 0, Kdd = 0.2;
PID turnPID(&degree_difference, &turn_bias, &zero, Kpp, Kii, Kdd, DIRECT);
//pid...................................................................................................

bool first_wait = true;
MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
void compass_loop() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    //Serial.println("not availble");
  } else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    //Serial.println("overflow");
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    //Serial.println("reading");
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    heading_degree = ypr[0] * 180 / M_PI;
    degree_difference = compass_cal(heading_degree - degree_ref);
    //      Serial.print("ypr\t");
    //      Serial.print(ypr[0] * 180/M_PI);
    //      Serial.print("\t");
    //      Serial.print(ypr[1] * 180/M_PI);
    //      Serial.print("\t");
    //      Serial.println(ypr[2] * 180/M_PI);
  }
}
//compass end.................................................................................................

void compass_loop_ad() {
  compass_loop();
  delay(10);
  compass_loop();
  return;
}


double compass_cal(double x) {
  if (x > 180) {
    return x - 360;
  }
  else if (x < -180) {
    return x + 360;
  }
  else {
    return x;
  }
}

void pickup() {
  gripper.write(85);
}

void putdown() {
  gripper.write(20);
}

void backward(int speed = 255 / 3) {
  digitalWrite(motor_a_direct_pin, HIGH);   //direction A right
  digitalWrite(motor_b_direct_pin, HIGH);   //direction B left
  digitalWrite(motor_a_brake_pin, LOW);     //disengage brake A
  digitalWrite(motor_b_brake_pin, LOW);     //disengage brake B
  analogWrite(motor_a_speed, speed - bias);      //A speed
  analogWrite(motor_b_speed, speed + bias);     //B speed
}


void forward(int speed = 255 / 3) {
  digitalWrite(motor_a_direct_pin, LOW);   //direction A
  digitalWrite(motor_b_direct_pin, LOW);   //direction B
  digitalWrite(motor_a_brake_pin, LOW);     //disengage brake A
  digitalWrite(motor_b_brake_pin, LOW);     //disengage brake B
  analogWrite(motor_a_speed, speed + bias);    //A speed
  analogWrite(motor_b_speed, speed - bias);   //B speed
}


void turn_left(int speed = 255 / 3) {
  digitalWrite(motor_a_direct_pin, LOW);   //direction A
  digitalWrite(motor_b_direct_pin, HIGH);   //direction B
  digitalWrite(motor_a_brake_pin, LOW);     //disengage brake A
  digitalWrite(motor_b_brake_pin, LOW);     //disengage brake B
  analogWrite(motor_a_speed, 60 + abs(turn_bias));    //A speed
  analogWrite(motor_b_speed, 60 + abs(turn_bias));     //B speed
}


void turn_right(int speed = 255 / 3) {
  digitalWrite(motor_a_direct_pin, HIGH);   //direction A
  digitalWrite(motor_b_direct_pin, LOW);   //direction B
  digitalWrite(motor_a_brake_pin, LOW);     //disengage brake A
  digitalWrite(motor_b_brake_pin, LOW);     //disengage brake B
  analogWrite(motor_a_speed, 60 + abs(turn_bias));  //A speed
  analogWrite(motor_b_speed, 60 + abs(turn_bias));   //B speed
}

void left_adjust(int speed = 255 / 4) {
  digitalWrite(motor_a_direct_pin, LOW);   //direction A
  digitalWrite(motor_b_direct_pin, HIGH);   //direction B
  digitalWrite(motor_a_brake_pin, LOW);     //disengage brake A
  digitalWrite(motor_b_brake_pin, LOW);     //disengage brake B
  analogWrite(motor_a_speed, 0.83 * speed);  //A speed
  analogWrite(motor_b_speed, speed);     //B speed
}


void right_adjust (int speed = 255 / 4) {
  digitalWrite(motor_a_direct_pin, HIGH);   //direction A
  digitalWrite(motor_b_direct_pin, LOW);   //direction B
  digitalWrite(motor_a_brake_pin, LOW);     //disengage brake A
  digitalWrite(motor_b_brake_pin, LOW);     //disengage brake B
  analogWrite(motor_a_speed, 0.83 * speed); //A speed
  analogWrite(motor_b_speed, speed);   //B speed
}

void stall() {
  digitalWrite(motor_a_brake_pin, HIGH);    //stop
  digitalWrite(motor_b_brake_pin, HIGH);
}

//excutions....................................................................................
void moving_forward() {
  bool moving_status = true;
  while (moving_status) {
    while (Serial.available() == 0) {
      forward();
      compass_loop_ad();
      xyPID.Compute();
      //use of ultrasnoic_bias
    }
    char command = Serial.read();
    if (command == 'U') {
      int ultrasonic_bias = Serial.parseInt();
    }
    else if (command == 's') {
      stall();
      moving_status = false;
    }
  }
  return;
}

void moving_backward() {
  bool moving_status = true;
  while (moving_status) {
    while (Serial.available() == 0) {
      backward();
      compass_loop_ad();
      xyPID.Compute();
      //use of ultrasnoic_bias
    }
    char command = Serial.read();
    if (command == 'U') {
      int ultrasonic_bias = Serial.parseInt();
    }
    else if (command == 's') {
      stall();
      moving_status = false;
    }
  }
  return;
}


void turn_right_test() {
  compass_loop();
  degree_ref = compass_cal(heading_degree - 90);
  //  Serial.println(compass_cal(heading_degree - degree_ref));
  while (compass_cal(heading_degree - degree_ref) > 1) {  //first movement
    turn_right();
    turnPID.Compute();
    compass_loop();
  }
  stall();
  delay(500);
  compass_loop_ad();
  while (abs(compass_cal(heading_degree - degree_ref)) > 2) {  //adjustment after rotation
    turn_bias = 40;
    if (compass_cal(heading_degree - degree_ref) > 2) {
      turn_right();
      long start_time = millis();
      while (millis() < start_time + 50) {
        //hold;
      }
      stall();
    }

    if (compass_cal(heading_degree - degree_ref) < -2) {
      turn_left();
      long start_time = millis();
      while (millis() < start_time + 50) {
        //hold;
      }
      stall();
    }
    delay(500);
    compass_loop_ad();
  }
}


void turn_left_test() {
  compass_loop();
  degree_ref = compass_cal(heading_degree + 90);
  //  Serial.println("left_turn");
  while (compass_cal(heading_degree - degree_ref) < -1) { //first movement
    turn_left();
    turnPID.Compute();
    compass_loop();
  }
  stall();
  delay(500);
  compass_loop_ad();
  while (abs(compass_cal(heading_degree - degree_ref)) > 2) {  //adjustment after rotation
    turn_bias = 40;
    if (compass_cal(heading_degree - degree_ref) > 2) {
      turn_right();
      long start_time = millis();
      while (millis() < start_time + 50) {
        //hold;
      }
      stall();
    }

    if (compass_cal(heading_degree - degree_ref) < -2) {
      turn_left();
      long start_time = millis();
      while (millis() < start_time + 50) {
        //hold;
      }
      stall();
    }
    delay(500);
    compass_loop_ad();
  }
}


void tilt_left() {
  turn_bias = 40;
  turn_left();
  long start_time = millis();
  while (millis() < start_time + 100) {
    //hold;
  }
  stall();
}

void tilt_right() {
  turn_bias = 40;
  turn_right();
  long start_time = millis();
  while (millis() < start_time + 100) {
    //hold;
  }
  stall();
}

void move_to_left_a_bit() {
  degree_ref = compass_cal(degree_ref - 30); //stage 1
  long start_time = millis();
  int ad_time = 450;
  while (millis() < start_time + ad_time) {
    backward();
    compass_loop();
    xyPID.Compute();
  }
  stall();

  degree_ref = compass_cal(degree_ref + 30); //stage 2
  while (millis() < start_time + 2*ad_time) {
    backward();
    compass_loop();
    xyPID.Compute();
  }
  stall();
  while (millis() < start_time + 4*ad_time) {
    forward();
    compass_loop();
    xyPID.Compute();
  }
  stall();
  return;
}

void move_to_right_a_bit() {
  degree_ref = compass_cal(degree_ref + 30); //stage 1
  long start_time = millis();
  int ad_time = 450;
  while (millis() < start_time + ad_time) {
    backward();
    compass_loop_ad();
    xyPID.Compute();
  }
  stall();

  degree_ref = compass_cal(degree_ref - 30);  //stage 2
  while (millis() < start_time + 2*ad_time) {
    backward();
    compass_loop_ad();
    xyPID.Compute();
  }
  stall();

  while (millis() < start_time + 4*ad_time) {  //stage 3
    forward();
    compass_loop_ad();
    xyPID.Compute();
  }
  stall();
  return;
}

void heading_to(double heading_target) {
  compass_loop();
  if (compass_cal(heading_target - degree_ref) > 0) {
    degree_ref = heading_target;
    while (compass_cal(heading_degree - degree_ref) < -1) {
      turn_left();
      turnPID.Compute();
      compass_loop();
    }
  }
  else {
    degree_ref = heading_target;
    while (compass_cal(heading_degree - degree_ref) > 1) {
      turn_right();
      turnPID.Compute();
      compass_loop();
    }
    stall();
  }
}

void setup() {
  //Start compass setup............................................................................................
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(-3);
  mpu.setYGyroOffset(27);
  mpu.setZGyroOffset(85);
  mpu.setZAccelOffset(4970);

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  //compass_setup_end...............................................................................................

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel B pin
  //setup Channel A
  pinMode(12, OUTPUT); //Initiates the Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates the Brake channel A pin
  gripper.attach(gripper_pin);
  gripper.write(95);

  xyPID.SetOutputLimits(-30.0, 30.0);
  //xyPID.SetSampleTime(10);
  xyPID.SetMode(AUTOMATIC);
  turnPID.SetOutputLimits(-100.0, 100.0);
  //turnPID.SetSampleTime(10);
  turnPID.SetMode(AUTOMATIC);
}

void loop() {
  if (first_wait) {
    delay(20000); //Serial.println("ready");
  }
  compass_loop_ad();
  if (first_wait) {
    degree_ref = heading_degree;
    first_wait = false;
  }
  //.......................................................................................
  compass_loop();
  if (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == 'f') {
      moving_forward();
    }
    else if (ch == 'N') {
      bias = Serial.parseInt();
      forward();
    }
    else if (ch == 'q') {
      move_to_left_a_bit(); //in case we are too close to the walls.
    }
    else if (ch == 'e') {
      move_to_right_a_bit();
    }
    else if (ch == 'b') {
      moving_backward();
    }
    else if (ch == 'l') {
      compass_loop_ad();
      degree_ref = heading_degree;
      turn_left_test();
      Serial.print('l'); //tell manager the left turn is complete
    }
    else if (ch == 'r') {
      compass_loop_ad();
      degree_ref = heading_degree;
      turn_right_test();
      Serial.print('r'); //tell manager the right turn is complete
    }
    else if (ch == 's') {
      stall();
    }
    else if (ch == 'p') {
      pickup();
    }
    else if (ch == 'd') {
      putdown();
    }
    else if (ch == 'm') {
      left_adjust();
    }
    else if (ch == 'n') {
      right_adjust();
    }
    else if (ch == 'x') {  //update the compass by update the degree_ref
      degree_ref = heading_degree;
    }
    else if (ch == 'j') {  //tilt to left
      tilt_left();
    }
    else if (ch == 'k') {  //tilt to right
      tilt_right();
    }
    else if (ch == 'C') {  
      Serial.print(heading_degree);
    }
  }
}
