//Serial1:slave
//Serial2:bluetooth
#include <NewPing.h>
#include "Wire.h"

#define ULTRA_TRIGGER_PIN 7
#define ECHO_PIN_BLOCK  3  //FRONT BLOCK SENSOR
#define ECHO_PIN_F  4  //FRONT SENSOR
#define ECHO_PIN_B  8  //BACK SENSOR
#define ECHO_PIN_LF  6 //LEFT FRONT SENSOR
#define ECHO_PIN_LB  9 //LEFT BACK SENSOR
#define ECHO_PIN_RF  5 //RIGHT BACK SENSOR
#define ECHO_PIN_RB  10 //RIGHT BACK SENSOR

#define MAX_DISTANCE 200

NewPing sonar_block(ULTRA_TRIGGER_PIN, ECHO_PIN_BLOCK, MAX_DISTANCE);
NewPing sonar_front(ULTRA_TRIGGER_PIN, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonar_back(ULTRA_TRIGGER_PIN, ECHO_PIN_B, MAX_DISTANCE);
NewPing sonar_left_front(ULTRA_TRIGGER_PIN, ECHO_PIN_LF, MAX_DISTANCE);
NewPing sonar_right_front(ULTRA_TRIGGER_PIN, ECHO_PIN_RF, MAX_DISTANCE);
NewPing sonar_left_back(ULTRA_TRIGGER_PIN, ECHO_PIN_LB, MAX_DISTANCE);
NewPing sonar_right_back(ULTRA_TRIGGER_PIN, ECHO_PIN_RB, MAX_DISTANCE);

int r_block, r_front, r_back, r_left_front, r_right_front, r_left_back, r_right_back;

int stop_distance = 10; //need to test to update, the distance to the front when stops, [7, 9];

void ultrasonic_read_block() {
  delay(30);
  r_block = sonar_block.convert_cm(sonar_block.ping_median(1));
}

void ultrasonic_read_all() {
  delay(30);
  r_block = sonar_block.convert_cm(sonar_block.ping_median(1));
  delay(30);
  r_front = sonar_front.convert_cm(sonar_front.ping_median(1));
  delay(30);
  r_back = sonar_back.convert_cm(sonar_back.ping_median(1));
  delay(30);
  r_left_front = sonar_left_front.convert_cm(sonar_left_front.ping_median(1));
  delay(30);
  r_right_front = sonar_right_front.convert_cm(sonar_right_front.ping_median(1));
  delay(30);
  r_left_back = sonar_left_back.convert_cm(sonar_left_back.ping_median(1));
  delay(30);
  r_right_back = sonar_right_back.convert_cm(sonar_right_back.ping_median(1));


  Serial.print("D_block: ");
  Serial.print(r_block);
  Serial.print("D_front: ");
  Serial.print(r_front);
  Serial.print(" D_back: ");
  Serial.print(r_back);
  Serial.print(" D_left_front: ");
  Serial.print(r_left_front);
  Serial.print(" D_left_back: ");
  Serial.print(r_left_back);
  Serial.print(" D_right_front: ");
  Serial.print(r_right_front);
  Serial.print(" D_right_back: ");
  Serial.println(r_right_back);
  return;
}


void ultrasonic_read_front() {
  delay(30);
  r_front = sonar_front.convert_cm(sonar_front.ping_median(1));
  return;
}

void ultrasonic_read_back() {
  delay(30);
  r_back = sonar_back.convert_cm(sonar_back.ping_median(1));
  return;
}

void forward_eee (int distance = 30) {
  Serial.println("case 2");
  int init_position = r_front;
  int final_position = r_front;
  //while (init_position - final_position < 10) {  //in case it doesn't move.
  while (r_front > (init_position - distance + 5)) {       //"first movement" case 2
    Serial1.print('f');
    ultrasonic_read_front();
  }
  Serial1.print('s');
  while ((r_front % 30 > stop_distance + 1) || (r_front % 30 < stop_distance - 1)) {  //"adjustment" case 2
    if (r_front % 30 > stop_distance + 1) {
      long start_time = millis();
      Serial1.print('f');
      while (millis() < start_time + 200) {
        //hold;
      }
      Serial1.print('s');
    }
    else if (r_front % 30 < stop_distance - 1) {
      Serial.println("b_ad");
      Serial1.print('b');
      long start_time = millis();
      while (millis() < start_time + 200) {
        //hold;
      }
      Serial1.print('s');
    }
    delay(500);
    ultrasonic_read_front();
    final_position = r_front;
  }
  //}
}

void forward(int distance) {  //in our case, distance is either 30 or 0;
  //  Serial.print("forward: ");
  //  Serial.print(distance);
  //  Serial.println(".");
  ultrasonic_read_all();

  if (distance == 0) {
    //   Serial.print("case 1");
    while (r_front > stop_distance + 1) {        //"fisrt movement" case 1
      Serial1.print('f');
      ultrasonic_read_front();
    }
    Serial1.print('s');
    while ((r_front > stop_distance + 1) || (r_front < stop_distance - 1)) {        //"adjustment" case 1
      if (r_front > stop_distance + 1) {
        long start_time = millis();
        Serial1.print('f');
        while (millis() < start_time + 200) {
          //hold;
        }
        Serial1.print('s');
      }
      else if (r_front < stop_distance - 1) {
        long start_time = millis();
        Serial1.print('b');
        while (millis() < start_time + 200) {
          //hold;
        }
        Serial1.print('s');
      }
      ultrasonic_read_front();
    }
  }
  else {
    if (r_front <= r_back) { //120
      Serial.println("case 2");
      int init_position = r_front;
      int final_position = r_front;
      //while (init_position - final_position < 10) {  //in case it doesn't move.
      while (r_front > (init_position - distance + 5)) {       //"first movement" case 2
        Serial1.print('f');
        ultrasonic_read_front();
      }
      Serial1.print('s');
      while ((r_front % 30 > stop_distance + 1) || (r_front % 30 < stop_distance - 1)) {  //"adjustment" case 2
        if (r_front % 30 > stop_distance + 1) {
          long start_time = millis();
          Serial1.print('f');
          while (millis() < start_time + 200) {
            //hold;
          }
          Serial1.print('s');
        }
        else if (r_front % 30 < stop_distance - 1) {
          Serial.println("b_ad");
          Serial1.print('b');
          long start_time = millis();
          while (millis() < start_time + 200) {
            //hold;
          }
          Serial1.print('s');
        }
        delay(500);
        ultrasonic_read_front();
        final_position = r_front;
      }
      //}
    }
    else if (r_front > r_back) {  //120
      Serial.print("case 3");
      int init_position = r_back;
      int final_position = r_back;
      //while (final_position - init_position < 10) { //just incase it doesn't move
      while (r_back < (init_position + distance - 3)) {       //"first movement" case 3
        Serial1.print('f');
        ultrasonic_read_back();
      }
      Serial1.print('s');
      while ((r_back % 30 > 16 - stop_distance + 1) || (r_back % 30 < 16 - stop_distance - 1)) { //"adjustment"case 3
        if (r_back % 30 > 16 - stop_distance + 1) {
          Serial.println("b_ad");
          long start_time = millis();
          Serial1.print('b');
          while (millis() < start_time + 200) {
            //hold;
          }
          Serial1.print('s');
        }
        else if (r_back % 30 < 16 - stop_distance - 1) {
          Serial.println("f_ad");
          Serial1.print('f');
          long start_time = millis();
          while (millis() < start_time + 200) {
            //hold;
          }
          Serial1.print('s');
        }
        delay(500);
        ultrasonic_read_back();
      }
      //}
    }
  }
  return;
}

void compass_calibration() {
  ultrasonic_read_all();
  if ((r_right_front < 11 && r_right_back < 11) && (r_left_front < 11 && r_left_back < 11)) {
    while (r_right_front != r_right_back && r_left_front != r_left_back) { //if not working, add a for loop.
      if (r_right_front < r_right_back || r_left_front > r_left_back) {
        Serial1.print('j'); //left_tilt
      }
      else if (r_right_front > r_right_back || r_left_front < r_left_back) {
        Serial1.print('k'); //right_tilt
      }
      delay(500);
      ultrasonic_read_all();
    }
  }
  else if (r_right_front < 14 && r_right_back < 14) {
    while (r_right_front != r_right_back) {
      if (r_right_front < r_right_back) {
        Serial1.print('j'); //left_tilt
      }
      else if (r_right_front > r_right_back) {
        Serial1.print('k'); //right_tilt
      }
      delay(500);
      ultrasonic_read_all();
    }
  }
  else if (r_left_front < 14 && r_left_back < 14) {
    while (r_left_front != r_left_back) {
      if (r_left_front > r_left_back) {
        Serial1.print('j'); //left_tilt
      }
      else if (r_left_front < r_left_back) {
        Serial1.print('k'); //right_tilt
      }
      delay(500);
      ultrasonic_read_all();
    }
  }
  Serial1.print('x'); //update the degree_ref
  delay(50);
}

void side_check() {
  ultrasonic_read_all();
  if (r_right_front + r_right_back < 8) {
    Serial1.print('q'); //move to left a bit
  }
  else if (r_left_front + r_left_back < 8) {
    Serial1.print('e'); //move to right a bit
  }
}

bool block_found() {
  delay(2000);
  ultrasonic_read_block();
  ultrasonic_read_front();
  //if (r_block < 25 && r_block > 0 && abs(r_block - r_front) > 5) {
  if (r_block < 25 && r_block > 4) {
    return true;
  }
  else
    return false;
}

bool block_found_double() {
  return true;
//    delay(2000);
//    ultrasonic_read_block();
//    ultrasonic_read_front();
//    if (r_block < 8 && r_block < r_front - 4) {
//      return true;
//    }
//    else
//      return false;
}

void find_block() {
  Serial1.print('d');

  ultrasonic_read_all();

  delay(5000);
  Serial1.print('f'); //around 25cm
  delay(2000);
  Serial1.print('s');
  delay(1000);
  ultrasonic_read_all();

  if (true) {
    while (true) {
      Serial1.print('r');

      while (Serial1.available() == 0) {
      }
      Serial1.read();
      delay(6000);

      if (block_found()) {

        Serial1.print('f');
        delay(1500);
        Serial1.print('s');
        delay(1500);

        if (block_found_double()) {  //double check
          Serial1.print('p');   //抓
          delay(1000);
          Serial1.print('l');
          delay(3000);
          forward(0);
          Serial1.print('r');
          delay(3000);
          compass_calibration();
          delay(1000);
          forward(0);
          break;
        }
        
      }
      delay(1000);

      Serial1.print('l');
      while (Serial1.available() == 0) {
      }

      Serial1.read();
      delay(6000);

      Serial1.print('f');
      delay(600);
      Serial1.print('s');
      ultrasonic_read_all();

      if (false) {

        Serial1.print('f');
        delay(1500);
        Serial1.print('s');
        delay(3000);
        if (block_found_double()) {

          Serial1.print('p');
          delay(2000);
          ultrasonic_read_all();
          if (r_right_front > r_left_front) {
            forward(0);
            Serial1.print('r');
            delay(3000);
            forward(0);
          }
          else {
            forward(0);
            Serial1.print('l');
            delay(3000);
            compass_calibration();
            delay(1000);
            forward(0);
          }
          return;
        }
        else {
          Serial1.print('b');
          delay(1300);
          Serial1.print('s');
        }
      }
      delay(1000);
    }

  }
//  else { //comments
//    while (true) {
//      Serial1.print('l');
//
//      while (Serial1.available() == 0) {
//      }
//      char test = Serial1.read();
//      delay(1000);
//
//
//      if (block_found()) {
//        Serial1.print('f');
//        delay(1400);
//        Serial1.print('s');
//        Serial1.print(2000);
//        if (block_found_double()) {
//          Serial1.print('p');   //抓
//          delay(1000);
//          Serial1.print('r');
//          delay(3000);
//          forward(0);
//
//          Serial1.print('l');
//          delay(3000);
//          compass_calibration();
//          delay(1000);
//          forward(0);
//          //break;
//        }
//        else {
//          Serial1.print('b');
//          delay(1400);
//          Serial1.print('s');
//        }
//      }
//      delay(1000);
//
//      Serial1.print('r');
//      while (Serial1.available() == 0) {
//      }
//
//      Serial1.read();
//      delay(6000);
//
//      Serial1.print('f');
//      delay(400);
//      Serial1.print('s');
//      //      while (r_right_front < r_right_back) {    //for testing only
//      //        Serial1.print('j');
//      //      }
//      //      while (r_right_front > r_right_back) {
//      //        Serial1.print('k');
//      //      }
//      if (block_found()) {
//
//        Serial1.print('f');
//        delay(1300);
//        Serial1.print('s');
//        Serial1.print(3000);
//        if (block_found_double()) {
//          Serial1.print('p');
//          delay(2000);
//          ultrasonic_read_all();
//          if (r_right_front > r_left_front) {
//            forward(0);
//            Serial1.print('r');
//            delay(3000);
//            forward(0);
//          }
//          else {
//            forward(0);
//            Serial1.print('l');
//            delay(3000);
//            compass_calibration();
//            delay(1000);
//            forward(0);
//          }
//          return;
//        }
//        else {
//          Serial1.print('b');
//          delay(1300);
//          Serial1.print('s');
//        }
//      }
//      delay(1000);
//    }
//  }    //comments
}

//void u_turn() {
//  compass_loop_ad();
//  double heading_target = compass_cal(heading_degree + 180);
//  double heading_new = heading_degree;
//  while (abs(compass_cal(heading_new - heading_target)) > 2.0) {
//    Serial1.print('l');
//    delay(200);
//    Serial1.print('s');
//    compass_loop_ad();
//    heading_new = heading_degree;
//  }
//  Serial1.print('s');
//}


//void heading_to(double heading_target) {
//  compass_loop_ad();
//  double heading_current = heading_degree;
//  if (compass_cal(heading_target - heading_current) >= 0) {
//    //Serial.print("left adjust");
//    Serial1.print("l");
//    while (abs(compass_cal(heading_current - heading_target)) > 1.0) {
//      compass_loop();
//      heading_current = heading_degree;
//    }
//    Serial1.print("s");
//  }
//  else {
//    Serial1.print("r");
//    //Serial.print("right adjust");
//    while (abs(compass_cal(heading_current - heading_target)) > 1.0) {
//      compass_loop();
//      heading_current = heading_degree;
//    }
//    Serial1.print("s");
//  }
//}

//bool ulrasonic_emergency() {
//  ultrasonic_read_alldirct();
//  if (r_left > 30 || r_right > 30) {
//    if (r_left > 9 && r_right > 30) {
//      Serial1.print('l');
//      delay(200);
//      //delay(r_left*20);
//      Serial1.print('s');
//    } else if (r_left <= 5 && r_right > 30) {
//      Serial1.print('r');
//      delay(300);
//      //delay(300-r_left*30);
//      Serial1.print('s');
//      if (r_left < 3) {
//        Serial1.print('f');
//        delay(300);
//        Serial1.print('s');
//      }
//    } else if (r_right > 9 && r_left > 30) {
//      Serial1.print('r');
//      delay(200);
//      //delay(r_right*20);
//      Serial1.print('s');
//    } else if (r_right <= 5 && r_left > 30) {
//      Serial1.print('l');
//      delay(300);
//      //delay(200-r_right*20);
//      Serial1.print('s');
//      if (r_right < 3) {
//        Serial1.print('f');
//        delay(300);
//        Serial1.print('s');
//      }
//    } else if (r_left < 15 && r_right < 15) {
//      if (r_right <= 5 && r_left > 8) {
//        Serial1.print('r');
//        delay(300);
//        //delay(r_left*20);
//        Serial1.print('s');
//        if (r_right < 3) {
//          Serial1.print('f');
//          delay(300);
//          Serial1.print('s');
//        }
//      } else if (r_left <= 5 && r_right > 8) {
//        Serial1.print('l');
//        delay(300);
//        //delay(r_right*20);
//        Serial1.print('s');
//        if (r_left < 3) {
//          Serial1.print('f');
//          delay(200);
//          Serial1.print('s');
//        }
//      }
//    }
//    Serial1.print('f');
//    delay(100);
//    Serial1.print('s');
//    return true;
//  }
//  else
//    return false;
//}

void flashing() { //blink when block is detected
  long fs = millis();
  while (millis() < fs + 2000) {
    digitalWrite(53, HIGH);
    delay(100);
    digitalWrite(53, LOW);
    delay(100);
  }
  digitalWrite(53, LOW);
}

void setup() {
  Serial3.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial.begin(9600);
  Serial.println("start");
  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);
  //delay(22000);
  Serial.println("ready");
}

void loop() {
  //   find_block();
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'p') {
      Serial.println(command);
      Serial.println("begin");
      find_block();
    }
  }


  if (Serial3.available()) {
    char command = Serial3.read();
    Serial.println(command);
    if (command == 'u') {
      Serial3.println("CEO, Manager is ready!");
    }
    else if (command == 'f') {
      forward(30);
      delay(100);
      side_check();
      delay(1000);
      compass_calibration();
      Serial3.println('c');
    }
//    else if (command == 'D'){
//      Serial1.print('d');
//    }
    else if (command == 'o') {
      //forward(0);
      forward_eee(30);
      delay(100);
      side_check();
      delay(1000);
      compass_calibration();
      Serial3.println('c');
    }
    else if (command == 'l') {
      Serial1.print('l');
    }
    else if (command == 'r') {
      Serial1.print('r');
    }
    else if (command == 's') {
      ultrasonic_read_all();
      Serial3.println(r_front);
      Serial3.println(r_left_front);
      Serial3.println(r_left_back);
      Serial3.println(r_right_front);
      Serial3.println(r_right_back);
      Serial3.println(r_back);
    }
    else if (command == 'C') {
      Serial1.print('C');
      while (Serial1.available() == 0) {
        //hold;
      }
      double compass_value = Serial1.parseFloat();
      Serial.print(compass_value);
      Serial3.println(compass_value);
    }
    else if (command == 'x') {
      compass_calibration();
    }
    else if (command == 'X') {
      Serial1.print('x');
    }
    else if (command == 'L') {
      find_block();
      //Serial3.println('c');
    }
    else if (command == 'v') {  //led on
      flashing();
      //Serial3.println('c');
    }
    else if (command == 't') { //drop off the block
      Serial1.print('b');
      long endcount = millis();
      while (millis() < endcount + 1000) {
      }
      Serial1.print('s');
      delay(200);
      Serial1.print('d');
      delay(5000);
      Serial1.print('b');
      endcount = millis();
      while (millis() < endcount + 1000) {
      }
      Serial1.print('s');
      Serial1.print('p');
    }
  }

  if (Serial1.available()) {
    char report = Serial1.read();
    if (report == 'r') {
      Serial3.println('c'); //tell the CEO the turns are completed.
    }
    if (report == 'l') {
      Serial3.println('c');
    }
  }
}
