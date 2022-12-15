/*
* Copyright (C) 2022 by Roberto Calvo-Palomino
*
*
*  This programa is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with RTL-Spec.  If not, see <http://www.gnu.org/licenses/>.
*
*   Authors: Roberto Calvo-Palomino <roberto.calvo [at] urjc [dot] es>
*/

// Subject: Sistemas Empotrados y de Tiempo Real
// Universidad Rey Juan Carlos, Spain
#include "Arduino.h"

// Variables
#define TRIG_PIN 13  
#define ECHO_PIN 12 
#define MAX_DISTANCE 200 
// INFRA RED SENSOR:
#define PIN_ITR20001xxxL A2
#define PIN_ITR20001xxxM A1
#define PIN_ITR20001xxxR A0


// Enable/Disable motor control.
//  HIGH: motor control enabled
//  LOW: motor control disabled
#define PIN_Motor_STBY 3

// Group A Motors (Right Side)
// PIN_Motor_AIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_AIN_1 7
// PIN_Motor_PWMA: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMA 5

// Group B Motors (Left Side)
// PIN_Motor_BIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_BIN_1 8
// PIN_Motor_PWMB: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMB 6

#define SPEED 50

#define team_name "Robotitos"
#define ID_EQUIPO  "9"
#define server "193.147.53.2"
#define port 21883

const char* ssid = "sensoresurjc";
const char* password = "Goox0sie_WZCGGh25680000";

int state, previus_state;
long tracking_time = 0;
bool line_lost_searching;
bool stop;
enum actions {
  START, FINISH, OBSTACLE, LINE_LOST, LINE_FOUND, SEARCHING_LINE, STOP_SEARCHING, PING
};

/*
String start[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "START_LAP"}};
String finish[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "END_LAP"}, {"time: ", (char *)tracking_time}};
String obstacle[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "OBSTACLE_DETECTED"}};
String line_lost[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "LINE_LOST"}};
String PING[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "PING"}, {"time: ", (char *)tracking_time}};
String searching_line[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "INIT_LINE_SEARCH"}};
String line_found[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "LINE_FOUND"}};
String stop_searching[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "STOP_LINE_SEARCH"}};
*/

void setup(){
  Serial.begin(9600);

  // line-tracking
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);

  // ultrasonic
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  //Inicio de la máquina de estados:
  state = 0;
  stop = false;
}


void forward() {
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, SPEED);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, SPEED);
  Serial.println("go forward!");
}

void left(){
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, SPEED);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, SPEED);
  
  Serial.println("go left!");
}

void right(){
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, SPEED);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, SPEED);
  Serial.println("go right!");
}

void stop_movement(){
  digitalWrite(PIN_Motor_STBY, false);
  analogWrite(PIN_Motor_PWMB, 0);
  analogWrite(PIN_Motor_PWMA, 0);
  Serial.println("stop!");
}

void loop(){
  while (!stop) {
    stop = stop_car();
    if (stop) {
      state = 3;
    } else {
      state = line_tracking();
    }
    
    if (state == 0) {
      previus_state = "foward";
      forward();
    } else {
      if (state == 1) {
        if(line_R() > 100) {
          previus_state = "right";
          right();
        } else {
          if(line_L() > 100) {
            previus_state = "left";
            left();
          } 
        }
      } else {
        if (state == 2) {
          if (previus_state == "right") {
            right();            
          } else {
            if (previus_state == "left") {
              left();              
            }
          }
        }        
        if (state == 3) {
          stop_movement();
        }
      }     
    }

    // estado 0 avanzar
    // estado 1 girar
    // estado 2 encontrar linea
    // estado 3 parar
  }
}

int line_tracking() {
  float lineL = line_L();
  float lineM = line_M();
  float lineR = line_R();
  int estado;
  if (lineM > 100) {
    estado = 0;
  } else {
    if ((lineR > 100) || (lineL > 100)) {
      estado = 1;
    } else {
      estado = 2;
    }
  }

  return estado;
}

float line_L(void) {
  return analogRead(PIN_ITR20001xxxL);
}
float line_M(void) {
  return analogRead(PIN_ITR20001xxxM);
}
float line_R(void) {
  return analogRead(PIN_ITR20001xxxR);
}

float UltraSonic_DistanceCm()
{
  digitalWrite(TRIG_PIN, LOW);
  float time = millis();
  while (millis() - time < 2);
  digitalWrite(TRIG_PIN, HIGH);
  while (millis() - time < 12);
  digitalWrite(TRIG_PIN, LOW);
  float speedOfSound = 0.03313 + 0.0000606;

  unsigned long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration / 2.0 * speedOfSound;
  return distance;
}

bool stop_car() {
  float distance = UltraSonic_DistanceCm();
  if (distance < 8){ // 8 cm
    Serial.write(OBSTACLE);
    return true;
  }
  return false;
}

void serialPort_read() {
  static String SerialPortData = "";
  uint8_t c = "";
  if (Serial.available() > 0) {
    while (c != '}' && Serial.available() > 0) {
      c = Serial.read();
      SerialPortData += (char)c;
    }
  }
}
