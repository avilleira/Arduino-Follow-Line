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
#define PIN_ITR20001_LEFT   'A2'
#define PIN_ITR20001_MIDDLE 'A1'
#define PIN_ITR20001_RIGHT  'A0'

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

#define team_name "Robotitos"
#define ID_EQUIPO  "9"
#define server "193.147.53.2"
#define port 21883

const char* ssid = "sensoresurjc";
const char* password = "Goox0sie_WZCGGh25680000";


// Messages
int state, previus_state;
long tracking_time = 0;
bool line_lost_searching;
String start[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "START_LAP"}};
String finish[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "END_LAP"}, {"time: ", (char *)tracking_time}};
String obstacle[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "OBSTACLE_DETECTED"}};
String line_lost[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "LINE_LOST"}};
String PING[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "PING"}, {"time: ", (char *)tracking_time}};
String searching_line[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "INIT_LINE_SEARCH"}};
String line_found[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "LINE_FOUND"}};
String stop_searching[][2] = {{"team_name: ", team_name}, {"id: ", ID_EQUIPO}, {"action: ", "STOP_LINE_SEARCH"}};

void setup(){
  Serial.begin(9600);

  // line-tracking
  pinMode(PIN_ITR20001_LEFT, INPUT);
  pinMode(PIN_ITR20001_MIDDLE, INPUT);
  pinMode(PIN_ITR20001_RIGHT, INPUT);

  // ultrasonic
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  state = 0;
}
 
void loop(){
  bool stop = stop_car;
  if (stop) {
    state = 3;
  } else {
    state = line_tracking();
  }
  // estado 0 avanzar
  // estado 1 girar
  // estado 2 encontrar linea
  // estado 3 parar
  
  previus_state = state;
}

int line_tracking() {
  float lineL = line_L();
  float lineM = line_M();
  float lineR = line_R();
  int estado;

  if (lineM < 800) {
    estado = 0;
  } else {
    if ((lineR < 800) || (lineL < 800)) {
      estado = 1;
    } else {
      estado = 2;
    }
  }

  return estado;
}

float line_L(void) {
  return analogRead(PIN_ITR20001_LEFT);
}
float line_M(void) {
  return analogRead(PIN_ITR20001_MIDDLE);
}
float line_R(void) {
  return analogRead(PIN_ITR20001_RIGHT);
}

float UltraSonic_DistanceCm()
{
  float distance = 0;
  digitalWrite(TRIG_PIN, LOW);
  float time = millis();
  while (millis() - time < 2);
  digitalWrite(TRIG_PIN, HIGH);
  while (millis() - time < 12);
  digitalWrite(TRIG_PIN, LOW);
  distance = pulseIn(ECHO_PIN, HIGH) / 58;
  return distance;
}

bool stop_car() {
  float distance = UltraSonic_DistanceCm();
  if (distance < 10){
    for (int i = 0;i <3; i++) {
      Serial.write("data");
    }
    return true;
  }
  return false;
}
