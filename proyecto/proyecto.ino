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
#include "FastLED.h"

// Variables
#define NUM_LEDS 1
#define MAX_DISTANCE 200 

// LED
#define PIN_RBGLED 4

// Ultrasonic sensor:
#define TRIG_PIN 13  
#define ECHO_PIN 12 

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

#define SPEED_MAX 200

#define team_name "Robotitos"
#define ID_EQUIPO  "9"
#define server "193.147.53.2"
#define port 21883

const char* ssid = "sensoresurjc";
const char* password = "Goox0sie_WZCGGh25680000";

int state, previus_state;
unsigned long tracking_time = 0;
unsigned long time_lost_init;
bool line_lost_searching;
bool stop;
const int vmin=100;
const int vmax=200;
const float kp=0.07;
const float ki=0.0004;
const float kd=0.01;
const float kv=1.5;
int p,d,u,vbase;
long i=0;
int p_old=0;
enum actions {
  START, FINISH, OBSTACLE, LINE_LOST, LINE_FOUND, SEARCHING_LINE, STOP_SEARCHING, PING };
CRGB leds[NUM_LEDS];

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

  // Motors  
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);

  // ultrasonic
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // LED setup
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  //Inicio de la máquina de estados:
  state = -1;
  stop = false;
}

void loop(){
  // estado -1 inicio
  // estado 0 avanzar
  // estado 1 girar
  // estado 2 encontrar linea
  // estado 3 parar

  if (state == -1) {
    String message = serialPort_read();
    FastLED.showColor(Color(255, 0, 0));
    /*while (message == "None") {
      message = serialPort_read();
    }*/
    Serial.println(String(START) + "}");
  }
  long time_init = millis();
  long time_finish;
  while (!stop) {
    stop = stop_car();
    if (stop) {
      state = 3;
    } else {
      state = line_tracking();
    }
    
    if (state == 0) {
      FastLED.showColor(Color(0, 255, 0));
      previus_state = "foward";
      follow_line();
    } else {
      if (state == 1) {
        FastLED.showColor(Color(0, 255, 0));
        if(line_R() > 100) {
          previus_state = "right";
          follow_line();
        } else {
          if(line_L() > 100) {
            previus_state = "left";
            follow_line();
          } 
        }
      } else {
        if (state == 2) {
          if (line_lost_searching) {
            line_lost_searching = false;
            FastLED.showColor(Color(0, 0, 255));
            Serial.println(String(SEARCHING_LINE) + "}");
          }

          if (previus_state == "right") {
            right();            
          } else {
            if (previus_state == "left") {
              left();              
            }
          }
        }        
        if (state == 3) {
          FastLED.showColor(Color(255, 0, 255));
          stop_movement();
          Serial.println(String(FINISH) + "}");
          stop = true;
        }
      }     
    }
  }
}

// Seguir de frente
void forward() {
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, SPEED_MAX);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, SPEED_MAX);

  Serial.println("go forward!");
}

// Girar a la izquierda
void left(){
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, SPEED_MAX);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, SPEED_MAX);
  
  Serial.println("go left!");
}

// Girar a la derecha
void right(){
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, SPEED_MAX);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, SPEED_MAX);

  Serial.println("go right!");
}

// Detener el movimiento
void stop_movement(){
  digitalWrite(PIN_Motor_STBY, false);
  analogWrite(PIN_Motor_PWMB, 0);
  analogWrite(PIN_Motor_PWMA, 0);

  Serial.println("stop!");
}

uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void follow_line() {
  float lineL = line_L();
  float lineM = line_M();
  float lineR = line_R();
  p = lineL - lineR;
  i=i+p;
  d=p-p_old;
  p_old=p;
  if ((p*i)<0) i=0;

  u=kp*p+ki*i+kd*d;
  vbase=vmin+(vmax-vmin)*exp(-kv*abs(kp*p));
  drive(vbase+u,vbase-u);
}

void drive(int L, int R) {
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, L);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, R);
}

// Calculo de giro
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
      line_lost_searching = false;
      time_lost_init = millis();
    } else {
      if (!line_lost_searching) {
        estado = 2;
      }
      if (millis() - time_lost_init > 1500) {
        Serial.println(String(LINE_LOST) + "}");
        line_lost_searching = true;       
      }
      if (millis() - time_lost_init > 5000) {
        estado = 3;       
      }      
    }
  }
  return estado;
}

// Lecturas del sensor infrarojo
float line_L(void) {
  return analogRead(PIN_ITR20001xxxL);
}
float line_M(void) {
  return analogRead(PIN_ITR20001xxxM);
}
float line_R(void) {
  return analogRead(PIN_ITR20001xxxR);
}

// Lectura del sensor ultrasonido
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

// Calculo de la distancia al obstaculo
bool stop_car() {
  float distance = UltraSonic_DistanceCm();
  if (distance < 8){ // 8 cm
    Serial.println(String(OBSTACLE) + "}");
    return true;
  }
  return false;
}

// Lectura del puerto serie
String serialPort_read() {
  static String SerialPortData = "";
  uint8_t c = "";
  if (Serial.available() > 0) {
    while (c != '}' && Serial.available() > 0) {
      c = Serial.read();
      SerialPortData += (char)c;
    }
    return SerialPortData;
  } else {
    return "None";
  }
}
