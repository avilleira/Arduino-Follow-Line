#include "Arduino.h"
#include "FastLED.h"

// LED:
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

// definiciones
#define NUM_LEDS 1
#define MAX_DISTANCE 12
#define SPEED_MAX 150

#define team_name "Robotitos"
#define ID_EQUIPO  "9"
#define server "193.147.53.2"
#define port 21883

// internet data
const char* ssid = "sensoresurjc";
const char* password = "Goox0sie_WZCGGh25680000";

// Variables
int state, previus_state;
unsigned long time_lost_init;
bool line_lost_searching;
bool lost;
bool stop;
const int vmin=100;
const int vmax=150;
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
  lost = false;
  previus_state = "right";
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
    while (message == "None") {
      message = serialPort_read();
    }
    Serial.println(String(START) + "}");
  }

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
        if(line_R() > 500) {
          previus_state = "right";
          follow_line();
        } else {
          if(line_L() > 500) {
            previus_state = "left";
            follow_line();
          } 
        }
      } else {
        if (state == 2) {
          if ((line_lost_searching) && (lost == true)) {
            line_lost_searching = false;
            FastLED.showColor(Color(0, 0, 255));
            Serial.println(String(SEARCHING_LINE) + "}");
          }

          if (previus_state == "right") {
            right();            
          } else {
            left();
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

// Girar a la izquierda
void left(){
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, SPEED_MAX);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, SPEED_MAX);
}

// Girar a la derecha
void right(){
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, SPEED_MAX);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, SPEED_MAX);
}

// Detener el movimiento
void stop_movement(){
  digitalWrite(PIN_Motor_STBY, false);
  analogWrite(PIN_Motor_PWMB, 0);
  analogWrite(PIN_Motor_PWMA, 0);
}

uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

// Calculo de PID del sigue linea
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

// Navegar con la velocidad obtenida del PID
void drive(int L, int R) {
  digitalWrite(PIN_Motor_STBY, true);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, L);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, R);
}

// Ver en que estado entrar
int line_tracking() {
  float lineL = line_L();
  float lineM = line_M();
  float lineR = line_R();
  int estado;

  // Seguir de frente
  if (lineM > 500) {
    estado = 0;
    line_lost_searching = false;
    time_lost_init = millis();
    if (lost) {
      Serial.println(String(LINE_FOUND) + "}");
      lost = false;
    }
  } else {
    // Girar
    if ((lineR > 500) || (lineL > 500)) {
      estado = 1;
      line_lost_searching = false;
      time_lost_init = millis();
      if (lost) {
        Serial.println(String(LINE_FOUND) + "}");
        lost = false;
      }
    // linea perdida
    } else {
      estado = 2;
      if (!line_lost_searching){
        Serial.println(String(LINE_LOST) + "}");
        line_lost_searching = true;    
        lost = true;   
      }
      if (millis() - time_lost_init > 3000) {
        Serial.println(String(STOP_SEARCHING) + "}");
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
  if (distance < MAX_DISTANCE) { // En realidad queda a 7 cm
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
