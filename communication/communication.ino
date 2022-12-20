/***************************************************
  Adafruit MQTT Library ESP8266 Example
  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino
  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>

// the on off button feed turns this LED on/off
#define LED 2  
// the slider feed sets the PWM output of this pin
#define PWMOUT 12

/************************* WiFi Access Point *********************************/

#define WLAN_SSID "MIWIFI_5G"  
//"vodafone8653"
// "MIWIFI_5G"

#define WLAN_PASS   "POYOYO64"
//"Goox0sie_WZCGGh25680000"
// "POYOYO64"
/************************* Adafruit.io Setup *********************************/

#define SERVER      "193.147.53.2"
#define SERVERPORT  21883                   // use 8883 for SSL
#define USERNAME    "Robotitos"
#define KEY         "9"

/************ Global State (you don't need to change this!) ******************/

/***************** Serial2 variables******************************************/
#define RXD2 33
#define TXD2 4

// Enumeration of all the possible actions:
enum actions {
  START,
  FINISH,
  OBSTACLE, 
  LINE_LOST, 
  LINE_FOUND, 
  SEARCHING_LINE, 
  STOP_SEARCHING, 
  PING };
// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, SERVER, SERVERPORT, USERNAME, USERNAME, KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish pub = Adafruit_MQTT_Publish(&mqtt, "/SETR/2022/9/");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.

// Creating JSON objects:

 //JSONVar start_lap;
 //
 //start_lap["team_name"] = "Robotitos";
 //start_lap["id"] = 9;

 //Creating all the possible messages. We define two differents types of JSON messages, first with time stamp and second without it:
 StaticJsonDocument<256> msg_no_time; // For START, OBSTACLE_DETECTED, LINE_LOST, SEARCHING_LINE, STOP_SEARCHING, LINE_FOUND
 StaticJsonDocument<256> msg_time;


void initWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
}
  
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }

  Serial.println("MQTT Connected!");
  digitalWrite(LED, HIGH);
  Serial2.println("A");
}

String sendBuff;

void recv_serial_msg() {
  if (Serial2.available()) {
    
    char c = Serial2.read();
    if (c != '}') 
      sendBuff += c;
    
    if (c == '}')  {        
      Serial.print("Received data in serial port from Arduino: ");
      Serial.println(sendBuff);
      sendBuff = "";
    } 
  }
}

void publish_msg() {
  msg_no_time["action"] = "START_LAP";

  //Serializing in order to publish in the topic:
  char out[128];
  serializeJson(msg_no_time, out);
  pub.publish(out);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  initWiFi();
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  //Initializing first values of the JSON msgs:
  //When it has time:
  msg_no_time["team_name"] = "Robotitos";
  msg_time["team_name"] = "Robotitos";
  msg_no_time["id"] = "9";
  msg_time["id"] = "9";
}

uint32_t x=0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  publish_msg();
  recv_serial_msg();
  
  // wait a couple seconds to avoid rate limit
  delay(2000);

}
