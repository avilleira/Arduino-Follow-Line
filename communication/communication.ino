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

#define WLAN_SSID "sensoresurjc"  

#define WLAN_PASS   "Goox0sie_WZCGGh25680000"
//"Goox0sie_WZCGGh25680000"
/************************* Adafruit.io Setup *********************************/

#define SERVER      "193.147.53.2"
#define SERVERPORT  21883                   // use 8883 for SSL
#define USERNAME    "Robotitos"
#define KEY         "9"

/************ Global State (you don't need to change this!) ******************/

/***************** Serial2 variables******************************************/
#define RXD2 33
#define TXD2 4

/********************** Global Variables *************************************/

unsigned long ping_counter;
unsigned long lap_timer;

bool stop = false;
bool lap_started = false;

int prev_state;

// Enumeration of all the possible actions:
enum actions {
  START = 0,
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

void initWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
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
}

int recv_serial_msg() {
  uint8_t c;
  int action;
  String sendBuff;
  if (Serial2.available() > 0) {
    c = Serial2.read();
    sendBuff += (char)c;     
    action = sendBuff.toInt();
    return action;
  }
  else
    return -1;
}

void publish_msg(int action) {

  StaticJsonDocument<256> msg;
  char out[128];
  Serial.println(action);
  if ((action < 9) && (action > 0)) {
    msg["team_name"] = "Robotitos";
    msg["id"] = "9";
    if (action == FINISH){
      msg["action"] = "END_LAP";
      msg["time"] = millis() - lap_timer;
      stop = true;
    }
    else if (action == OBSTACLE)
      msg["action"] = "OBSTACLE_DETECTED";
    else if (action == LINE_LOST)
      msg["action"] = "LINE_LOST";
    else if (action == LINE_FOUND)
      msg["action"] = "LINE_FOUND";
    else if (action == SEARCHING_LINE)
      msg["action"] = "INIT_LINE_SEARCH";
    else if (action == STOP_SEARCHING)
      msg["action"] = "STOP_LINE_SEARCH";
    else if (action == PING) {
      msg["action"] = "PING";
      msg["time"] = millis() - lap_timer;
    }


    //Serializing in order to publish in the topic:
    if (action != -1) {
      serializeJson(msg, out);
      pub.publish(out);
    }
  }
}

void setup() {
  char out[128];
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

  // publishing start lap:
  MQTT_connect();
  StaticJsonDocument<256> start_msg;
  start_msg["team_name"] = "Robotitos";
  start_msg["id"] = "9";
  start_msg["action"] = "START_LAP";
  Serial2.println("A");
  serializeJson(start_msg, out);
  pub.publish(out);
  // Initializing lap_counter:
  lap_timer = millis();
  // Initializing ping_counter:
  ping_counter = millis();
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  while (!stop) {
    //updating ping_counter:
    if ((millis() - ping_counter) > 3980) {
      publish_msg(PING);
      ping_counter = millis();
    }
    //Publishing the corresponding message:
    publish_msg(recv_serial_msg());
    
  }
}