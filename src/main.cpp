#include <Arduino.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <ArduinoJson.h>
#include <MqttConnector.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_INA219.h>
Adafruit_INA219 ina219(0x40);

float shuntvoltage, busvoltage, loadvoltage;
double current_mA, current_A, sumCurrent;
float power_mW = 0, power_W = 0;

uint32_t pevTime = 0, pevPrint = 0;
uint32_t powerCount = 0;

double powerSum = 0;
double powerAverage = 0;
double powerHour = 0, powerWattHour, powerKilosWattHour;

#include "init_mqtt.h"
#include "_publish.h"
#include "_receive.h"
#include "_config.h"

MqttConnector *mqtt;


int relayPin = 15;
int relayPinState = HIGH;
int LED_PIN = 2;

char myName[40];

void init_hardware()
{
  Wire.begin(21, 22);
  ina219.begin();
  ina219.setCalibration_16V_400mA();

  pinMode(relayPin, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(relayPin, relayPinState);
  ;
  // serial port initialization
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("Starting...");
}

void init_wifi()
{
  WiFi.disconnect();
  delay(20);
  WiFi.mode(WIFI_STA);
  delay(50);
  const char *ssid = WIFI_SSID.c_str();
  const char *pass = WIFI_PASSWORD.c_str();
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf("Connecting to %s:%s\r\n", ssid, pass);
    delay(300);
  }
  Serial.println("WiFi Connected.");
  digitalWrite(2, HIGH);
}

void setup()
{
  init_hardware();
  init_wifi();
  init_mqtt();
}

void loop()
{
  mqtt->loop();
}
