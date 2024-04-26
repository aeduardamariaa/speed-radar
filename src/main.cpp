#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <nlohmann/json.hpp>
#include "dotenv.h"

#include <string>
#include <iostream>

using namespace std;
using json = nlohmann::json;

// hardware constants
const int SERIAL_SPEED = 115200;
const uint8_t TRIG_PINS[2] = {5, 26};
const uint8_t ECHO_PINS[2] = {18, 25};

// physical constants
const float SOUND_SPEED = 0.034;
const float CM_TO_INCH = 0.393701;
const float MT_S_TO_KM_H = 3.6;

// project constants
const string WIFI_NAME = "Andrép";
const string WIFI_PW = "naotemsenha";
const string SERVER_HOST = "192.168.170.236";
const string SERVER_PORT = "3000";
const float DISTANCE_THRESHOLD_CM = 25.0;
const float DISTANCE_BETWEEN_M = 0.3;
const float MAX_SPEED_KM_H = 4.0;

// project objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
HTTPClient http;

// global variables
bool firstActivated = false;
bool secondActivated = false;
int globalTimeTracking;

float getDistanceCm(uint8_t trigPin, uint8_t echoPin)
{
  long timeTracking = micros();

  digitalWrite(trigPin, LOW);
  while (micros() - timeTracking <= 2);

  digitalWrite(trigPin, HIGH);
  while (micros() - timeTracking <= 12);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distanceCm = duration * SOUND_SPEED / 2;

  return distanceCm;
}

double getAverageSpeedKmHour(int timePassedMillis)
{
  double avgSpeedMetersSecond = DISTANCE_BETWEEN_M / (timePassedMillis / 1000.0);

  return (avgSpeedMetersSecond * MT_S_TO_KM_H);
}

bool isPastOver()
{
  float distanceCm = getDistanceCm(TRIG_PINS[1], ECHO_PINS[1]);
  bool isPassing = (distanceCm <= DISTANCE_THRESHOLD_CM);

  if (isPassing && !secondActivated)
    secondActivated = true;

  return !isPassing && secondActivated;
}

bool postSpeed(float speedKmH)
{
  const char *serverHost = SERVER_HOST.c_str();
  const char *serverPort = SERVER_PORT.c_str();

  string url;
  if (serverHost && serverPort)
    url = "http://" + string(serverHost) + ":" + string(serverPort) + "/violation";
  else
    Serial.println("Error: SERVER_HOST or SERVER_PORT environment variables not set.");

  http.begin(String(url.c_str()));
  http.addHeader("Content-type", "application/json");

  json requestBody;
  requestBody["speedKmH"] = speedKmH;

  string requestBodyString = requestBody.dump();
  int statusCode = http.POST(String(requestBodyString.c_str()));

  bool hasSucceeded = false;
  if (statusCode >= 200 && statusCode < 300)
    hasSucceeded = true;
  else
    Serial.println("POST request failed.");

  http.end();

  return hasSucceeded;
}

void setup()
{
  Serial.begin(SERIAL_SPEED);

  WiFi.begin(WIFI_NAME.c_str(), WIFI_PW.c_str());
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi network!");

  lcd.begin();
  lcd.backlight();

  for (uint8_t trigPin : TRIG_PINS)
    pinMode(trigPin, OUTPUT);

  for (uint8_t echoPin : ECHO_PINS)
    pinMode(echoPin, INPUT);
}

void loop()
{
  if (!firstActivated)
  {
    float distanceCm = getDistanceCm(TRIG_PINS[0], ECHO_PINS[0]);

    if (distanceCm <= DISTANCE_THRESHOLD_CM)
    {
      firstActivated = true;
      globalTimeTracking = millis();
    }
  }
  else
  {
    while (!isPastOver());

    int timePassedMillis = millis() - globalTimeTracking;
    double avgSpeed = getAverageSpeedKmHour(timePassedMillis);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("%.2f Km/h", avgSpeed);

    if (avgSpeed > MAX_SPEED_KM_H)
      postSpeed(avgSpeed);

    firstActivated = false;
    secondActivated = false;

    delay(5);
  }
}