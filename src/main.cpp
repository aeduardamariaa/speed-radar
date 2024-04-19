#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// hardware constants
#define SERIAL_SPEED 115200
const uint8_t TRIG_PINS[2] = {5, 26};
const uint8_t ECHO_PINS[2] = {18, 25};

// physical constants
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
#define MT_S_TO_KM_H 3.6

// project constants
#define DISTANCE_THRESHOLD_CM 25.0
#define DISTANCE_BETWEEN_M 0.3

LiquidCrystal_I2C lcd(0x27, 16, 2);

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

double getAverageSpeedKmHour(int timePassedMillis) {
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

void setup()
{
  Serial.begin(SERIAL_SPEED);
  lcd.begin();

  lcd.backlight();

  for (uint8_t trigPin : TRIG_PINS)
  {
    pinMode(trigPin, OUTPUT);
  }

  for (uint8_t echoPin : ECHO_PINS)
  {
    pinMode(echoPin, INPUT);
  }
}

void loop()
{
  if (!firstActivated)
  {
    float distanceCm = getDistanceCm(TRIG_PINS[0], ECHO_PINS[0]);

    if (distanceCm <= 25.0)
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

    lcd.setCursor(0, 0);
    lcd.printf("%.4f", avgSpeed);

    firstActivated = false;
    secondActivated = false;

    delay(5);
  }
}