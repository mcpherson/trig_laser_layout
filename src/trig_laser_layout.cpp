/* 
 * Project: Trig Laser Layout
 * Author: Marlon McPherson
 * Date: 1 APR 2025
 */

#include "Particle.h"
#include "Stepper.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_VL53L1X.h"

// same as const int?
#define IRQ_PIN 2
#define XSHUT_PIN 3

SYSTEM_MODE(MANUAL);
// SYSTEM_THREAD(ENABLED);

const int OLED_RESET=-1;
const int LASERPIN = D14;
// X Stepper
const int X_ST_1 = D3, X_ST_2 = D4, X_ST_3 = D5, X_ST_4 = D6;
// Y Stepper
const int Y_ST_1 = D10, Y_ST_2 = D11, Y_ST_3 = D12, Y_ST_4 = D13;

bool laserToggle, measureToggle;
// Stepper
int SPR = 2048; // steps per revolution
int speed = 12; // RPM
int stepsX, stepsY;
// Calcs
uint8_t lastTime; 
int16_t normalX[3], normalY[3]; 

enum Mode {
  FREE_GRID,
  CENTERED_GRID,
  STUDS,
  EDGE_PROFILE,
  DETECT_MOLDING,
  ROOM_PROFILE
};

SerialLogHandler logHandler(LOG_LEVEL_INFO);
Adafruit_SSD1306 display(OLED_RESET);
// xStepper rotates around Z, yStepper rotates around X
// rotation angle, distance, and normal vector are all that's needed
Stepper xStepper(SPR, X_ST_1, X_ST_3, X_ST_2, X_ST_4);
Stepper yStepper(SPR, Y_ST_1, Y_ST_3, Y_ST_2, Y_ST_4);
Adafruit_VL53L1X distanceSensor = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void toggleLaser();
int calcSteps();
void showPoint(int16_t point[3]);
void normalize();
void orient();
void setMode(Mode mode);
void displayInstructions(Mode mode, uint8_t line);


void setup() {
  // SERIAL MONITOR
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  // PINS
  pinMode(LASERPIN, OUTPUT);

  // OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.fillRect(0,0,128,64,BLACK);
  display.drawLine(64,0,64,64,WHITE);
  display.drawLine(0,32,128,32,WHITE);
  display.display(); 

  // STEPPERS
  xStepper.setSpeed(speed);
  yStepper.setSpeed(speed);

  // TIME OF FLIGHT SENSOR (TOFS)
  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  // MPU6050?

  // TIMERS
  lastTime = millis();
}

void loop() {
  if (millis() - lastTime > 2000) {
    
    lastTime = millis();
  }
}

void toggleLaser() {
  if (!laserToggle) {
    laserToggle = true;
    digitalWrite(LASERPIN, HIGH);
  }
  else {
    laserToggle = false;
    digitalWrite(LASERPIN, LOW);
  }
}

void showPoint(int16_t point[3]) {

}

void normalize() {
  // x+ steps to min
  while (normalX == 0) {
    xStepper.step(stepsX);

  }
}
void orient();
void setMode(Mode mode);
void displayInstructions(Mode mode, uint8_t line);