/* 
 * Project: Trig Laser Layout
 * Author: Marlon McPherson
 * Date: 1 APR 2025
 */

#include "Particle.h"
#include "math.h"
#include "Stepper.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_VL53L1X.h"
#include <Wire.h>

#define IRQ_PIN 2
#define XSHUT_PIN 3
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

SYSTEM_MODE(MANUAL);
// SYSTEM_THREAD(ENABLED);

const int OLED_RESET=-1;
const int LASERPIN = D19;
// X Stepper
const int X_ST_1 = D18;
const int X_ST_2 = D17;
const int X_ST_3 = D15; // also out of order on the Photon 2
const int X_ST_4 = D16;
// Y Stepper
const int Y_ST_1 = D11;
const int Y_ST_2 = D12;
const int Y_ST_3 = D13;
const int Y_ST_4 = D14;

bool laserToggle, measureToggle;
bool normalized, normalizedX, normalizedY;
bool oriented, orientedX, orientedY;
// Stepper
int SPR = 2048; // steps per revolution
float SPD = SPR / 360.0; // steps Per Degree
int speed = 15; // RPM
int stepsX, stepsY;
// Calcs
uint32_t lastTime; 
int16_t normalX[3], normalY[3]; 

int status;

const int numMeas = 5;
int measCount = 0;
int measurements[numMeas];
int thisDistance;

// MPU6050
byte accel_x_h, accel_x_l, accel_y_h, accel_y_l, accel_z_h, accel_z_l;
int16_t accel_x, accel_y, accel_z;
float x_Gs, y_Gs, z_Gs;
byte AFS_SEL_value;
const int AFS_SEL_factor[4] = {16384, 8192, 4096, 2048};
float pitch, roll;
// USE ATAN2()
// find pitch and roll angles with orient();

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
// Adafruit_VL53L1X distanceSensor = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void toggleLaser();
int calcSteps();
void showPoint(int16_t point[3]);
float getMeasurement();
void getAccel();
void getPitchAndRoll(float *x, float *y, float *z);
void orient();
void normalize();
void setMode(Mode mode);
void displayInstructions(Mode mode, uint8_t line);

void stepTo(float xSteps, float ySteps);
void stepperTest();


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
  
  // MPU6050
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); 
  Wire.endTransmission(true);
  AFS_SEL_value = 0;

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

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  // GET important settings:
  // Serial.print(F("Signal Per SPAD: "));
  // Serial.println(vl53.VL53L1X_GetSignalPerSpad());
  // Serial.print(F("Offset: "));
  // Serial.println(vl53.VL53L1X_GetOffset());
  // Serial.print(F("ROI X/Y: "));
  // Serial.println(vl53.VL53L1X_GetROI_XY());
  // Serial.print(F("ROI Center: "));
  // Serial.println(vl53.VL53L1X_GetROICenter());
  // Serial.print(F("Intermeasurement (ms): "));
  // Serial.println(vl53.VL53L1X_GetInterMeasurementInMs());

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */

  // TIMERS
  lastTime = millis();
}

void loop() {



  if (!oriented) {
    getAccel();
    getPitchAndRoll(&x_Gs, &y_Gs, &z_Gs);
    orient();
  }
  
  if (!normalized) {
    normalize();
  }
  
  Serial.println(getMeasurement());
  
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

float getMeasurement() {
  float avgDist;
  while (measCount < numMeas) {
    int16_t distance;
    if (vl53.dataReady()) {
      distance = vl53.distance();
      if (distance == -1) { // error
        Serial.print(F("Couldn't get distance: "));
        Serial.println(vl53.vl_status);
        return -1;
      }
  
      measurements[measCount] = distance;
      measCount++;
      if (measCount == numMeas) {
        int measSum;
        for (int i=0; i<numMeas; i++) {
          measSum += measurements[i];
        }
        avgDist = (float)measSum / numMeas;
        Serial.printf("SUM: %i, AVG: %0.0f\n", measSum, avgDist);
      }
      
      vl53.clearInterrupt();
    }
  }
  measCount = 0;
  return avgDist;
}

void getAccel() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  // get the values
  Wire.requestFrom(0x68, 6, true);
  // combine accel - shift h bits left by 8, then OR with the l bits to combine
  accel_x = (Wire.read() << 8 | Wire.read());
  accel_y = (Wire.read() << 8 | Wire.read());
  accel_z = (Wire.read() << 8 | Wire.read());
  // calculate Gs with accel values and the AFS_SEL factor
  x_Gs = (float)accel_x / AFS_SEL_factor[AFS_SEL_value];
  y_Gs = (float)accel_y / AFS_SEL_factor[AFS_SEL_value];
  z_Gs = (float)accel_z / AFS_SEL_factor[AFS_SEL_value];
}

void getPitchAndRoll(float *x, float *y, float *z) {
  // calculates pitch in degrees
  // NaN is returned if pitch > 90 or pitch < -90
  // this is intended behavior. 
  pitch = -asin(*x) * (180.0/M_PI);
  Serial.printf("PITCH: %0.2f\n", pitch);

  // calculates roll in degrees
  // flips from 180 to -180 halfway through roll
  // this is ALSO intended behavior.
  roll = atan2(*y, *z) * (180.0/M_PI);
  Serial.printf("ROLL:  %0.2f\n", roll);
}

void orient() {

}

void normalize() {
  float xCurr, yCurr, xLast, yLast, xCurrMin, yCurrMin;
  int sweepSteps = 50;
  int sweepDecr = 5;
  bool stepsDirection = true;
  // take first X measurement
  xCurr = getMeasurement();
  // start X normalization sweeps
  while (!normalizedX) {
    xStepper.step(sweepSteps);
    xLast = xCurr;
    xCurr = getMeasurement();
    if (xCurr > xLast) {
      xCurrMin = xLast;
      // reverse direction and decrease step increment
      if (stepsDirection) {
        stepsDirection = false;
        sweepSteps -= ((2 * sweepSteps) + sweepDecr);
      }
      else {
        stepsDirection = true;
        sweepSteps += ((2 * sweepSteps) - sweepDecr);
      }
      // check for 0 steps increment (normalizeX finished)
      if (sweepSteps == 0) {
        normalX = xCurrMin;
        normalizedX = true;
      }
    }
    else {
      xCurrMin = xCurr;
    }
  }

  // reset vars for Y normalization
  sweepSteps = 50;
  stepsDirection = true;
  // take first Y measurement
  yCurr = getMeasurement();
  // start Y normalization sweeps
  while (!normalizedY) {
    yStepper.step(sweepSteps);
    yLast = yCurr;
    yCurr = getMeasurement();
    if (yCurr > yLast) {
      yCurrMin = yLast;
      // reverse direction and decrease step increment
      if (stepsDirection) {
        stepsDirection = false;
        sweepSteps -= ((2 * sweepSteps) + sweepDecr);
      }
      else {
        stepsDirection = true;
        sweepSteps += ((2 * sweepSteps) - sweepDecr);
      }
      // check for 0 steps increment (normalizeX finished)
      if (sweepSteps == 0) {
        normalY = yCurrMin;
        normalizedY = true;
      }
    }
    else {
      yCurrMin = yCurr;
    }
  }
}

void setMode(Mode mode);
void displayInstructions(Mode mode, uint8_t line);

void stepperTest() {

  for (int i = 0; i < 180; i++) {

      xStepper.step(1);
    
      yStepper.step(-1);

  }
}

void stepTo(float xDeg, float yDeg) {
  // Store mantissa for rounding correction?
  int xSteps = round(xDeg * SPD);
  int ySteps = round(yDeg * SPD);
  // TRACK TOTAL STEPS GLOBALLY
  
  int xStepsAbs = abs(xSteps);
  int yStepsAbs = abs(ySteps);
  int max;
  int currSteps = 0;

  int xDir;
  int yDir;
  if (xSteps >= 0) { xDir = 1; }
  else            { xDir = -1; }
  // yStepper's polarity is reversed for some reason...
  if (ySteps >= 0) { yDir = -1; }
  else            { yDir = 1; }

  if (xStepsAbs > yStepsAbs || xStepsAbs == yStepsAbs) {
    max = xStepsAbs;
  }
  else {
    max = yStepsAbs;
  }
  Serial.printf("xSteps: %i, ySteps: %i, max: %i, xDir: %i, yDir: %i\n", xSteps, ySteps, max, xDir, yDir);
  for (int i = 0; i < max; i++) {
    if (currSteps < xStepsAbs) {
      // round here instead to minimize error accum?
      xStepper.step(xDir);
    }
    if (currSteps < yStepsAbs) {
      yStepper.step(yDir);
    }
    currSteps++;
  }

}