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
#include "Button.h"
#include <Wire.h>

#define IRQ_PIN 2
#define XSHUT_PIN 3

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// PINS
const int OLED_RESET=-1;
const int LASERPIN = D19;
const int JOYSTICK_BUTTON = D10; // also use as wakeup?
const int JOYSTICK_X = A0;
const int JOYSTICK_Y = A1;
// X Stepper
const int X_ST_1 = D18;
const int X_ST_2 = D17;
const int X_ST_3 = D15;
const int X_ST_4 = D16;
// Y Stepper
const int Y_ST_1 = D7;
const int Y_ST_2 = D6;
const int Y_ST_3 = D5;
const int Y_ST_4 = D4;

// STATES
bool laserToggle, measureToggle;
bool isDriving;
bool normalized, normalizedX, normalizedY;
bool oriented, orientedX, orientedY;

// WALL DIMENSIONS
// TL = top left, BR = bottom right
int16_t TLsteps[2], TLdistance, BRsteps[2], BRdistance;
float normalX, normalY; 


// STEPPER
int SPR = 2048; // steps per revolution
float SPD = SPR / 360.0; // steps Per Degree
int speed = 15; // RPM
int totalStepsX, totalStepsY;
int stepperLimitX = 400; // ~70deg
int stepperLimitY = 285; // ~50deg

// ToF
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

// CONTROLS
uint16_t sense[6] = {500, 1250, 1900, 2200, 2850, 3600}; // sensitivity thresholds 

// TIMING
uint32_t lastTime; 

enum Mode {
  MANUAL_DRIVE,
  FREE_GRID,
  CENTERED_GRID,
  STUDS,
  EDGE_PROFILE,
  DETECT_MOLDING,
  ROOM_PROFILE
};

// DEVICES
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
Adafruit_SSD1306 display(OLED_RESET);
Button joystickButton(JOYSTICK_BUTTON);
// xStepper rotates around Z, yStepper rotates around X
Stepper xStepper(SPR, X_ST_1, X_ST_3, X_ST_2, X_ST_4);
Stepper yStepper(SPR, Y_ST_1, Y_ST_3, Y_ST_2, Y_ST_4);

void toggleLaser();
int calcSteps();
void showPoint(int16_t point[3]);
float getMeasurement();
void getAccel();
void getPitchAndRoll(float *x, float *y, float *z);
void orient();
void normalize();
void beginManualDrive();
void manualDrive();
void drive();
void setMode(Mode mode);
void displayInstructions(Mode mode, uint8_t line);
void stepTo(float xSteps, float ySteps, bool stepwise);


void setup() {
  // SERIAL MONITOR
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  // PINS / INTERRUPTS
  pinMode(LASERPIN, OUTPUT);
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
  attachInterrupt(JOYSTICK_BUTTON, beginManualDrive, FALLING);

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

  // LASER
  toggleLaser();

}


void loop() {

  // stepTo(-10.0, 10.0);
  // delay(50000);        
  
  if (isDriving) {
    manualDrive();
    // reset stepper speed to max when finished
    xStepper.setSpeed(speed);
    yStepper.setSpeed(speed);
  }
  

  // if (!oriented) {
  //   getAccel();
  //   getPitchAndRoll(&x_Gs, &y_Gs, &z_Gs);
  //   stepTo(0.0, -15.0);
  //   delay(3000);
  //   getAccel();
  //   getPitchAndRoll(&x_Gs, &y_Gs, &z_Gs);
  //   stepTo(0.0, 15.0);
  //   delay(3000);
  //   orient();
  // }
  
  // if (!normalized) {
  //   normalize();
  // }
  
  // Serial.println(getMeasurement());

  
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
  // COMPASS? HALL EFFECT?

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

void beginManualDrive() { isDriving = true; }

void manualDrive() {
  detachInterrupt(JOYSTICK_BUTTON);
  Serial.printf("I drive.\n");
  displayInstructions(MANUAL_DRIVE, 0);

  // decrease stepper speed for increased accuracy
  xStepper.setSpeed(2);
  yStepper.setSpeed(2);

  // track when x or y steps are taken to skip subsequent sensitivity checks
  bool xStepped, yStepped = false;

  // using this button is dependent on it being available if the same pin's interrupt triggered this function.
  while (!joystickButton.isClicked()) {

    drive();
  // while (true) {
    int xStick = analogRead(JOYSTICK_X);
    int yStick = analogRead(JOYSTICK_Y);

    // DEBUG
    // Serial.printf("X: %i, Y: %i\n", xStick, yStick);
    // delay(200);

    // confine x/y movement to the correct sensitivity range
    xStepped = false;
    yStepped = false;
    // determine direction and magnitude
    float xMag = 0.0; 
    float yMag = 0.0;
    // X
    if (xStick > sense[5]) {              // +X max 
      xMag = 3.0;
      xStepped = true;
    }
    if (xStick > sense[4] && !xStepped) { // +X mid
      xMag = 2.0;
      xStepped = true;
    }
    if (xStick > sense[3] && !xStepped) { // +X min
      xMag = 1.0;
    }
    if (xStick < sense[0]) {              // -X max
      xMag = -3.0;
      xStepped = true;
    }
    if (xStick < sense[1] && !xStepped) { // -X mid
      xMag = -2.0;
      xStepped = true;
    }
    if (xStick < sense[2] && !xStepped) { // -X min
      xMag = -1.0;
    }
    // Y 
    if (yStick > sense[5]) {              // +Y max 
      yMag = 3.0;
      yStepped = true;
    }
    if (yStick > sense[4] && !yStepped) { // +Y mid
      yMag = 2.0;
      yStepped = true;
    }
    if (yStick > sense[3] && !yStepped) { // +Y min
      yMag = 1.0;
    }
    if (yStick < sense[0]) {              // -Y max
      yMag = -3.0;
      yStepped = true;
    }
    if (yStick < sense[1] && !yStepped) { // -Y mid
      yMag = -2.0;
      xStepped = true;
    }
    if (yStick < sense[2] && !yStepped) { // -Y min
      yMag = -1.0;
    }

    // Drive
    if (xMag != 0.0 || yMag != 0.0) {
      stepTo(xMag, yMag, true);
    }
  
  }
  
  // short delay to stabilize before measuring top left corner distance and getting total steps for angle calcs
  delay(200);
  TLdistance = getMeasurement();
  TLsteps[0] = totalStepsX;
  TLsteps[1] = totalStepsY;


  isDriving = false;
  attachInterrupt(JOYSTICK_BUTTON, beginManualDrive, FALLING);
}

void drive() {

  // track when x or y steps are taken to skip subsequent sensitivity checks
  bool xStepped, yStepped = false;

  int xStick = analogRead(JOYSTICK_X);
  int yStick = analogRead(JOYSTICK_Y);

  // DEBUG
  // Serial.printf("X: %i, Y: %i\n", xStick, yStick);
  // delay(200);

  // confine x/y movement to the correct sensitivity range
  xStepped = false;
  yStepped = false;
  // determine direction and magnitude
  float xMag = 0.0; 
  float yMag = 0.0;
  // X
  if (xStick > sense[5]) {              // +X max 
    xMag = 3.0;
    xStepped = true;
  }
  if (xStick > sense[4] && !xStepped) { // +X mid
    xMag = 2.0;
    xStepped = true;
  }
  if (xStick > sense[3] && !xStepped) { // +X min
    xMag = 1.0;
  }
  if (xStick < sense[0]) {              // -X max
    xMag = -3.0;
    xStepped = true;
  }
  if (xStick < sense[1] && !xStepped) { // -X mid
    xMag = -2.0;
    xStepped = true;
  }
  if (xStick < sense[2] && !xStepped) { // -X min
    xMag = -1.0;
  }
  // Y 
  if (yStick > sense[5]) {              // +Y max 
    yMag = 3.0;
    yStepped = true;
  }
  if (yStick > sense[4] && !yStepped) { // +Y mid
    yMag = 2.0;
    yStepped = true;
  }
  if (yStick > sense[3] && !yStepped) { // +Y min
    yMag = 1.0;
  }
  if (yStick < sense[0]) {              // -Y max
    yMag = -3.0;
    yStepped = true;
  }
  if (yStick < sense[1] && !yStepped) { // -Y mid
    yMag = -2.0;
    xStepped = true;
  }
  if (yStick < sense[2] && !yStepped) { // -Y min
    yMag = -1.0;
  }

  // Drive
  if (xMag != 0.0 || yMag != 0.0) {
    stepTo(xMag, yMag, true); // stepwise
  }
}

void setMode(Mode mode);

void displayInstructions(Mode mode, uint8_t line) {
  display.clearDisplay();
  switch (mode) {
    case MANUAL_DRIVE:
      switch (line) {
        case 0:
          display.printf("MANUAL MODE:\nDrive to the\ntop left corner.\nClick to continue.");
          break;
        case 1: 
          display.printf("MANUAL MODE:\nDrive to the\nbottom right corner.\nClick to finish.");
          break;
        default:
          display.printf("MANUAL MODE:\nERROR");
          break;
      }
      break;
    case FREE_GRID:
    default:
      break;
  }
  display.display();
}


void stepTo(float xDeg, float yDeg, bool stepwise) {
  int xSteps, ySteps;
  if (!stepwise) { // convert degrees to steps
    xSteps = round(xDeg * SPD);
    ySteps = round(yDeg * SPD);
  }
  else {           // use degrees as steps
    xSteps = (int)xDeg;
    ySteps = (int)yDeg;
  }

  int xStepsAbs = abs(xSteps);
  int yStepsAbs = abs(ySteps);

  // Prevent movement outside predefined range (assuming I can roughly center the device on startup using MPU for pitch and compass or Hall sensor for yaw)
  // if (abs(totalStepsX) + xStepsAbs >= stepperLimitX) {
  //   Serial.printf("Attempted X overstep from %i to %i.\n", totalStepsX, totalStepsX + xSteps);
  //   xSteps = 0;
  // }
  // if (abs(totalStepsY) + yStepsAbs >= stepperLimitY) {
  //   Serial.printf("Attempted Y overstep from %i to %i.\n", totalStepsY, totalStepsY + ySteps);
  //   ySteps = 0;
  // }
  
  // find direction - positive angle = clockwise (looking at the motor shaft)
  int xDir;
  int yDir;
  if (xSteps >= 0) { xDir = -1; }
  else             { xDir = 1; }
  if (ySteps >= 0) { yDir = -1; }
  else             { yDir = 1; }
  
  // find maximum to bound loop
  int max;
  (xStepsAbs > yStepsAbs || xStepsAbs == yStepsAbs) ? max = xStepsAbs : max = yStepsAbs;
  
  // track total steps globally
  totalStepsX += xSteps;
  totalStepsY += ySteps;

  // step 1 degree at a time until both angles have been reached
  for (int i = 0; i < max; i++) {
    if (i < xStepsAbs) { xStepper.step(xDir); }
    if (i < yStepsAbs) { yStepper.step(yDir); }
  }
  
  
  // DEBUG
  Serial.printf("Total steps X: %i, Total steps Y: %i\n", totalStepsX, totalStepsY);
  // Serial.printf("xSteps: %i, ySteps: %i, max: %i, xDir: %i, yDir: %i\n", xSteps, ySteps, max, xDir, yDir);
}