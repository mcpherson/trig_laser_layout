/* 
 * Project: Trig Laser Layout
 * Author: Marlon McPherson
 * Date: 1 APR 2025
 */

#include "Particle.h"
#include <Wire.h>
#include "math.h"
#include "Stepper.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_VL53L1X.h"
#include "Button.h"

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// PINS
const int OLED_RESET=-1;
const int LASERPIN = D19;
const int BUTTONPIN = D3;
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
// TOF
const int IRQ_PIN = D13;
const int XSHUT_PIN = D14;

// STATES
bool laserToggle = false;
bool measureToggle = false;
bool isDriving = false;
bool normalized = false;

// WALL DIMENSIONS
// N = normal vector, BL/TL/TR = bottom left/top left/top right corners (defined via manualDrive())
// typedef struct {
//   int x,
//   int y
// } Steps;

int16_t Ndistance;
int16_t BLdistance, TLdistance, TRdistance;

int16_t BLsteps[2];
int16_t TLsteps[2];  
int16_t TRsteps[2]; 

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

// CONTROLS
uint16_t sensi[6] = {500, 1250, 1900, 2200, 2850, 3600}; // sensitivity thresholds 

// TIMING
uint32_t lastTime; 

// MODES
enum Mode {
  NORMALIZE,
  DEFAULT_MODE,
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
// Button nextButton(BUTTONPIN);
// xStepper rotates around Z, yStepper rotates around X
Stepper xStepper(SPR, X_ST_1, X_ST_3, X_ST_2, X_ST_4);
Stepper yStepper(SPR, Y_ST_1, Y_ST_3, Y_ST_2, Y_ST_4);

void toggleLaser();
int getMeasurement();
void normalize();
void beginManualDrive();
void manualDrive();
void drive();
void setMode(Mode mode);
void displayInstructions(Mode mode, uint8_t line);
void stepTo(float xSteps, float ySteps, bool stepwise);
void calculateWall();
void showPoint();


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

  // TIME OF FLIGHT SENSOR (TOFS)
  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error initializing VL53L1X: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X initialized"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms
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
  // uint16_t ROIC;
  // Serial.print(F("ROI Center: "));
  vl53.VL53L1X_SetDistanceMode(1); // short = 1, long = 2
  // Serial.println(ROIC);
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
  
  if (isDriving) { manualDrive(); }

  if (!normalized) { normalize(); } 
  

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


int getMeasurement() {
  float avgDist;
  loopStart:
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
        // DEBUG
        // Serial.printf("SUM: %i, AVG: %0.0f\n", measSum, avgDist);
      }
      
      vl53.clearInterrupt();
    }
  }
  measCount = 0;
  if (avgDist > 10000) { goto loopStart; } // sensor sometimes spits out a huge value
  return round(avgDist);
}


void normalize() {
  xStepper.setSpeed(2);
  yStepper.setSpeed(2);
  Serial.printf("NORMALIZING...\n");
  displayInstructions(NORMALIZE, 0);
  int xCurr, yCurr, xLast, yLast;
  float startSweepSteps = -250.0;
  float sweepSteps = 16.0;
  bool prevIncreased = false;
  int timesStepped = 0;
  int slopSteps = 11;
  bool backlashed = false;

  // X normalization
  stepTo(startSweepSteps, 0.0, true);
  delay(250);
  xLast = 10000; // spoof impossible value to pass the IF
  getMeasurement(); // VL53L1X often returns a junk value after waking up, so I purge it by taking a few measurements
  getMeasurement();
  while (true) {
    stepTo(sweepSteps, 0.0, true);
    delay(300);
    if (timesStepped > slopSteps) { // don't start measuring until backlash is taken up
      xCurr = getMeasurement();
      Serial.printf("X - CURR: %i,  LAST: %i,  ALREADY: %i\n", xCurr, xLast, prevIncreased);
      if (xCurr >= xLast) {
        if (prevIncreased) {
          break;
        }
        else {
          prevIncreased = true;
        }
      }
      else {
        prevIncreased = false;
      }
    }
    timesStepped++;
    if (timesStepped > slopSteps && !backlashed) {
      backlashed = true;
      sweepSteps = 7;
    }
    xLast = xCurr;
  }

  stepTo(-sweepSteps, 0.0, true); // step to previous X value since that's the actual normal
  prevIncreased = false;
  sweepSteps = 15.0;
  timesStepped = 0;
  backlashed = false;
  // Y normalization
  stepTo(0.0, startSweepSteps, true);
  delay(250);
  yLast = 10000;
  while (true) {
    stepTo(0.0, sweepSteps, true);
    delay(300);
    if (timesStepped > slopSteps) {
      yCurr = getMeasurement();
      Serial.printf("Y - CURR: %i,  LAST: %i,  ALREADY: %i\n", yCurr, yLast, prevIncreased);
      if (yCurr >= yLast) {
        if (prevIncreased) {
          break;
        }
        else {
          if (timesStepped > slopSteps) {
            prevIncreased = true;
          }
        }
      }
      else {
        prevIncreased = false;
      }
    }
    timesStepped++;
    if (timesStepped > slopSteps && !backlashed) {
      backlashed = true;
      sweepSteps = 7;
    }
    yLast = yCurr;
  }

  Ndistance = yLast;
  normalized = true;
  stepTo(0.0, -sweepSteps, true); // step to previous Y value before setting origin
  totalStepsX = 0;
  totalStepsY = 0;
  Serial.printf("NORMALIZED!\n");
  displayInstructions(NORMALIZE, 1);
  xStepper.setSpeed(speed);
  yStepper.setSpeed(speed);
}


void beginManualDrive() { isDriving = true; } // interrupt

// Allows the user to drive the gimbal. 
// Easiest and most accurate way to find the true corners of a wall with budget parts.
void manualDrive() {
  Serial.printf("I drive.\n");
  // change joystick click functionality
  detachInterrupt(JOYSTICK_BUTTON);
  Button nextButton(JOYSTICK_BUTTON, true);
  
  // decrease stepper speed for increased accuracy
  xStepper.setSpeed(1);
  yStepper.setSpeed(1);
  
  // Bottom left corner
  displayInstructions(MANUAL_DRIVE, 0);
  delay(100); // prevent clickthrough
  while (!nextButton.isClicked()) { drive(); }
  delay(100); // prevent clickthrough
  BLsteps[0] = totalStepsX;
  BLsteps[1] = totalStepsY;
  
  // Top left corner
  displayInstructions(MANUAL_DRIVE, 1);
  delay(100); // prevent clickthrough
  while (!nextButton.isClicked()) { drive(); } 
  delay(100); // prevent clickthrough
  TLsteps[0] = totalStepsX;
  TLsteps[1] = totalStepsY;

  // Top right corner
  displayInstructions(MANUAL_DRIVE, 2);
  delay(100); // prevent clickthrough
  while (!nextButton.isClicked()) { drive(); } 
  delay(100); // prevent clickthrough
  TRsteps[0] = totalStepsX;
  TRsteps[1] = totalStepsY;

  // DEBUG
  Serial.printf("BLdist: %i, X: %i, Y: %i\n", BLdistance, BLsteps[0], BLsteps[1]);
  Serial.printf("TLdist: %i, X: %i, Y: %i\n", TLdistance, TLsteps[0], TLsteps[1]);
  Serial.printf("TRdist: %i, X: %i, Y: %i\n", TRdistance, TRsteps[0], TRsteps[1]);
  stepTo(-totalStepsX, -totalStepsY, true);

  attachInterrupt(JOYSTICK_BUTTON, beginManualDrive, FALLING);

  // reset stepper speed to max when finished
  xStepper.setSpeed(speed);
  yStepper.setSpeed(speed);

  isDriving = false;
}


// joystick controls
void drive() {

  // track when x or y steps are taken to skip subsequent sensitivity checks
  bool xStepped = false;
  bool yStepped = false;

  int xStick = analogRead(JOYSTICK_X);
  int yStick = analogRead(JOYSTICK_Y);

  // DEBUG
  // Serial.printf("X: %i, Y: %i\n", xStick, yStick);
  // delay(200);

  // determine direction and magnitude
  // sensitivity doesn't work because of how stepTo() works...
  float xMag = 0.0; 
  float yMag = 0.0;
  // X
  if (xStick > sensi[5]) {              // +X max 
    xMag = 3.0;
    xStepped = true;
  }
  if (xStick > sensi[4] && !xStepped) { // +X mid
    xMag = 2.0;
    xStepped = true;
  }
  if (xStick > sensi[3] && !xStepped) { // +X min
    xMag = 1.0;
  }
  if (xStick < sensi[0]) {              // -X max
    xMag = -3.0;
    xStepped = true;
  }
  if (xStick < sensi[1] && !xStepped) { // -X mid
    xMag = -2.0;
    xStepped = true;
  }
  if (xStick < sensi[2] && !xStepped) { // -X min
    xMag = -1.0;
  }

  // Y 
  if (yStick > sensi[5]) {              // +Y max 
    yMag = -3.0;
    yStepped = true;
  }
  if (yStick > sensi[4] && !yStepped) { // +Y mid
    yMag = -2.0;
    yStepped = true;
  }
  if (yStick > sensi[3] && !yStepped) { // +Y min
    yMag = -1.0;
  }
  if (yStick < sensi[0]) {              // -Y max
    yMag = 3.0;
    yStepped = true;
  }
  if (yStick < sensi[1] && !yStepped) { // -Y mid
    yMag = 2.0;
    xStepped = true;
  }
  if (yStick < sensi[2] && !yStepped) { // -Y min
    yMag = 1.0;
  }

  // Drive
  if (xMag != 0.0 || yMag != 0.0) {
    stepTo(xMag, yMag, true); // stepwise
  }
}


void setMode(Mode mode) {

}


void displayInstructions(Mode mode, uint8_t line) {
  display.clearDisplay();
  display.setCursor(0,0);
  switch (mode) {
    case NORMALIZE:
      switch (line) {
        case 0:
          display.printf("NORMALIZING...\n");
          break;
        case 1:
          display.printf("DONE NORMALIZING.\nClick stick to begin.");
          break;
        default:
          display.printf("NORMALIZING:\nERROR");
          break;
      }
      break;
    case DEFAULT_MODE:
      switch (line) {
        case 0:
          display.printf("Click stick to begin.\n");
          break;
        default:
          break;
      }
      break;
    case MANUAL_DRIVE:
      switch (line) {
        case 0: // bottom left
          display.printf("MANUAL MODE:\nDrive to the exact\n bottom left corner\n(ABOVE BASEBOARD!)\nClick to continue.");
          break;
        case 1: // top left
          display.printf("MANUAL MODE:\nDrive to the exact\ntop left corner\n(BELOW MOLDING!)\nClick to finish.");
          break;
        case 2: // top right
          display.printf("MANUAL MODE:\nDrive to the exact\ntop right corner\n(BELOW MOLDING!)\nClick to finish.");
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
  if (xSteps >= 0) { xDir = -1; }
  else             { xDir = 1; }
  int yDir;
  if (ySteps >= 0) { yDir = -1; }
  else             { yDir = 1; }
  
  // find maximum to bound loop
  int xStepsAbs = abs(xSteps);
  int yStepsAbs = abs(ySteps);
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


void calculateWall() {

}


void showPoint() {

}