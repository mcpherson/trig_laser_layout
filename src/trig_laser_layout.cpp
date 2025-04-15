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

// #define X 0
// #define Y 1

// Steps stepperLimit;
// Steps totalSteps;
// Steps BLsteps;
// Steps TLsteps;
// Steps TRsteps;

int16_t Ndistance;
// user-defined points
int16_t BLdistance[2], TLdistance[2], TRdistance[2];

int16_t BLsteps[2];
int16_t TLsteps[2];  
int16_t TRsteps[2]; 

float BLtheta[2], TLtheta[2], TRtheta[2];



// hardcoded for now, will refactor and allocate dynamically when I have time to learn how
int16_t P0steps[2]; // reference on left edge of wall
int16_t P1steps[2]; // first real point
int16_t P2steps[2]; 
int16_t P3steps[2]; 

float P0theta[2], P1theta[2], P2theta[2], P3theta[2];
int16_t P0distance[2], P1distance[2], P2distance[2], P3distance[2];

// vectors
float TLtoBL[2], TLtoTR[2];






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

// TIMING
uint32_t lastTime; 

// MODES
enum Mode {
  NORMALIZE,
  SET_MODE,
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
void setMode();
void freeGridMode();
int8_t stickAdjust();
void displayInstructions(Mode mode, int8_t line);
void stepTo(float xSteps, float ySteps, bool stepwise);
void calculateWall();
void showPoint();
float deg_2_rad(float d);
float rad_2_deg(float r);


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

  if (millis() - lastTime > 10000) {
    if (!normalized) { normalize(); } 
    setMode();
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
  xLast = 10000; // spoof
  getMeasurement(); // VL53L1X often returns a junk value after waking up, so I purge it by taking a couple measurements
  getMeasurement();
  while (true) {
    stepTo(sweepSteps, 0.0, true);
    if (timesStepped > slopSteps) { // don't start measuring until backlash is taken up
      delay(200); // stabilize 
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
      xLast = xCurr;
    }
    timesStepped++;
    if (timesStepped > slopSteps && !backlashed) {
      backlashed = true;
      sweepSteps = 7;
    }
    // xLast = xCurr;
  }

  stepTo(-sweepSteps, 0.0, true); // step to previous X value since that's the actual normal
  prevIncreased = false;
  sweepSteps = 15.0;
  timesStepped = 0;
  backlashed = false;
  // Y normalization
  stepTo(0.0, -startSweepSteps, true);
  yLast = 10000;
  while (true) {
    stepTo(0.0, -sweepSteps, true);
    if (timesStepped > slopSteps) {
      delay(200); // stabilize
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
      yLast = yCurr;
    }
    timesStepped++;
    if (timesStepped > slopSteps && !backlashed) {
      backlashed = true;
      sweepSteps = 7;
    }
    // yLast = yCurr;
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
  xStepper.setSpeed(2);
  yStepper.setSpeed(2);
  
  // Bottom left corner
  displayInstructions(MANUAL_DRIVE, 0);
  delay(100); // prevent clickthrough
  while (!nextButton.isClicked()) { drive(); }
  delay(100); // prevent clickthrough
  BLsteps[0] = totalStepsX;
  BLsteps[1] = totalStepsY;
  BLtheta[0] = (float)BLsteps[0] / SPD;
  BLtheta[1] = (float)BLsteps[1] / SPD;
  d_x = NORMAL_DIST * tan(deg_2_rad(p->theta_x));
  d_y = NORMAL_DIST * tan(deg_2_rad(p->theta_y));
  
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

  // attachInterrupt(JOYSTICK_BUTTON, beginManualDrive, FALLING);

  // reset stepper speed to max when finished
  xStepper.setSpeed(speed);
  yStepper.setSpeed(speed);

  isDriving = false;

  setMode();
}


// joystick controls
void drive() {

  uint16_t sensi[6] = {500, 1250, 1900, 2200, 2850, 3600}; // sensitivity thresholds

  // track when x or y steps are taken to skip subsequent sensitivity checks
  bool xStepped = false;
  bool yStepped = false;

  int xStick = analogRead(JOYSTICK_X);
  int yStick = analogRead(JOYSTICK_Y);

  // DEBUG
  // Serial.printf("X: %i, Y: %i\n", xStick, yStick);
  // delay(200);

  // determine direction and magnitude
  float xMag = 0.0; 
  float yMag = 0.0;

  // X direction
  if (xStick > sensi[3]) {  // +X
    xMag = 1.0;
  }
  if (xStick < sensi[2]) {  // -X
    xMag = -1.0;
  }
  // X speed
  if (xStick > sensi[5] || xStick < sensi[0]) {                // X max 
    xStepper.setSpeed(4);
    xStepped = true;
  }
  if ((xStick > sensi[4] || xStick < sensi[1]) && !xStepped) { // X mid
    xStepper.setSpeed(2);
    xStepped = true;
  }
  if ((xStick > sensi[3] || xStick < sensi[2]) && !xStepped) { // X min
    xStepper.setSpeed(1);
  }

  // Y direction
  if (yStick > sensi[3]) {  // +Y
    yMag = 1.0;
  }
  if (yStick < sensi[2]) {  // -Y
    yMag = -1.0;
  }
  if (yStick > sensi[5] || yStick < sensi[0]) {                // Y max 
    yStepper.setSpeed(4);
    yStepped = true;
  }
  if ((yStick > sensi[4] || yStick < sensi[1]) && !yStepped) { // Y mid
    yStepper.setSpeed(2);
    yStepped = true;
  }
  if ((yStick > sensi[3] || yStick < sensi[2]) && !yStepped) { // Y min
    yStepper.setSpeed(1);
  }

  // Drive
  if (xMag != 0.0 || yMag != 0.0) {
    stepTo(xMag, yMag, true); // stepwise
  }
}


void setMode() {
  Button nextButton(JOYSTICK_BUTTON, true);
  int8_t modeSel = -1;
  int modeMax = 1;
  bool modeChange = false;
  int xThreshPos = 2250;
  int xThreshNeg = 1750;

  while (!nextButton.isClicked()) {
    int xStick = analogRead(JOYSTICK_X);
    if (xStick > xThreshPos && !modeChange) {
      if (modeSel < modeMax) {
        modeSel++;
      }
      modeChange = true;
    }
    if (xStick < xThreshNeg && !modeChange && modeSel > -1) {
      if (modeSel > 0) {
        modeSel--;
      }
      modeChange = true;
    }
    if (xStick < xThreshPos && xStick > xThreshNeg) {
      modeChange = false;
    }
  
    displayInstructions(SET_MODE, modeSel);

  }

  switch (modeSel) {
    case 0: 
      Serial.printf("Free Grid Mode selected\n");
      freeGridMode();
      break;
    case 1:
      Serial.printf("Stud Mode selected\n");
      break;
    default:
      break;
  }
}


void freeGridMode() {
  Serial.printf("FREE GRID MODE\n");
  Button nextButton(JOYSTICK_BUTTON, true);
  int xOffset = 0, yOffset = 0, gridCols = 0, gridRows = 0, colGap = 0, rowGap = 0;
  int prevVal = 0;
  bool firstLoop = true; 

  delay(200); // prevent clickthrough
  
  while (!nextButton.isClicked()) { // x offset
    if (prevVal != xOffset || firstLoop) {
      displayInstructions(FREE_GRID, 0);
      display.setTextSize(2);
      display.printf("%i cm", xOffset);
      display.display();
      firstLoop = false;
    }

    prevVal = xOffset;
    int incr = stickAdjust();
    if ((xOffset == 0 && incr > 0) || xOffset > 0) {
      xOffset += incr;
      // Serial.printf("xOffset: %i cm\n", xOffset);
    }
  }
  firstLoop = true;

  while (!nextButton.isClicked()) { // y offset
    if (prevVal != yOffset || firstLoop) {
      displayInstructions(FREE_GRID, 1);
      display.setTextSize(2);
      display.printf("%i cm", yOffset);
      display.display();
      firstLoop = false;
    }

    prevVal = yOffset;
    int incr = stickAdjust();
    if ((yOffset == 0 && incr > 0) || yOffset > 0) {
      yOffset += incr;
      // Serial.printf("yOffset: %i cm\n", yOffset);
    }
  }
  firstLoop = true;

  while (!nextButton.isClicked()) { // columns
    if (prevVal != gridCols || firstLoop) {
      displayInstructions(FREE_GRID, 2);
      display.setTextSize(2);
      display.printf("%i columns", gridCols);
      display.display();
      firstLoop = false;
    }

    prevVal = gridCols;
    int incr = stickAdjust();
    if ((gridCols == 0 && incr > 0) || gridCols > 0) {
      gridCols += incr;
      // Serial.printf("Columns: %i\n", gridCols);
    }
  }
  firstLoop = true;

  while (!nextButton.isClicked()) { // rows
    if (prevVal != gridRows || firstLoop) {
      displayInstructions(FREE_GRID, 3);
      display.setTextSize(2);
      display.printf("%i rows", gridRows);
      display.display();
      firstLoop = false;
    }

    prevVal = gridRows;
    int incr = stickAdjust();
    if ((gridRows == 0 && incr > 0) || gridRows > 0) {
      gridRows += incr;
      // Serial.printf("Rows: %i\n", gridRows);
    }
  }
  firstLoop = true;

  while (!nextButton.isClicked()) { // column gap
    if (prevVal != colGap || firstLoop) {
      displayInstructions(FREE_GRID, 4);
      display.setTextSize(2);
      display.printf("%i cm", colGap);
      display.display();
      firstLoop = false;
    }

    prevVal = colGap;
    int incr = stickAdjust();
    if ((colGap == 0 && incr > 0) || colGap > 0) {
      colGap += incr;
      // Serial.printf("Column gap: %i cm\n", colGap);
    }
  }
  firstLoop = true;

  while (!nextButton.isClicked()) { // row gap
    if (prevVal != rowGap || firstLoop) {
      displayInstructions(FREE_GRID, 5);
      display.setTextSize(2);
      display.printf("%i cm", rowGap);
      display.display();
      firstLoop = false;
    }

    prevVal = rowGap;
    int incr = stickAdjust();
    if ((rowGap == 0 && incr > 0) || rowGap > 0) {
      rowGap += incr;
      // Serial.printf("Row gap: %i cm\n", rowGap);
    }
  }
}


int8_t stickAdjust() {
  int xThreshPos = 2250;
  int xThreshNeg = 1750;
  while (true) {
    int xStick = analogRead(JOYSTICK_X);
    if (xStick > xThreshPos) {
      return 1;
    }
    if (xStick < xThreshNeg) {
      return -1;
    }
    if (xStick < xThreshPos && xStick > xThreshNeg) {
      return 0;
    }
  }
}


void displayInstructions(Mode mode, int8_t line) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
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
    case SET_MODE:
      switch (line) {
        case -1: // start
          display.printf("SELECT MODE:\nNavigate with stick\n");
          break;
        case 0: // Free grid mode
          display.printf("FREE GRID MODE:\nDefine a grid of\npoints to display.\nClick stick to begin.");
          break;
        case 1: // Stud mode
          display.printf("STUD MODE:\nClick stick to begin.");
          break;
        default:
          break;
      }
      break;
    case MANUAL_DRIVE:
      switch (line) {
        case 0: // bottom left
          display.printf("MANUAL MODE:\nDrive to the exact\nbottom left corner\n(ABOVE BASEBOARD!)\nClick to continue.");
          break;
        case 1: // top left
          display.printf("MANUAL MODE:\nDrive to the exact\ntop left corner\n(BELOW MOLDING!)\nClick to continue.");
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
      switch(line) {
        case 0:
          display.printf("Set distance from\nleft edge of wall.\nClick to continue.\n\n");
          break;
        case 1:
          display.printf("Set distance from\ntop edge of wall.\nClick to continue.\n\n");
          break;
        case 2:
          display.printf("Set # of columns.\nClick to continue.\n\n");
          break;
        case 3:
          display.printf("Set # of rows.\nClick to continue.\n\n");
          break;
        case 4:
          display.printf("Set column gap.\nClick to continue.\n\n");
          break;
        case 5:
          display.printf("Set row gap.\nClick to continue.\n\n");
          break;
        case 6:
          display.printf("Displaying point.\nUse stick to\nchange points.\nClick to end.");
          break;
        default:
          break;
      }
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

  
  // find direction - positive angle = clockwise (looking at the motor shaft)
  int xDir;
  if (xSteps >= 0) { xDir = -1; }
  else             { xDir = 1; }
  int yDir;
  if (ySteps >= 0) { yDir = 1; }
  else             { yDir = -1; }
  
  // find maximum to bound loop
  int xStepsAbs = abs(xSteps);
  int yStepsAbs = abs(ySteps);
  int max;
  (xStepsAbs > yStepsAbs || xStepsAbs == yStepsAbs) ? max = xStepsAbs : max = yStepsAbs;
  
  // Prevent movement outside predefined range (assuming I can roughly center the device on startup using MPU for pitch and compass or Hall sensor for yaw)
  // if (abs(totalStepsX) + xStepsAbs >= stepperLimitX) {
  //   Serial.printf("Attempted X overstep from %i to %i.\n", totalStepsX, totalStepsX + xSteps);
  //   xSteps = 0;
  //   xDir = 0;
  // }
  // if (abs(totalStepsY) + yStepsAbs >= stepperLimitY) {
  //   Serial.printf("Attempted Y overstep from %i to %i.\n", totalStepsY, totalStepsY + ySteps);
  //   ySteps = 0;
  //   yDir = 0;
  // }

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

float deg_2_rad(float d)
{
    return d * (M_PI / 180.0);
}

float rad_2_deg(float r)
{
    return (r * 180.0) / M_PI;
}