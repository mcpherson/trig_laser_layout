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

// X/Y array readability
const int X = 0;
const int Y = 1;

// STATES
bool laserToggle = false;
bool measureToggle = false;
bool isDriving = false;
bool normalized = false;

// typedef struct {
//   int x,
//   int y
// } Steps;

// Steps stepperLimit;
// Steps totalSteps;
// Steps BLsteps;
// Steps TLsteps;
// Steps TRsteps;
  
// WALL DIMENSIONS
// N = normal vector, BL/TL/TR = bottom left/top left/top right corners (defined via manualDrive())
int16_t Ndistance;
// wall corner points
int16_t BLdistance[2], TLdistance[2], TRdistance[2];

int16_t BLsteps[2];
int16_t TLsteps[2];  
int16_t TRsteps[2]; 

float BLtheta[2], TLtheta[2], TRtheta[2];
int16_t wallWidth, wallHeight;

// user-defined points
// hardcoded for now, will refactor and allocate dynamically when I have time to learn how
int16_t P0steps[2]; // reference on left edge of wall
int16_t P1steps[2]; // first real point
int16_t P2steps[2]; 
int16_t P3steps[2]; 

float P0theta[2], P1theta[2], P2theta[2], P3theta[2];
int16_t P0distance[2], P1distance[2], P2distance[2], P3distance[2];

// vectors
int16_t TLtoBLsteps[2], TLtoTRsteps[2];
float TLtoBLnorm[2], TLtoTRnorm[2];

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
void stepToFrom(int xStepsTo, int yStepsTo, int xStepsFrom, int yStepsFrom);
void stepTo(float xSteps, float ySteps, bool stepwise);
void calculateWall();
void calculateLayout(int wallOffset, int ceilOffset, int cols, int rows, int colGap, int rowGap);
void showPoints();
float degToRad(float d);
float radToDeg(float r);


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
  
  if (isDriving) manualDrive(); 

  // brief pause after startup to allow manual adjustment for bad shutdown position
  if (millis() - lastTime > 5000) { 
    if (!normalized) normalize(); 
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
  Serial.printf("NORMALIZED! %imm\n", Ndistance);
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
  BLsteps[X] = totalStepsX;
  BLsteps[Y] = totalStepsY;
  BLtheta[X] = (float)BLsteps[X] / SPD;
  BLtheta[Y] = (float)BLsteps[Y] / SPD;
  BLdistance[X] = Ndistance * tan(degToRad(BLtheta[X]));
  BLdistance[Y] = Ndistance * tan(degToRad(BLtheta[Y]));
  
  // Top left corner
  displayInstructions(MANUAL_DRIVE, 1);
  delay(100); // prevent clickthrough
  while (!nextButton.isClicked()) { drive(); } 
  delay(100); // prevent clickthrough
  TLsteps[X] = totalStepsX;
  TLsteps[Y] = totalStepsY;
  TLtheta[X] = (float)TLsteps[X] / SPD;
  TLtheta[Y] = (float)TLsteps[Y] / SPD;
  TLdistance[X] = Ndistance * tan(degToRad(TLtheta[X]));
  TLdistance[Y] = Ndistance * tan(degToRad(TLtheta[Y]));

  // Top right corner
  displayInstructions(MANUAL_DRIVE, 2);
  delay(100); // prevent clickthrough
  while (!nextButton.isClicked()) { drive(); } 
  delay(100); // prevent clickthrough
  TRsteps[X] = totalStepsX;
  TRsteps[Y] = totalStepsY;
  TRtheta[X] = (float)TRsteps[X] / SPD;
  TRtheta[Y] = (float)TRsteps[Y] / SPD;
  TRdistance[X] = Ndistance * tan(degToRad(TRtheta[X]));
  TRdistance[Y] = Ndistance * tan(degToRad(TRtheta[Y]));

  // DEBUG
  Serial.printf("BLdX: %i, BLdY: %i, X: %i, Y: %i\n", BLdistance[X], BLdistance[Y], BLsteps[X], BLsteps[Y]);
  Serial.printf("BLthetaX: %0.3f, BLthetaY: %0.3f\n", BLtheta[X], BLtheta[Y]);
  Serial.printf("TLdX: %i, TLdY: %i, X: %i, Y: %i\n", TLdistance[X], TLdistance[Y], TLsteps[X], TLsteps[Y]);
  Serial.printf("TLthetaX: %0.3f, TLthetaY: %0.3f\n", TLtheta[X], TLtheta[Y]);
  Serial.printf("TRdX: %i, TRdY: %i, X: %i, Y: %i\n", TRdistance[X], TRdistance[Y], TRsteps[X], TRsteps[Y]);
  Serial.printf("TRthetaX: %0.3f, TRthetaY: %0.3f\n", TRtheta[X], TRtheta[Y]);
  stepTo(-totalStepsX, -totalStepsY, true);

  // attachInterrupt(JOYSTICK_BUTTON, beginManualDrive, FALLING);

  // reset stepper speed to max when finished
  xStepper.setSpeed(speed);
  yStepper.setSpeed(speed);

  isDriving = false;

  calculateWall();
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
  xOffset *= 10; // convert to mm
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
  yOffset *= 10; // convert to mm
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
  colGap *= 10; // convert to mm
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
  rowGap *= 10; // convert to mm
  calculateLayout(xOffset, yOffset, gridCols, gridRows, colGap, rowGap);
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


void stepToFrom(int xStepsTo, int yStepsTo, int xStepsFrom, int yStepsFrom) {
  stepTo((float)(xStepsTo - xStepsFrom), (float)(yStepsTo - yStepsFrom), true);
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
  // Serial.printf("Total steps X: %i, Total steps Y: %i\n", totalStepsX, totalStepsY);
  // Serial.printf("xSteps: %i, ySteps: %i, max: %i, xDir: %i, yDir: %i\n", xSteps, ySteps, max, xDir, yDir);
}


void calculateWall() {

  // vector described by left wall points
  TLtoBLsteps[X] = BLsteps[X] - TLsteps[X];
  TLtoBLsteps[Y] = BLsteps[Y] - TLsteps[Y];
  float magnitude = sqrt(pow(TLtoBLsteps[X], 2) + pow(TLtoBLsteps[Y], 2));
  if (magnitude == 0) {
    Serial.printf("0 magnitude error");
    return;
  }
  TLtoBLnorm[X] = TLtoBLsteps[X] / magnitude;
  TLtoBLnorm[Y] = TLtoBLsteps[Y] / magnitude;

  // vector described by ceiling points
  TLtoTRsteps[X] = TRsteps[X] - TLsteps[X];
  TLtoTRsteps[Y] = TRsteps[Y] - TLsteps[Y];
  magnitude = sqrt(pow(TLtoTRsteps[X], 2) + pow(TLtoTRsteps[Y], 2));
  if (magnitude == 0) {
    Serial.printf("0 magnitude error");
    return;
  }
  TLtoTRnorm[X] = TLtoTRsteps[X] / magnitude;
  TLtoTRnorm[Y] = TLtoTRsteps[Y] / magnitude;

  // calculate wall dimensions in mm
  wallHeight = sqrt(pow(TLdistance[X] - BLdistance[X], 2) + pow(TLdistance[Y] - BLdistance[Y], 2)); 
  wallWidth = sqrt(pow(TRdistance[X] - TLdistance[X], 2) + pow(TRdistance[Y] - TLdistance[Y], 2)); 

  Serial.printf("TLtoBLnormX: %0.3f, TLtoBLnormY: %0.3f\n", TLtoBLnorm[X], TLtoBLnorm[Y]);
  Serial.printf("TLtoTRnormX: %0.3f, TLtoTRnormY: %0.3f\n", TLtoTRnorm[X], TLtoTRnorm[Y]);
  Serial.printf("Width: %i, Height: %i\n", wallWidth, wallHeight);

}

void calculateLayout(int wallOffset, int ceilOffset, int cols, int rows, int colGap, int rowGap) {

  // eventually this will be dynamic. Only one row of three points is calculated currently for demonstration purposes.

  // Wall point calcs: steps -> angle -> dist, User point calcs: dist -> angle -> steps
  // from TL, along TLtoBL to P0 (yOffset)
  P0distance[X] = TLdistance[X] + (TLtoBLnorm[X] * ceilOffset);
  P0distance[Y] = TLdistance[Y] + (TLtoBLnorm[Y] * ceilOffset);
  P0theta[X] = radToDeg(atan((float)P0distance[X] / Ndistance));
  P0theta[Y] = radToDeg(atan((float)P0distance[Y] / Ndistance));
  P0steps[X] = P0theta[X] * SPD;
  P0steps[Y] = P0theta[Y] * SPD;

  // from P0, along TLtoTR to P1 (xOffset)
  P1distance[X] = P0distance[X] + (TLtoTRnorm[X] * wallOffset);
  P1distance[Y] = P0distance[Y] + (TLtoTRnorm[Y] * wallOffset);
  P1theta[X] = radToDeg(atan((float)P1distance[X] / Ndistance));
  P1theta[Y] = radToDeg(atan((float)P1distance[Y] / Ndistance));
  P1steps[X] = P1theta[X] * SPD;
  P1steps[Y] = P1theta[Y] * SPD;

  // from P1, along TLtoTR to P2 (colGap) 
  P2distance[X] = P1distance[X] + (TLtoTRnorm[X] * colGap);
  P2distance[Y] = P1distance[Y] + (TLtoTRnorm[Y] * colGap);
  P2theta[X] = radToDeg(atan((float)P2distance[X] / Ndistance));
  P2theta[Y] = radToDeg(atan((float)P2distance[Y] / Ndistance));
  P2steps[X] = P2theta[X] * SPD;
  P2steps[Y] = P2theta[Y] * SPD;

  // from P2, along TLtoTR to P3 (colGap) 
  P3distance[X] = P2distance[X] + (TLtoTRnorm[X] * colGap);
  P3distance[Y] = P2distance[Y] + (TLtoTRnorm[Y] * colGap);
  P3theta[X] = radToDeg(atan((float)P3distance[X] / Ndistance));
  P3theta[Y] = radToDeg(atan((float)P3distance[Y] / Ndistance));
  P3steps[X] = P3theta[X] * SPD;
  P3steps[Y] = P3theta[Y] * SPD;

  Serial.printf("P0dX: %i, P0dY: %i\n", P0distance[X], P0distance[Y]);
  Serial.printf("P1dX: %i, P1dY: %i\n", P1distance[X], P1distance[Y]);
  Serial.printf("P2dX: %i, P2dY: %i\n", P2distance[X], P2distance[Y]);
  Serial.printf("P3dX: %i, P3dY: %i\n", P3distance[X], P3distance[Y]);
  Serial.printf("===============\n");
  Serial.printf("P0stepsX: %i, P0stepsY: %i\n", P0steps[X], P0steps[Y]);
  Serial.printf("P1stepsX: %i, P1stepsY: %i\n", P1steps[X], P1steps[Y]);
  Serial.printf("P2stepsX: %i, P2stepsY: %i\n", P2steps[X], P2steps[Y]);
  Serial.printf("P3stepsX: %i, P3stepsY: %i\n", P3steps[X], P3steps[Y]);

  showPoints();

}


void showPoints() {
  Button nextButton(JOYSTICK_BUTTON, true);

  // eventually this will be dynamic. Only one row of three points is calculated currently for demonstration purposes.

  stepTo((float)P0steps[X], (float)P0steps[Y], true);
  while (true) {
    if (nextButton.isClicked()) {
      break;
    }
  }
  
  stepToFrom(P1steps[X], P1steps[Y], P0steps[X], P0steps[Y]);
  while (true) {
    if (nextButton.isClicked()) {
      break;
    }
  }

  stepToFrom(P2steps[X], P2steps[Y], P1steps[X], P1steps[Y]);
  while (true) {
    if (nextButton.isClicked()) {
      break;
    }
  }

  stepToFrom(P3steps[X], P3steps[Y], P2steps[X], P2steps[Y]);
  while (true) {
    if (nextButton.isClicked()) {
      break;
    }
  }
  // return to origin
  stepTo((float)-P3steps[X], (float)-P3steps[Y], true);
  
}

float degToRad(float d) {
    return d * (M_PI / 180.0);
}

float radToDeg(float r) {
    return (r * 180.0) / M_PI;
}