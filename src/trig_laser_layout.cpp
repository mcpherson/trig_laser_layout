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
// #include "vl53l1_api.h"

// ======== Adafruit =========
#define IRQ_PIN 2
#define XSHUT_PIN 3
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// ======== API ========
// #define USE_BLOCKING_LOOP
// #define MEASUREMENT_BUDGET_MS 50
// #define INTER_MEASUREMENT_PERIOD_MS 60
// VL53L1_Dev_t dev;
// VL53L1_DEV Dev = &dev;

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
// Stepper
int SPR = 2048; // steps per revolution
float SPD = SPR / 360.0; // steps Per Degree
int speed = 15; // RPM
int stepsX, stepsY;
// Calcs
uint8_t lastTime; 
int16_t normalX[3], normalY[3]; 

int status;

const int numMeas = 10;
int measCount = 0;
int measurements[numMeas];

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
void normalize();
void orient();
void setMode(Mode mode);
void displayInstructions(Mode mode, uint8_t line);

void stepTo(float xSteps, float ySteps);
void stepperTest();

void printRangingData();


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
  // ======== Adafruit =========
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

  // ======== API ========
  // uint8_t byteData = 0;
  // uint16_t wordData = 0;

  // Wire.begin();
  // Wire.setClock(400000);

  // Dev->I2cDevAddr = 0x52;
  // // Dev->I2cDevAddr = 0x29; // address given by Adafruit...?

  // VL53L1_software_reset(Dev);

  // VL53L1_RdByte(Dev, 0x010F, &byteData);
  // Serial.print(F("VL53L1X Model_ID: "));
  // Serial.println(byteData, HEX);
  // VL53L1_RdByte(Dev, 0x0110, &byteData);
  // Serial.print(F("VL53L1X Module_Type: "));
  // Serial.println(byteData, HEX);
  // VL53L1_RdWord(Dev, 0x010F, &wordData);
  // Serial.print(F("VL53L1X: "));
  // Serial.println(wordData, HEX);

  // Serial.println(F("Autonomous Ranging Test"));
  // status = VL53L1_WaitDeviceBooted(Dev);
  // if(!status) status = VL53L1_DataInit(Dev);
  // if(!status) status = VL53L1_StaticInit(Dev);
  // if(!status) status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_MEDIUM);
  // if(!status) status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, (uint32_t)MEASUREMENT_BUDGET_MS * 1000);
  // if(!status) status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS);
  // if(!status) status = VL53L1_StartMeasurement(Dev);

  // if(status)
  // {
  //   Serial.println(F("VL53L1_StartMeasurement failed"));
  //   while(1);
  // }
  

  // MPU6050?

  // TIMERS
  lastTime = millis();
}

void loop() {
  // ======== Adafruit ========
  int16_t distance;

  // move
  // dwell / measure 



  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    // Serial.print(F("Distance: "));
    // Serial.print(distance);
    // Serial.println(" mm");

    measurements[measCount] = distance;
    measCount++;
    if (measCount == numMeas) {
      int measSum;
      float avgDist;
      for (int i=0; i<numMeas; i++) {
        measSum += measurements[i];
      }
      avgDist = (float)measSum / numMeas;
      Serial.printf("SUM: %i, AVG: %0.0f\n", measSum, avgDist);
      measCount = 0;
    }

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }


  // ======== API ========
  // #ifdef USE_BLOCKING_LOOP

  //   // blocking wait for data ready
  //   status = VL53L1_WaitMeasurementDataReady(Dev);

  //   if(!status)
  //   {
  //     printRangingData();
  //     VL53L1_ClearInterruptAndStartMeasurement(Dev);
  //   }
  //   else
  //   {
  //     Serial.print(F("Error waiting for data ready: "));
  //     Serial.println(status);
  //   }

  // #else

  //   static uint16_t startMs = millis();
  //   uint8_t isReady;

  //   // non-blocking check for data ready
  //   status = VL53L1_GetMeasurementDataReady(Dev, &isReady);

  //   if(!status)
  //   {
  //     if(isReady)
  //     {
  //       printRangingData();
  //       VL53L1_ClearInterruptAndStartMeasurement(Dev);
  //       startMs = millis();
  //     }
  //     else if((uint16_t)(millis() - startMs) > VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS)
  //     {
  //       Serial.print(F("Timeout waiting for data ready."));
  //       VL53L1_ClearInterruptAndStartMeasurement(Dev);
  //       startMs = millis();
  //     }
  //   }
  //   else
  //   {
  //     Serial.print(F("Error getting data ready: "));
  //     Serial.println(status);
  //   }

  //   // Optional polling delay; should be smaller than INTER_MEASUREMENT_PERIOD_MS,
  //   // and MUST be smaller than VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS
  //   delay(10);

  // #endif
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
  if (ySteps >= 0) { yDir = 1; }
  else            { yDir = -1; }

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

// void printRangingData()
// {
//   static VL53L1_RangingMeasurementData_t RangingData;

//   status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
//   if(!status)
//   {
//     Serial.print(RangingData.RangeStatus);
//     Serial.print(F(","));
//     Serial.print(RangingData.RangeMilliMeter);
//     Serial.print(F(","));
//     Serial.print(RangingData.SignalRateRtnMegaCps/65536.0);
//     Serial.print(F(","));
//     Serial.println(RangingData.AmbientRateRtnMegaCps/65336.0);
//   }
// }