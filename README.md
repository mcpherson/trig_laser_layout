# Trig Laser Layout
This device uses a gimbal-mounted distance sensor to measure a wall and display a user-defined set of points on it with a visible laser.


## PURPOSE
Measuring and laying out points on a wall by hand can be difficult, and it's something that most of us have struggled with at some point. A person might have to use a stepladder to measure from the ceiling, or they may need to create evenly spaced holes across a wall's entire width. Gravity-based leveling tools like bubble levels and plumb bobs can be inaccurate due to a structure's misalignment. Currently available computerized laser leveling devices are very expensive. My goal with this project was to create a lightweight, low-cost device that could be placed in front of a wall and calculate points on the wall regardless of the device's orientation relative to the structure. This was created as the capstone project for the IoT/Rapid Prototyping bootcamp offered by [CNM Ingenuity](https://www.cnmingenuity.org/).

## FUNCTIONALITY
Five components are key to this device's functionality (apart from the Particle Photon 2 microcontroller) - stepper motors, a 2-axis analog joystick, an OLED display, a visible laser diode, and the VL53L1X time of flight distance sensor by ST Microelectronics. The gimbal is driven by two stepper motors (one for each axis). Each step the motors take rotates the gimbal by a known angle, which is one of the keys to the trigonometry used to lay out points. The other is the length of the normal vector to the wall, which is measured by the VL53. The joystick allows a user to drive the gimbal to the corners of the wall, eliminating a planned corner detection method that's impossible with the level of accuracy inherent to the project's components. The laser diode visually represents where the gimbal is pointed. Instructions are displayed on the OLED display during operation. 

### VL53L1X time of flight sensor
This sensor uses a principle called "time of flight" (ToF) that measures the time it takes photons to be emitted, reflected, and detected. The receiver has a configurable field of view (from the default 27° down to 15° via interaction with the API). It provides fairly accurate point distance measurements (within ~5cm of error at the max range of the sensor, about 4 meters). It communicates via I2C, allowing for seamless integration into IoT devices. Its low cost and small footprint make it ideal for lightweight applications.

### 28BYJ-48 stepper motors
The low-cost stepper motors used in this project have 32 internal positions. To reduce the angular rotation per step, a 1/64 gear ratio is built in. 2048 steps are needed to complete a full revolution, meaning that each step rotates the motor by about 0.176°. This is all fairly accurate for this application. However, these motors had an issue that caused significant issues for this project (discussed below).

### 2-axis analog joystick
These joysticks are commonly found as thumb controls in game console controllers. I mounted mine inside a 3D-printed housing designed by Chet Johnston ([available here on Thingiverse](https://www.thingiverse.com/thing:700346) - many thanks to Chet for the great design!). The joystick contains a push button and 2 potentiometers (one for each axis) that return analog values from 0-4095 depending on which side of their range of motion they're on. The joystick is the only control for this device. It allows a user to manually drive the gimbal, and clicking the stick allows the user to proceed through the program flow. I also wrote code to allow a user to select and enter values during layout mode setup.

### OLED display
This 128x64 display communicates over I2C. The limited size of the screen necessitates concise instructions. Switch/case statements govern which instructions to display at any given point in the program's operation. 

### Visible laser diode
This small laser diode from Adafruit runs on 3V3 power and can easily be switched on or off using a digital input passed through a TIP120 transistor. Its small size, low price, and decent beam quality made it ideal for this application.

## PROGRAM FLOW

### Normalization
After turning on the device, the gimbal begins a normalization procedure. First, it sweeps across the X-axis, looking for the shortest distance measured by the ToF sensor. Once that point is found, the procedure is repeated for the Y-axis above and below the X-minimum. The point where the X and Y distance is shortest represents the normal (perpendicular) vector to the wall from the gimbal. This point is the origin of the gimbal coordinate system. The measured distance is stored for calculations.

### Manual drive
Once the normal vector is found, the user must drive the gimbal to the bottom left, top left, and top right corners of the room using the joystick. Because the origin (0,0) is at the normal point, the number of steps to reach each corner in X and Y is known (and therefore the angles traveled to those corners). 

### Wall calculations
After defining the points on the wall, the calculations begin. First, the total steps taken to reach each point from the origin are logged and translated into angles. Those angles are then used to calculate the distance to each point along each axis using `D * tan(theta)` (D = normal vector length). With those points, the distance between the points defining the left and top edges can be found using `wallHeight = sqrt(pow(TLdistance[X] - BLdistance[X], 2) + pow(TLdistance[Y] - BLdistance[Y], 2));`, giving us the height and width of the wall. The vectors between these points are also calculated and normalized for future calculations.
```
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
```

### User input
The user is prompted to select a mode and enter values using the thumbstick. Currently, only Free Grid mode is implemented. This mode allows a user to enter an X and Y offset, a number of rows and columns, and the gaps between rows and columns. 

### Point calculation
The device then calculates the position of those points on the wall using normalized vectors:
```
// Wall point calcs: steps -> angle -> dist, User point calcs: dist -> angle -> steps
// from TL, along TLtoBL to P0 (yOffset)
P0distance[X] = TLdistance[X] + (TLtoBLnorm[X] * ceilOffset);
P0distance[Y] = TLdistance[Y] + (TLtoBLnorm[Y] * ceilOffset);
P0theta[X] = radToDeg(atan((float)P0distance[X] / Ndistance));
P0theta[Y] = radToDeg(atan((float)P0distance[Y] / Ndistance));
P0steps[X] = P0theta[X] * SPD;
P0steps[Y] = P0theta[Y] * SPD;
```
These points are then displayed with the visible laser one at a time. The user clicks the joystick to proceed to the next point. When finished, a final click drives the gimbal back to the normal vector.

## LIMITATIONS AND POTENTIAL SOLUTIONS
### Stepper motor slop
Small spaces between the teeth of the gears result in a loss of motion when the rotation direction is changed. This is known as backlash, play, or slop. 

The motors I used have normal slop, which I accounted for using a rubber band to pull the X-axis in one direction. For the Y-axis, the weight of the wires hanging from the gimbal generally kept the Y-axis pulled upward. 

However, there is a second "slop", or dead zone, in these motors, often happening 20° or more after the motor direction changes. This dead zone may be delayed due to precession in the gearing system. The normal slop wasn't much of an issue, but the dead zone creates significant issues for this device. There are many methods for dealing with slop, but most assume that the slop will be taken up immediately. The fact that it's taken up at different points in my motors' motion makes the normalization process described above difficult. 

My strategy to deal with the dead zone while normalizing is to step the motors to about 45° from their starting position to ensure that the dead zone is taken up in that direction. Then, the motors change direction step back toward the anticipated normal location to take up the dead zone in that direction prior to the normalization measurements. This was not foolproof - it worked one day, but after more testing the next day, the dead zone seemed to have moved into the range covered by the normalization measurements, giving false positives when checking for different distances.

This is also problematic for accurate layout. Currently, I don't have many ways of dealing with this. The range of motion of the device is limited (explained below), meaning that overrotation to take up the dead zone is not always an option when displaying points that are close to the edge of the wall. I have accepted that this is a quirk of the motors I used for the device and is simply something I had to work around during the brief time I had to complete the project. 

### VL53L1X accuracy
Though extremely impressive in scale, accuracy, and cost, the VL53L1X isn't perfect. Adafruit has created a lightweight library that exposes a few of the basic measurement functions from the API created by ST. The ST API contains many methods that can be used to improve accuracy for certain applications. Unfortunately, I lack the experience with C necessary to dig further into the API for things like SPAD/ROI configuration and other calibration features. This is one of the main ways to improve the accuracy of this project without using a more expensive LIDAR sensor.

Ambient light conditions and surface reflectivity are important to consider when using these sensors. Rooms are likely to be exposed to sunlight, which negatively impacts accuracy. Surface reflectivity and texture also factors in. The values returned by the sensor are somewhat variable, so I did some averaging with multiple measurements to decrease the error. Post-averaging, the sensor was accurate within 1-2mm at the scale I used for testing (a mock wall of about 1 meter in height and width). This variance made the normalization difficult, as the sensor might read the same value across an arc of several degrees. Due to time constraints, I implemented a temporary solution that results in an error of about 2cm when finding the normal vector. This method checks for at least two successive distance readings that are greater than or equal to the previous one, then steps the motor back to a point in the middle of the similar value range.

### Wire drag
Because this device is a prototype that I had a couple weeks to complete, wire routing was not a consideration. 6 wires are connected to the gimbal - 4 for the ToF sensor and 2 for the visible laser. Currently, these wires dangle from the back of the gimbal and are fairly stiff because I didn't have access to very small-gauge wires. Since the motors are fast, it was easy to get the gimbal wrapped up in its wires if my stepper code went awry. At one point during development, the device even pulled its own wires out before I was able to pull the plug.

For a gimbal, a no-wires solution is best, but it wasn't in the cards for a project like this. In future iterations of this device, I would use slip rings to transfer power and data through the gimbal. This would allow it to rotate freely, giving it a full range of motion to make modes like room scanning possible. 

### Laser/sensor alignment
Another difficulty I had was ensuring that the visible laser's position was actually pointing where the gimbal is "looking". I designed the gimbal such that the laser and sensor mounts were parallel. The sensor is mounted to a breakout board created by Adafruit, so I created a flat face to mount it to. I put rubber tubing around the screws I used to attach it and tightened the board down over the tubing. The rebound in the tubing pushed the board up against the back of the screws, allowing me to make fine adjustments to the corners. Because the IR laser of the sensor is invisible without an IR camera, getting a true alignment was impossible. Instead, I brought it close to a true parallel using a reference surface.

I mounted the visible laser with set screws in a barrel that hung below the sensor. The laser and sensor's beams were emitted about 1cm from each other, which is as close as I could possibly get them without potentially drilling a hole in the sensor's breakout board. I accepted this offset as a quirk of the project, and if this was truly a precision application, I would use the offset in my calculations whenever the position of the laser dot mattered (manual drive and point display).

### 3D projection
This was my first time working with a gimbal. In my mind's eye, just a few known values would be necessary to fully describe the wall relative to the sensor. Unfortunately, this wasn't the case. The gimbal behaves normally when close to the normal vector, but as it gets further away, the tilt of the wall relative to the gimbal's point of view becomes apparent. When the laser is driven straight across the X-axis at the top of the wall, the line described is a conic section. As a result, calculated points were also skewed relative to the wall's tilt. 

To get a truly straight line (both physically and in calculations), 3D projection/transformation between the sensor coordinates and wall coordinates is required. This is beyond my skills (for now). I discovered this issue late in the process of designing and building this project, so I was unable to fully account for it. I was able to get a decent result by mapping out points close to the normal vector, which proves that I'm at least able to calculate points relative to the corners of the wall. 


## FUTURE IMPROVEMENTS
- Correct 3D projection
  - The project would actually work very well (considering all of the other challenges) if I figured this out.
- Better components
  - Better stepper motors
    - Removing the mid-range dead zone would greatly improve functionality
  - Better distance sensor (e.g. LIDAR)
    - With a smaller margin of error, accurately measuring the normal would be easier
    - I could also use improved accuracy to get point measurements, entirely eliminating the need for manual drive
  - Slip rings would eliminate wire drag and allow unrestrained motion
- Make it self-orienting - several possible methods:
  - Use a Hall sensor to detect the gimbal position
  - Use a gyroscope to measure pitch to level out the Y-axis
  - Use EEPROM to store and return to a set of known steps (from the last position it was in before being switched off)
- More modes
  - Stud mode: find one stud, drive to it, and display all other studs with a laser line diode
  - Edge/corner detection
    - Useful for room mapping and other similar applications
  - Multi-room mode 
    - Useful for determining continuity between rooms
    - Room dimensions and relative positions (gathered via accelerometer) can be stored in memory to create a working model of connected interior spaces
