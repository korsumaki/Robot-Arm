/*
 * Robot arm - Markku Korsumäki
 */

#include <Servo.h>


// Servo pins
// Nano: PWM: 3, 5, 6, 9, 10, and 11. Provide 8-bit PWM output with the analogWrite() function.
const int rotationServoPin = 3;
const int shoulderServoPin = 5;
const int elbowServoPin = 6;

/* 
 * We assume that robot arm have the same length from shoulder to elbow, and from elbow to tip.
 * This will make calculations easier.
 */
const int armLength = 125; // in millimeters

/*
 * Servo positions fine tuning.
 * Use these variables to find exact position for each servo.
 * These will take care of mounting variations.
 */
int fineTuneRotationAngle = 6;  // positive -> left
int fineTuneShoulderAngle = -7; // negative -> shoulder go more forward
int fineTuneElbowAngle = 0;

// Arm mounting position: 0,0
// TODO not yet used
const int minX = -armLength;
const int maxX =  armLength;
const int minY =  0;
const int maxY =  armLength;
const int minZ =  0;
const int maxZ =  armLength;

// Global variables for theoretical servo angles.
// Actual servo angles are calculated in getRealRotationAngle() etc. functions.
// TODO move near calculateAngles() function.
float rotationAngle = 0;  // 0 = to forward
float shoulderAngle = 0;  // 0 = to up
float elbowAngle = 0;     // 0 = towards origo


Servo rotationServo;
Servo shoulderServo;
Servo elbowServo;


// Function prototypes
void linearMove(float x, float y, float z);

//int getPosition(int pin, int min_val, int max_val);
float radToDeg(double rad);
void calculateAngles(float x, float y, float z);
void logValues(int x, int y, int z, double rotationAngle, double shoulderAngle, double elbowAngle);
void unitTest();

bool continueLinearMoveWithSpeed(unsigned long currentTime, bool unittest=false);


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // Run unit tests to make sure all important functions work as those should.
  unitTest();
  
  // Attach servos
  rotationServo.attach(rotationServoPin);
  shoulderServo.attach(shoulderServoPin);
  elbowServo.attach(elbowServoPin);

  // Go to initial position.
  linearMove(0, 150, 0);
  blink_led(5);
}

void loop()
{
  handleMovements();
}




const int SPEED_VERY_SLOW = 20;
const int SPEED_SLOW = 50;
const int SPEED_FAST = 150;
const int SPEED_VERY_FAST = 250;

int currentMovementIndex = -1;

// x, y, z, speed
int movements[][4] = {
  // Kulmat
  /*{ -50, 150, 20, 150 },
    { -50, 150, -40, 50 },
    { -50, 150, 20, 50 },
  {  50, 150, 20, 150 },
    {  50, 150, -40, 50 },
    {  50, 150, 20, 50 },
  {  50,  50, 20, 150 },
    {  50,  50, -40, 50 },
    {  50,  50, 20, 50 },
  { -50,  50, 20, 150 },
    { -50,  50, -40, 50 },
    { -50,  50, 20, 50 },
  {   0, 100, 20, 150 },
    {   0, 100, -40, 20 },
    {   0, 100, 20, 20 },
  */
  {   0, 150, -15, SPEED_FAST },
  {  50, 150, -15, SPEED_FAST },
  {  50,  50, -15, SPEED_FAST },
  { -50,  50, -15, SPEED_FAST },

  { -50, 150, -15, SPEED_FAST },

  { -50, 150, 150, SPEED_FAST },
  {  50, 150, 150, SPEED_FAST },
  {  50,  50, 150, SPEED_FAST },
  { -50,  50, 150, SPEED_FAST },
  { -50, 150, 150, SPEED_FAST },

  //{   0, 150, -20, SPEED_FAST }, // takaisin alkupisteeseen
  { -50, 150, -15, SPEED_FAST }, // alas
  {   0, 150, -15, SPEED_FAST }, // takaisin alkupisteeseen

  //{  0, 150, -15, SPEED_SLOW }, // ojennus
  {  0, 230, -15, SPEED_SLOW }, // ojennus
  {  0, 230,  70, SPEED_SLOW }, // ojennus
  {  0,  40,  70, SPEED_SLOW }, // ojennus
  {  0,  40, -15, SPEED_SLOW }, // ojennus
  {  0, 150, -15, SPEED_SLOW }, // ojennus

  // Ympyrä
  //{  0, 150, -15, SPEED_FAST }, // 1
  { 20, 145, -15, SPEED_FAST },
  { 35, 135, -15, SPEED_FAST },
  { 46, 120, -15, SPEED_FAST },
  { 50, 100, -15, SPEED_FAST }, // 2
  { 46,  80, -15, SPEED_FAST },
  { 35,  65, -15, SPEED_FAST },
  { 20,  55, -15, SPEED_FAST },
  {  0,  50, -15, SPEED_FAST }, // 3
  {-20,  55, -15, SPEED_FAST },
  {-35,  65, -15, SPEED_FAST },
  {-46,  80, -15, SPEED_FAST },
  {-50, 100, -15, SPEED_FAST }, // 4
  {-46, 120, -15, SPEED_FAST },
  {-35, 135, -15, SPEED_FAST },
  {-20, 145, -15, SPEED_FAST },
  {  0, 150, -15, SPEED_FAST }, // 1

  //{  0, 150,  0 -15, SPEED_FAST }, // 1
  { 20, 150,  5 -15, SPEED_FAST },
  { 35, 150, 15 -15, SPEED_FAST },
  { 46, 150, 30 -15, SPEED_FAST },
  { 50, 150, 50 -15, SPEED_FAST }, // 2
  { 46, 150, 70 -15, SPEED_FAST },
  { 35, 150, 85 -15, SPEED_FAST },
  { 20, 150, 95 -15, SPEED_FAST },
  {  0, 150,100 -15, SPEED_FAST }, // 3
  {-20, 150, 95 -15, SPEED_FAST },
  {-35, 150, 85 -15, SPEED_FAST },
  {-46, 150, 70 -15, SPEED_FAST },
  {-50, 150, 50 -15, SPEED_FAST }, // 4
  {-46, 150, 30 -15, SPEED_FAST },
  {-35, 150, 15 -15, SPEED_FAST },
  {-20, 150,  5 -15, SPEED_FAST },
  {  0, 150,  0 -15, SPEED_FAST }, // 1


};

const int movementCountMax = sizeof(movements)/sizeof(int)/4;


void handleMovements()
{
  bool movement_is_ready = false;
  if (currentMovementIndex == -1)
  {
    movement_is_ready = true;
  }
  else
  {
    const unsigned long delay_ms = 20;
    unsigned long current_time_ms = millis();
    static unsigned long timestamp_ms = 0;

    if (timestamp_ms+delay_ms < current_time_ms)
    {
      movement_is_ready = continueLinearMoveWithSpeed( current_time_ms );
    }
  }
  
  if (movement_is_ready)
  {
    currentMovementIndex++;
    if (currentMovementIndex >= movementCountMax)
    {
      currentMovementIndex = 0;
    }

    Serial.print("currentMovementIndex: ");
    Serial.println(currentMovementIndex);

    int x = movements[currentMovementIndex][0];
    int y = movements[currentMovementIndex][1];
    int z = movements[currentMovementIndex][2];
    int speed = movements[currentMovementIndex][3];

    startLinearMoveWithSpeed( x, y, z, speed, millis() );
  }
}


// Interpolate function is similar to map() but this uses some floats.
float interpolate(float x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// This function convert servo angle in degrees to microseconds.
int getAngleInMicroseconds(float angle)
{
  // Now all servos (rotation, shoulder, elbow) have the same corrections
  int fix = -30;
  // 45 = 1000
  // 90 = 1500+fix
  // 135 = 2000+2*fix
  return interpolate(angle, 45, 135, 1000, 2000+2*fix);
}

// ========================
// These getReal*Angle() functions convert calculated angles to real servo angles.
// Similar getReal*AngleUs() functions do the same, but those return microseconds value for servos.
int getRealRotationAngle(float angle)
{
  // swap left <-> right
  // input 0 deg => output midpoint
  float ang = 180-(angle+90);
  ang += fineTuneRotationAngle;
  return constrain(ang, 0, 180);
}

int getRealShoulderAngle(float angle)
{
  //
  float ang = angle+90;
  ang -= fineTuneShoulderAngle;
  return constrain(ang, 0, 180);
}

int getRealElbowAngle(float angle)
{
  //
  float ang = angle;
  ang -= fineTuneElbowAngle;
  return constrain(ang, 0, 180);
}

int getRealRotationAngleUs(float angle)
{
  // swap left <-> right
  // input 0 deg => output midpoint
  float ang = 180-(angle+90);
  ang += fineTuneRotationAngle;
  return getAngleInMicroseconds(ang);
}

int getRealShoulderAngleUs(float angle)
{
  //
  float ang = angle+90;
  ang -= fineTuneShoulderAngle;
  return getAngleInMicroseconds(ang);
}

int getRealElbowAngleUs(float angle)
{
  //
  float ang = angle;
  ang -= fineTuneElbowAngle;
  return getAngleInMicroseconds(ang);
}




int startX = 0;
int startY = 150;
int startZ = 0;
int endX = 0;
int endY = 150;
int endZ = 0;
unsigned long startTime = 0;
unsigned long endTime = 0;


// Set initial endpoint for linear move.
// Unit for speed is mm/sec
void startLinearMoveWithSpeed(int x, int y, int z, int speed, unsigned long currentTime)
{
  endX = x;
  endY = y;
  endZ = z;

  // laske etäisyys
  int dx = startX-endX;
  int dy = startY-endY;
  int dz = startZ-endZ;
  float length = sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2));

  // Calculate duration of movement
  int durationMs = 1000*length/speed;

  // Set times
  startTime = currentTime;
  endTime = currentTime+durationMs;
}

// global only for unit testing
float currentX;
float currentY;
float currentZ;

// Return true when movement is done.
bool continueLinearMoveWithSpeed(unsigned long currentTime, bool unittest)
{
  // interpolate
  //interpolate()
  currentX = interpolate(currentTime, startTime, endTime, startX, endX);
  currentY = interpolate(currentTime, startTime, endTime, startY, endY);
  currentZ = interpolate(currentTime, startTime, endTime, startZ, endZ);

  if (!unittest)
  {
    linearMove(currentX, currentY, currentZ);
  }

  if (currentTime >= endTime)
  {
    startX = endX;
    startY = endY;
    startZ = endZ;
    return true;
  }
  return false;
}



// Convert radians to degrees
float radToDeg(double rad)
{
  return rad*180/PI;
}

/*
 * rotationAngle -> 0 deg = forward, can move about -90 ... +90
 * shoulderAngle -> 0 deg = up, can move about -90 (backwards) ... +90 (forward)
 * elbowAngle -> 0 deg = pointing to shoulder, can move about 0 ... +180
 */
void calculateAngles(float x, float y, float z)
{
  // ===== RotationAngle
  double angleRad = atan( (float)x / (float)y ); // TODO voiko olla 0 tai ääretön?
  rotationAngle = radToDeg(angleRad);

  // ===== ElbowAngle
  // Length of 3d-distance from origo to hand tip
  double c_xyz = sqrt( pow(x, 2) + pow(y, 2) + pow(z, 2)); // hypotenuusa x-y-z avaruudessa

  // (Half of) required elbow angle to get 3d-distance c_xyz
  double half_elbow_angle = asin(c_xyz/2 / armLength);
  elbowAngle = radToDeg( half_elbow_angle*2 );


  // ===== ShoulderAngle
  // Angle of direct distance to get correct height for hand tip
  double height_angle = asin(z / c_xyz);
  
  shoulderAngle = radToDeg( half_elbow_angle - height_angle );
}


// Move robot arm to (x, y, z) with linear movement.
void linearMove(float x, float y, float z)
{
  // Calculate needed angles for given coordinates
  calculateAngles(x, y, z);

  // Move servos (select either degrees or microseconds)

  // Use degrees
  //rotationServo.write( getRealRotationAngle(rotationAngle) );
  //shoulderServo.write( getRealShoulderAngle(shoulderAngle) );
  //elbowServo.write( getRealElbowAngle(elbowAngle) );

  // Use microseconds
  rotationServo.writeMicroseconds( getRealRotationAngleUs(rotationAngle) );
  shoulderServo.writeMicroseconds( getRealShoulderAngleUs(shoulderAngle) );
  elbowServo.writeMicroseconds( getRealElbowAngleUs(elbowAngle) );
}




// Function for blinking builtin led.
// This can be used for debugging by getting specific execution points visible.
// NOTE: This use delay(), so it will stop execution flow during blinking.
void blink_led(int count) {
  for (int i=0; i<count; ++i)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

// =========================================================================
// Test code section
// =========================================================================


// This function is used to find and match servos degrees position vs microseconds position.
// TODO move to test area
void test_movements()
{
  /*int fix = -30; // fix per 45 degrees
  for (int i=0; i<5; ++i)
  {
    delay(500);
    rotationServo.write( 45 );
    delay(500);
    rotationServo.writeMicroseconds( 1000 );
  }
  for (int i=0; i<5; ++i)
  {
    delay(500);
    rotationServo.write( 90 );
    delay(500);
    rotationServo.writeMicroseconds( 1500 + fix );
  }
  for (int i=0; i<5; ++i)
  {
    delay(500);
    rotationServo.write( 135 );
    delay(500);
    rotationServo.writeMicroseconds( 2000+ 2*fix );
  }*/

  int fix = -30;
  for (int i=0; i<5; ++i)
  {
    delay(500);
    elbowServo.write( 45 );
    delay(500);
    elbowServo.writeMicroseconds( 1000 );
  }
  for (int i=0; i<5; ++i)
  {
    delay(500);
    elbowServo.write( 90 );
    delay(500);
    elbowServo.writeMicroseconds( 1500 + fix );
  }
  for (int i=0; i<5; ++i)
  {
    delay(500);
    elbowServo.write( 135 );
    delay(500);
    elbowServo.writeMicroseconds( 2000+ 2*fix );
  }
}


// =========================================================================
// Unit test section
// =========================================================================

// Unit test helpers
#define UT_EXPECT_FLOAT(expected, actual)  UT_check_float(expected, actual, __LINE__)
#define UT_EXPECT_INT(expected, actual)  UT_check(expected, actual, __LINE__)
#define UT_EXPECT_BOOL(expected, actual)  UT_check(expected, actual, __LINE__)


void UT_check(int expected, int actual, int linenumber)
{
  if (expected != actual)
  {
    Serial.println("");
    Serial.print("ERROR (int): expected=");
    Serial.print(expected);
    Serial.print(", actual=");
    Serial.print(actual);
    Serial.print(": line ");
    Serial.println(linenumber);
  }
}

void UT_check(bool expected, bool actual, int linenumber)
{
  if (expected != actual)
  {
    Serial.println("");
    Serial.print("ERROR (bool): expected=");
    Serial.print(expected);
    Serial.print(", actual=");
    Serial.print(actual);
    Serial.print(": line ");
    Serial.println(linenumber);
  }
}

void UT_check_float(double expected, double actual, int linenumber)
{
  float diff = 0.00001;
  if (expected+diff < actual || expected-diff > actual)
  {
    Serial.print("ERROR (float): expected=");
    Serial.print(expected);
    Serial.print(", actual=");
    Serial.print(actual);
    Serial.print(": line ");
    Serial.println(linenumber);
  }
}

// Unit tests
void unitTest()
{
  Serial.println("UNIT TEST begin");
  
  UT_EXPECT_FLOAT(M_PI, PI);
  UT_EXPECT_FLOAT(180, radToDeg(PI));

  calculateAngles(0, 10, 0);
  UT_EXPECT_FLOAT(0, rotationAngle);

  calculateAngles(-50, 50, 0);
  UT_EXPECT_FLOAT(-45, rotationAngle);

  calculateAngles(50, 50, 0);
  UT_EXPECT_FLOAT(45, rotationAngle);

  calculateAngles(0, armLength, 0);
  UT_EXPECT_FLOAT(0, rotationAngle);
  UT_EXPECT_FLOAT(30, shoulderAngle);
  UT_EXPECT_FLOAT(60, elbowAngle);

  calculateAngles(0, armLength, armLength);
  UT_EXPECT_FLOAT(0, rotationAngle);
  UT_EXPECT_FLOAT(0, shoulderAngle);
  UT_EXPECT_FLOAT(90, elbowAngle);

  calculateAngles(10, 30, 20);
  UT_EXPECT_FLOAT(18.4349488, rotationAngle);
  // Cxyz = sqrt( pow(10,2)+pow(30,2)+pow(20,2)) = 37.41657386...

  // half_elbow_angle = 8.6075883946...
  // elbowAngle = 17.21517678

  //double height_angle = asin(z / c_xyz);
  //shoulderAngle = radToDeg( half_elbow_angle - height_angle );

  // These values are for armLength=125
  UT_EXPECT_FLOAT((8.6075883946-32.3115332374), shoulderAngle);
  UT_EXPECT_FLOAT(17.21517678, elbowAngle);

  // Rotation angle
  UT_EXPECT_INT(90,  getRealRotationAngle(0 + fineTuneRotationAngle));
  UT_EXPECT_INT(180, getRealRotationAngle(-90 + fineTuneRotationAngle));
  UT_EXPECT_INT(0,   getRealRotationAngle(90 + fineTuneRotationAngle));
  UT_EXPECT_INT(135, getRealRotationAngle(-45 + fineTuneRotationAngle));
  UT_EXPECT_INT(45,  getRealRotationAngle(45 + fineTuneRotationAngle));

  // Shoulder angle
  UT_EXPECT_INT(90,  getRealShoulderAngle(0 + fineTuneShoulderAngle));
  UT_EXPECT_INT(45,  getRealShoulderAngle(-45 + fineTuneShoulderAngle));
  UT_EXPECT_INT(135, getRealShoulderAngle(45 + fineTuneShoulderAngle));
  UT_EXPECT_INT(180, getRealShoulderAngle(90 + fineTuneShoulderAngle));

  // Elbow angle
  UT_EXPECT_INT(0, getRealElbowAngle(0 + fineTuneElbowAngle));
  UT_EXPECT_INT(90, getRealElbowAngle(90 + fineTuneElbowAngle));
  UT_EXPECT_INT(180, getRealElbowAngle(180 + fineTuneElbowAngle));


  Serial.println("UNIT TEST movement");
  startX = 0;
  startY = 150;
  startZ = 0;
  startLinearMoveWithSpeed(10, 150, 0, 10, 1000);
  UT_EXPECT_INT(startTime, 1000);
  UT_EXPECT_INT(endTime, 2000);
  
  bool cont = continueLinearMoveWithSpeed(1100, true);
  UT_EXPECT_BOOL(cont, false);
  UT_EXPECT_INT(currentX, 1);
  UT_EXPECT_INT(currentY, 150);
  UT_EXPECT_INT(currentZ, 0);
  cont = continueLinearMoveWithSpeed(1300, true);
  UT_EXPECT_BOOL(cont, false);
  UT_EXPECT_INT(currentX, 3);
  UT_EXPECT_INT(currentY, 150);
  UT_EXPECT_INT(currentZ, 0);
  cont = continueLinearMoveWithSpeed(1800, true);
  UT_EXPECT_BOOL(cont, false);
  UT_EXPECT_INT(currentX, 8);
  UT_EXPECT_INT(currentY, 150);
  UT_EXPECT_INT(currentZ, 0);
  cont = continueLinearMoveWithSpeed(2000, true);
  UT_EXPECT_BOOL(cont, true);
  UT_EXPECT_INT(currentX, 10);
  UT_EXPECT_INT(currentY, 150);
  UT_EXPECT_INT(currentZ, 0);

  startLinearMoveWithSpeed(10, 160, 0, 5, 2000);
  UT_EXPECT_INT(startTime, 2000);
  UT_EXPECT_INT(endTime, 4000);
  cont = continueLinearMoveWithSpeed(2000, true);
  UT_EXPECT_BOOL(cont, false);
  UT_EXPECT_INT(currentX, 10);
  UT_EXPECT_INT(currentY, 150);
  UT_EXPECT_INT(currentZ, 0);
  cont = continueLinearMoveWithSpeed(3000, true);
  UT_EXPECT_BOOL(cont, false);
  UT_EXPECT_INT(currentX, 10);
  UT_EXPECT_INT(currentY, 155);
  UT_EXPECT_INT(currentZ, 0);
  cont = continueLinearMoveWithSpeed(4000, true);
  UT_EXPECT_BOOL(cont, true);
  UT_EXPECT_INT(currentX, 10);
  UT_EXPECT_INT(currentY, 160);
  UT_EXPECT_INT(currentZ, 0);


  Serial.println("UNIT TEST end");
  //delay(2000);
  // Starting point after unit tests
  startX = 0;
  startY = 150;
  startZ = 0;
}


#if 0
int getPosition(int pin, int /*min_val*/, int /*max_val*/)
{
  int sensorValue = analogRead(pin);

  int minimi = 77;
  int maksimi = 923;
  //923, 498, 77
  int outputValue = map(sensorValue, maksimi, minimi, 0, 180); // servon ohjaus asteilla
  outputValue = constrain(outputValue, 0, 180);

  //int outputValue = map(sensorValue, maksimi, minimi, 1000, 2000); // servon ohjaus mikrosekunteina
  //int outputValue = map(sensorValue, 0, 1023, min_val, max_val);

  Serial.print("pot: ");
  Serial.print(sensorValue);
  Serial.print(" -> ");
  Serial.println(outputValue);

  return outputValue;
}
#endif // 0


//void logValues(int x, int y, int z, int rotationAngle, int shoulderAngle, int elbowAngle)
void logValues(int x, int y, int z, double rotationAngle, double shoulderAngle, double elbowAngle)
{
  Serial.print("Coordinates: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print("\tAngles: ");
  Serial.print(rotationAngle);
  Serial.print(", ");
  Serial.print(shoulderAngle);
  Serial.print(", ");
  Serial.print(elbowAngle);
  Serial.println();
}
