#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


double xPos = 0, yPos = 0, headingVel_X = 0, headingVel_Y = 0, headingVel_Z = 0;
uint16_t BNO055_SAMPLERATE_DELAYMS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS =100; // how often to print the data
//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAYMS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
uint16_t printCount = 0; //counter to avoid printing every 10MS sample


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
 // Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(10);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  //Serial.print("Current Temperature: ");
  //Serial.print(temp);
  //Serial.println(" C");
  //Serial.println("");

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  unsigned long tStart = micros();
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> Accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> Mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> Gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> LinAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> Grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  xPos = xPos + ACCEL_POS_TRANSITION * LinAccel.x();
  yPos = yPos + ACCEL_POS_TRANSITION * LinAccel.y();
  
    // velocity of sensor in the direction it's facing
  headingVel_X = ACCEL_VEL_TRANSITION * LinAccel.x() / cos(DEG_2_RAD * euler.x());
  headingVel_Y = ACCEL_VEL_TRANSITION * LinAccel.y() / cos(DEG_2_RAD * euler.y());
  headingVel_Z = ACCEL_VEL_TRANSITION * LinAccel.z() / cos(DEG_2_RAD * euler.z());

  Serial.print(F("Orientation: "));
  Serial.print(euler.x());
  Serial.print(F(" "));
  Serial.print(euler.y());
  Serial.print(F(" "));
  Serial.print(euler.z());
  Serial.println(F(" "));
  Serial.print(F("Movement: "));
  Serial.print(xPos);
  Serial.print(F(" "));
  Serial.print(yPos);
  Serial.println(F(" "));
  Serial.print(F("Speed: "));
  Serial.print(headingVel_X);
  Serial.print(F(" "));
  Serial.print(headingVel_Y);
  Serial.print(F(" "));
  Serial.print(headingVel_Z);
  Serial.println(F(" "));
  
  
    /* Display the floating point data */
  Serial.print(F("Acceleration: "));
  Serial.print(Accel.x());
  Serial.print(F(" "));
  Serial.print(Accel.y());
  Serial.print(F(" "));
  Serial.print(Accel.z());
  Serial.println(F(""));
    /* Display the floating point data */
  Serial.print(F("Magnatometer: "));
  Serial.print(Mag.x());
  Serial.print(F(" "));
  Serial.print(Mag.y());
  Serial.print(F(" "));
  Serial.print(Mag.z());
  Serial.println(F(""));
    /* Display the floating point data */
  Serial.print(F("Gyroscope: "));
  Serial.print(Gyro.x());
  Serial.print(F(" "));
  Serial.print(Gyro.y());
  Serial.print(F(" "));
  Serial.print(Gyro.z());
  Serial.println(F(""));
    /* Display the floating point data */
  Serial.print(F("LinearAcceleration: "));
  Serial.print(LinAccel.x());
  Serial.print(F(" "));
  Serial.print(LinAccel.y());
  Serial.print(F(" "));
  Serial.print(LinAccel.z());
  Serial.println(F(""));
    /* Display the floating point data */
  Serial.print(F("Gravity: "));
  Serial.print(F(" "));
  Serial.print(Grav.x());
  Serial.print(F(" "));
  Serial.print(Grav.y());
  Serial.print(F(" "));
  Serial.print(Grav.z());
  Serial.println(F(""));

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("Calibration: ");
  Serial.print(system, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);

  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAYMS * 1000))
  {
    //poll until the next sample is ready
  }
  }
 // delay(BNO055_SAMPLERATE_DELAY_MS);
//}
