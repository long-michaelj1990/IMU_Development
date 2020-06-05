#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);

  int8_t boardTemp = bno.getTemp();
 // Serial.print(F("temperature: "));
 // Serial.println(boardTemp);


  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printEvent(sensors_event_t* event) {
 // Serial.println();
 // Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    float AccX = event->acceleration.x;
    float AccY = event->acceleration.y;
    float AccZ = event->acceleration.z;
    Serial.print(F("Acceleration: "));
    Serial.print(AccX);
    Serial.print(F(" "));
    Serial.print( AccY);
    Serial.print(F(" "));
    Serial.print(AccZ);
    Serial.println(F(""));
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    float OrientX = event->orientation.x;
    float OrientY = event->orientation.y;
    float OrientZ = event->orientation.z;
    Serial.print(F("Orientation: "));
    Serial.print(OrientX);
    Serial.print(F(" "));
    Serial.print(OrientY);
    Serial.print(F(" "));
    Serial.print(OrientZ);
    Serial.println(F(""));
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    float MagX = event->magnetic.x;
    float MagY = event->magnetic.y;
    float MagZ = event->magnetic.z;
    Serial.print(F("Magnetic: "));
    Serial.print(MagX);
    Serial.print(F(" "));
    Serial.print(MagY);
    Serial.print(F(" "));
    Serial.print(MagZ);
    Serial.println(F(""));
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    float GyroX = event->gyro.x;
    float GyroY = event->gyro.y;
    float GyroZ = event->gyro.z;
    Serial.print(F("Gyro: "));
    Serial.print(GyroX);
    Serial.print(F(" "));
    Serial.print(GyroY);
    Serial.print(F(" "));
    Serial.print(GyroZ);
    Serial.println(F(""));
  }
}
