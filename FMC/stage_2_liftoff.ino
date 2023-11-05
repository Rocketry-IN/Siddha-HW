#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
/*
  STAGE II: SIDDHA [JERICHO]
  Transitioning to a mode where the rocket continuously loops a piece of code to monitor liftoff.
  You may utilize the acceleration data from the IMU for this purpose.
  Liftoff:
  -> change(upward_axis)>buffer
  -> ensure Accel. in y axis post n milliseconds, and not confirm liftoff
*/

/*
  CODE DETAILS:
  Sketch uses 11598 bytes (37%) of program storage space. Maximum is 30720 bytes.
  Global variables use 627 bytes (30%) of dynamic memory, leaving 1421 bytes for local variables. Maximum is 2048 bytes.
*/

/*
  Install Adafruit MPU6050 Library from Arduino Library Manager,
  along-with dependencies.
  Reference: https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
*/
// Define MPU6050 object
Adafruit_MPU6050 mpu;

// Define liftoff detection parameters
int liftoffThreshold = 500; // Adjust this threshold as needed
unsigned long liftoffWaitTime = 5000; // Time in milliseconds to wait before confirming liftoff
unsigned long liftoffDetectionTime = 0;
bool liftoffConfirmed = false;

void setup() {
  Serial.begin(115200);
  /*
    Make sure you set the baud rate to “115200” in the
    serial port monitor. Because the MPU6050 returns an
    excessive amount of data, this higher speed is required
    to display it.
  */

  // Initialize MPU6050
  while(!mpu.begin()) 
  {   
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      delay(500); 
  }

  /* @Aujas - Set Zeroes */
  
  // set accelerometer range to +-8G
	// mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
	// mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
	// mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}
void loop() {
  // Read accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Check acceleration in the y-axis
  float accelY = a.acceleration.y;

  if (!liftoffConfirmed) {
    // If the rocket is accelerating upwards (negative y-axis acceleration)
    if (accelY < -liftoffThreshold) {
      liftoffDetectionTime = millis();
    }
    // If liftoff is detected and the specified wait time has passed, confirm liftoff
    if (liftoffDetectionTime > 0 && (millis() - liftoffDetectionTime) >= liftoffWaitTime) {
      liftoffConfirmed = true;
      Serial.println("Liftoff confirmed!");
    }
  }
}
