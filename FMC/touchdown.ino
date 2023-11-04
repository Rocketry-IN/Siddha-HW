#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


Adafruit_MPU6050 mpu;
int threshold = 500;  //Taken from prev reading, Adjust according to requirement.



void setup() {
  Wire.begin();
  mpu.begin();
  Serial.begin(9600);

}

void loop() {



  // Read accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Check acceleration in the y-axis
  float accelY = a.acceleration.y;



  // Remove noise with running average
  float avgY;
  avgY = (accelY*0.9) + (accelY*0.1);


/*
This takes 90% of the new raw value and 10% of the previous filtered value.
For example, if the previous avgY is 500 and the new accelY is 520:
avgY = (520*0.9) + (500*0.1) = 468 + 50 = 518
So the new filtered value is 518 instead of the raw 520.
Like I said, we have to play a lil bit with it to find the ideal value!

*/


  if(avgY > threshold) {
    Serial.println("Ascending"); 
  }


  else if(avgY < -threshold) {
    Serial.println("Descending");
  }


  else {
    Serial.println("Apogee");
  }



  delay(10);



 
}