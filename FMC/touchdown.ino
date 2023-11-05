/*
 Rocket Touchdown Logic

 Uses BMP280 pressure sensor to detect apogee and deploy parachutes.
 Uses MPU6050 accelerometer and gyro to track orientation during descent.
 Beeps a piezo buzzer after landing is detected.

 Hardware Connections:

 BMP280 - I2C
 MPU6050 - I2C
 chute pins - D9 and D10
 Buzzer - D6

 Created by sunnygavali02
*/

#include <Adafruit_BMP280.h> // Pressure sensor library
#include <Adafruit_MPU6050.h> // Motion sensor library
#include <Servo.h> // Servo library


// Object instances
Adafruit_BMP280 bmp; // Pressure sensor  
Adafruit_MPU6050 mpu; // Motion sensor
Servo drogueChute; // Drogue parachute servo
Servo mainChute; // Main parachute servo


// Altitude thresholds (meters)
const float seaLevelPressure = 1013.25;  
int deployDrogue = 1000;   //meters
int deployMain = 500; //meters


int landedThreshold = 2;  //meters
/* It represents an altitude threshold in meters above ground level.
When the calculated altitude from the BMP280 pressure sensor drops below this threshold,
it indicates the rocket has very nearly reached the ground.*/

// Servo pins 
const int droguePin = 9;
const int mainPin = 10;
const int buzzer = 6;

void setup() {

  Serial.begin(9600); // Initialize serial monitor


 pinMode(buzzer, OUTPUT);
  // Initialize BMP280
  if(!bmp.begin()){
    Serial.println("BMP280 error!");
    while(1);
  }

  // Initialize MPU6050
  if(!mpu.begin()){ 
    Serial.println("MPU6050 error!");
    while(1);
  }

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //Setting range as discussed prev
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); //Approximately 
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);  //Approx. 

  // Attach pins
  drogueChute.attach(droguePin);  
  mainChute.attach(mainPin);

}

void loop() {

  // Read BMP280 pressure
  float pressure = bmp.readPressure(); 
  
  // Calculate altitude
  float atmospheric = pressure / seaLevelPressure;
  float altitude = 44330 * (1.0 - pow(atmospheric, 0.1903));


/*
The key formula is:

altitude = 44330 * (1 - (pressure / seaLevelPressure)^0.1903)

Where:

    44330 is a constant representing altitude change per pressure change.
    seaLevelPressure is assumed pressure at sea level.
    pressure is measured ambient pressure.
    (pressure / seaLevelPressure)^0.1903 applies a standard atmosphere model.

In summary:

    It calculates the pressure ratio compared to sea level.
    Applies an atmospheric model exponent.
    Scales the result into meters of altitude based on a pressure-altitude constant.

So it converts the pressure reading to an altitude estimate using standard atmosphere modeling.

Thanks to GPT!!!

*/
  // Check if pressure decreased
  static float prevPressure;
  bool isDecreasing = pressure < prevPressure;
  prevPressure = pressure;

  // Detect apogee
  if(isDecreasing == false){
    Serial.println("Apogee reached!"); 
    drogueChute.write(180); // Release drogue
  }

  // Deploy main chute  
  if(altitude < deployMain && isDecreasing){
    Serial.println("Deploying main chute!");
    mainChute.write(180); 
  }

  // Detect landing
  if(altitude < landedThreshold){
    Serial.println("Landed!");
    
    // Beep every 1 sec
    while(1){
      digitalWrite(buzzer, HIGH);
      delay(1000); 
      digitalWrite(buzzer, LOW);
      delay(1000); 
    }
  }

  // Print motion data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y); 
  Serial.print(",");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.println(g.gyro.y);
  
  delay(100);
}
