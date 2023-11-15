#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
File dataFile;

int elapsedtime;
int r = 1; // set some pin
int g = 2; // set some pin
int b = 3; // set some pin
int buzz = 4; // set some pin
int linearACT = 5; // set some pin
const int chipSelect = 6; // SD card chip select pin, set it to some pin
float bmp_pressure;
float bmp_temperature;
float bmp_alt;
int liftoffThreshold = 9000; // Adjust this threshold as needed
unsigned long liftoffDetectionTime = 0;
int currentevent = 0; // 0 = testing, 1 = liftoff , 2 = apogee, 3 = touchdown
float cALT, iALT, pALT;

// Variables for IMU and BMP280 sensor data
float ax, ay, az;
float gx, gy, gz;

void errorFunc() {
  while (true) {             // error throw function 
    digitalWrite(r, HIGH);
    digitalWrite(g, LOW);
    digitalWrite(b, LOW);
    digitalWrite(buzz, HIGH);
    delay(500);
    digitalWrite(buzz, LOW);
    delay(500);
  }
}
void setup() {
  pinMode(linearACT, OUTPUT);
  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(buzz, OUTPUT);
  currentevent = 0;
  Serial.begin(9600);

  if (!bmp.begin(0x76)) {
    Serial.println("shit soldering!");
    errorFunc();
  } else {
    iALT = bmp.readAltitude(1013.25);
    pALT = iALT;
  }

  while (!mpu.begin()) {
    Serial.println("shit wiring");
    errorFunc();
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    digitalWrite(r, HIGH);
    errorFunc();
  }
  dataFile = SD.open("jericho_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("time_ms, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, bmp_pressure, bmp_temperature, bmp_alt");
    dataFile.close();
  } else {
    Serial.println("Error opening jericho_data.csv");
    errorFunc();
  }
}

void loop() {
  // Liftoff logic by Aditya
  while (currentevent != 1) {
    dataStore();
    latestIMUData();
    if (ay > liftoffThreshold) {
      liftoffDetectionTime = millis();
      currentevent = 1;
      Serial.println("Liftoff confirmed!");
      digitalWrite(r, HIGH);
      digitalWrite(g, HIGH);
      digitalWrite(b, HIGH);
    }
  }

  // Apogee logic
  while (currentevent == 1) {
    dataStore();
    if (altitude() <= 0 || elapsedtime >= 16000) {
      currentevent = 2;
      digitalWrite(linearACT, HIGH);
    }
  }

  // Touchdown logic
  while (currentevent == 2) {
    float delta = abs(altitude());
    if(delta <= 0.17){
      Serial.println("Landed!");
    currentevent = 3;
     // Beep every 1 sec
    while (1) {
      digitalWrite(buzz, HIGH);
      delay(1000);
      digitalWrite(buzz, LOW);
      delay(1000);
      digitalWrite(r, HIGH);
      digitalWrite(g, LOW);
      digitalWrite(b, HIGH);
    }
    }
  }

  while(1){
  float delta = abs(altitude());
    if(delta <= 0.17){
      Serial.println("Landed!");
      currentevent = 3;
    while(1){
      digitalWrite(buzz, HIGH);
      delay(1000); 
      digitalWrite(buzz, LOW);
      delay(1000); 
        digitalWrite(r,HIGH);
        digitalWrite(g,LOW);
        digitalWrite(b,HIGH);
    }
  }
  else{   
    latestIMUData();
    latestBaroData();
    dataStore();
      } 
}
}


void dataStore() {                  // data store function by Mr.Aujas
  latestIMUData();
  latestBaroData();
  dataFile = SD.open("jericho_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(elapsedtime = millis());
    dataFile.print(", ");
    dataFile.print(ax);
    dataFile.print(", ");
    dataFile.print(ay);
    dataFile.print(", ");
    dataFile.print(az);
    dataFile.print(", ");
    dataFile.print(gx);
    dataFile.print(", ");
    dataFile.print(gy);
    dataFile.print(", ");
    dataFile.print(gz);
    dataFile.print(", ");
    dataFile.print(bmp_pressure);
    dataFile.print(", ");
    dataFile.print(bmp_temperature);
    dataFile.print(", ");
    dataFile.print(bmp_alt);
    dataFile.println();
    dataFile.close();
  } 
  else {
    Serial.println("Error writing to data.csv");
  }
  delay(0);  // Adjust the delay to control data logging frequency
}

void latestIMUData() {                 // get latest data for sensors function
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
}

void latestBaroData() {
  bmp_pressure = bmp.readPressure() / 100.0F;
  bmp_temperature = bmp.readTemperature();
  bmp_alt = bmp.readAltitude(1013.25);
}

float altitude() {                       //function to find delta change in altitude by MR KUNJ
  cALT = bmp.readAltitude(1013.25);
  float deltaAlt = cALT - pALT;
  pALT = cALT;
  return (deltaAlt);
}

//Contribution: Aditya, Syed, Aujas, Sunny, Kunj, Aarush, Abhishek, Harsh
