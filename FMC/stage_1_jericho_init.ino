#include <Wire.h>
#include <Adafruit_BMP280.h>  
#include <Adafruit_MPU6050.h> // Reference: https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/arduino
#include <Adafruit_Sensor.h>
#include <SD.h>

// Define MPU6050 object
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;  
File dataFile;
//Unneeded
float ax, ay, az;
float gx, gy, gz;

void setup() {
  Serial.begin(9600);

  if (!SD.begin(10)) {
    Serial.println("SD card initialization failed");
    return;
  }

  if (!bmp.begin(0x76)) {
    Serial.println("shit soldering!");
    while (1);
  }

  // Initialize MPU6050
  while(!mpu.begin()) 
  {   
      Serial.println("shit wiring");
      delay(500); 
  }

  dataFile = SD.open("jericho_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, bmp_pressure, bmp_temperature, time_ms");
    dataFile.close();
  } else {
    Serial.println("Error opening jericho_data.csv");
  }
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ElectronicCats Libary [REDUNDANT]: mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  

  float bmp_pressure = bmp.readPressure() / 100.0F;  
  float bmp_temperature = bmp.readTemperature();    

  dataFile = SD.open("jericho_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(ax); dataFile.print(", ");
    dataFile.print(ay); dataFile.print(", ");
    dataFile.print(az); dataFile.print(", ");
    dataFile.print(gx); dataFile.print(", ");
    dataFile.print(gy); dataFile.print(", ");
    dataFile.print(gz); dataFile.print(", ");
    dataFile.print(bmp_pressure); dataFile.print(", ");
    dataFile.print(bmp_temperature); dataFile.print(", ");
    dataFile.print(millis());
    dataFile.println();
    dataFile.close();
  } else {
    Serial.println("Error writing to data.csv");
  }

  delay(1000);  // Adjust the delay to control data logging frequency
}
