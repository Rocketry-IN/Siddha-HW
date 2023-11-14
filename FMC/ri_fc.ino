// all intitialization by aujas
#include <Wire.h>
#include <Adafruit_BMP280.h>  
#include <Adafruit_MPU6050.h> // Reference: https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/arduino
#include <Adafruit_Sensor.h>
#include <SD.h>
int elapsedtime;
int r = 1;//set some pin
int g = 2;//set some pin;
int b = 3;//set some pin;
int buzz = 4;//set some pin;
int linearACT = 5; // set some pin
const int chipSelect = 6; // sd card chip select pin, set it to some pin
float bmp_pressure;
float bmp_temperature;
float bmp_alt;
int liftoffThreshold = 9000; // Adjust this threshold as needed
unsigned long liftoffDetectionTime = 0;
int currentevent; // 0 = testing, 1 = liftoff , 2 = apogee and parachute ejected, 3 = toucdown.
float cALT,iALT,pALT; // bruh do better variable names (Mr. Kunj)
// Define MPU6050 object
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;  
File dataFile;
float ax, ay, az;
float gx, gy, gz;

void setup() {
  pinMode(linearACT,OUTPUT);
  pinMode(r,OUTPUT);
  pinMode(g,OUTPUT);
  pinMode(b,OUTPUT);
  pinMode(buzz,OUTPUT);
  currentevent = 0;
  Serial.begin(9600);

  if (!bmp.begin(0x76)) { // check address on I2C scanner code, if bmp not working
    Serial.println("shit soldering!");
    errorFunc();
  }
  else{
    iALT = bmp.readAltitude(1013.25);
    pALT = iALT;
  }

  // Initialize MPU6050
  while(!mpu.begin()) 
  {   
      Serial.println("shit wiring");
      errorFunc();
  }

  // sd card init
    if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    digitalWrite(r,HIGH);
    errorFunc();
  }
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    digitalWrite(r,HIGH);
    errorFunc();
  } else {
   Serial.println("Wiring is correct and a card is present.");
    if (SD.remove("jericho_data.csv")){
      Serial.println("done deleting old file instance");
    }else{
      Serial.println("no previous file existd named: jericho_data.csv");
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
    
// lift off logic by aditya
while(currentevent != 1) {
    dataStore();
    latestIMUData();
    if (ay > liftoffThreshold) {
      liftoffDetectionTime = millis();
      currentevent = 1;
      Serial.println("Liftoff confirmed!");
      digitalWrite(r,HIGH);
      digitalWrite(g,HIGH);
      digitalWrite(b,HIGH);
    }
  }
  

  
  
// apogee logic 
while(currentevent == 1){ // run loop to check apogee
        dataStore(); // rocket goin up
        if(altitude() <= 0 | elapsedtime >= 16000) // with hard coded timer
          {
            currentevent = 2;
            digitalWrite(linearACT,HIGH);       
          }

      
}

// touch down logic 
while(1){
  if(altitude < 2){ // less than 2 meters
    Serial.println("Landed!");
    currentevent = 3
    // Beep every 1 sec
    while(1){
      digitalWrite(buzz, HIGH);
      delay(1000); 
      digitalWrite(buzz, LOW);
      delay(1000); 
        digitalWrite(r,HIGH);
        digitalWrite(g,LOW);
        digitalWrite(b,HIGH);
    }
  }else{
    
    latestIMUData();
    latestBaroData();
    dataStore();
      }
  
}
  
}

void errorFunc(){     // error throw function 
while(true){
  digitalWrite(r,HIGH);
  digitalWrite(g,LOW);
  digitalWrite(b,LOW);
  digitalWrite(buzz,HIGH);
  delay(500);
  digitalWrite(buzz,LOW);
  delay(500);
}
}


void dataStore(){       // data store function by aujas
  latestIMUData();
  latestBaroData();
  dataFile = SD.open("jericho_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(elapsedtime = millis()); dataFile.print(", ");
    dataFile.print(ax); dataFile.print(", ");
    dataFile.print(ay); dataFile.print(", ");
    dataFile.print(az); dataFile.print(", ");
    dataFile.print(gx); dataFile.print(", ");
    dataFile.print(gy); dataFile.print(", ");
    dataFile.print(gz); dataFile.print(", ");
    dataFile.print(bmp_pressure); dataFile.print(", ");
    dataFile.print(bmp_temperature); dataFile.print(", ");
    dataFile.print(bmp_alt);
    dataFile.println();
    dataFile.close();
  } else {
    Serial.println("Error writing to data.csv");
  }
  delay(0);  // Adjust the delay to control data logging frequency
}

void latestIMUData(){     // get latest data for sensors function 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
}
void latestBaroData(){
  bmp_pressure = bmp.readPressure() / 100.0F;  
  bmp_temperature = bmp.readTemperature();  
  bmp_alt = bmp.readAltitude(1013.25); 
}

float altitude(){ //function to find delta change in altitude 
    cALT = bmp.readAltitude(1013.25);
    float deltaAlt = cALT - pALT;
    pALT = cALT;
    return(deltaAlt);
  }
// contribution: aditya, syed, aujas, sunny, kunj, aarush, harsh, abhishek
