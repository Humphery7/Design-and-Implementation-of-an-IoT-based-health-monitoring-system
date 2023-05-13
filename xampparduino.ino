

#include <WiFiEsp.h>
#include <WiFiEspClient.h>

#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2, 3); // RX, TX

#define ESP_BAUDRATE  115200
#endif

#include <Wire.h>

float accelx, accely, accelz;
float gForceX, gForceY, gForceZ;

float gyrox, gyroy, gyroz;
float rotX, rotY, rotZ;

/* This arduino code is sending data to mysql server every 30 seconds.
Created By Embedotronics Technologies*/



// #include <MFRC522.h>

 unsigned long timerBefore = 0;
  const int timer = 1000; //1 second
// #include <MFRC522.h>


const char* ssid = "Umidigi A3_Pro";//                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
const char* password = "enogobetterforwhodeystealmydata";
//WiFiClient client;
char server[] = "192.168.43.14";   //eg: 192.168.0.222


WiFiEspClient client;    


void setup()
{
 Serial.begin(19200);
  delay(10);
  Wire.begin();
  setupMPU();
  Serial1.begin(19200);

  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo native USB port only
  }

  Serial.print("Searching for ESP8266..."); 
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  Serial.println("found it!");
    
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  // WiFi.init(&Serial1);
 
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
//  server.begin();
  Serial.println("Server started");
  Serial.print(WiFi.localIP());
  delay(1000);
  Serial.println("connecting...");
 }
void loop()
{ 
 unsigned long timerNow=millis();
if(timerNow-timerBefore>=timer){
    recordAccelRegisters();
  recordGyroRegisters();
  delay(5000);
   Sending_To_phpmyadmindatabase();
   timerBefore=millis();
}
 }

 void Sending_To_phpmyadmindatabase()   //CONNECTING WITH MYSQL
 {
   if (client.connect(server, 80)) {
    Serial.println("connected");
    // Make a HTTP request:
    Serial.print("GET http://localhost/testcode/connect.php?gyroz=");
    client.print("GET http://localhost/testcode/connect.php?gyroz="); 
    Serial.println(gyroz);     
    client.print(gyroz);

    client.print("&gyroy=");
    Serial.println("&gyroy");
    client.print(gyroy);
    Serial.println(gyroy);

    client.print("&gyrox=");
    Serial.println("&gyrox");
    client.print(gyrox);
    Serial.println(gyrox);

    
    client.print("&accelz=");
    Serial.println("&accelz=");
    client.print(accelz);
    Serial.println(accelz);

    client.print("&accely=");
    Serial.println("&accely=");
    client.print(accely);
    Serial.println(accely);

    client.print("&accelx=");
    Serial.println("&accelx=");
    client.print(accelx);
    Serial.println(accelx);

    client.print(" ");      //SPACE BEFORE HTTP/1.1
    client.print("HTTP/1.1");
    client.println();
    client.println("Host: Your Local IP");
    client.println("Connection: close");
    client.println();
    client.flush();
    client.stop();
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
 }










/*
===Contact & Support===
Website: http://eeenthusiast.com/
Youtube: https://www.youtube.com/EEEnthusiast
Facebook: https://www.facebook.com/EEEnthusiast/
Patreon: https://www.patreon.com/EE_Enthusiast
Revision: 1.0 (July 13th, 2016)
===Hardware===
- Arduino Uno R3
- MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)
===Software===
- Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
- Arduino IDE v1.6.9
- Arduino Wire library
===Terms of use===
The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
the software.
*/







void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x1A); // Accessing the register 0x1A (DLPF_CFG)
  Wire.write(2); // Setting the filter bandwidth
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0b00001000); //Setting the gyro to full scale +/- 500deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00010000); //Setting the accel to +/- 8g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  gForceX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gForceY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gForceZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  accelx = ((((gForceX)*9.81) / 4096.0));
  accely = ((((gForceY)*9.81) / 4096.0)); 
  accelz = (((gForceZ)*9.81) / 4096.0);
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  rotX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  rotY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  rotZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  gyrox = ((((rotX)*0.01745) / 65.5));
  gyroy = ((((rotY)*0.01745) / 65.5)); 
  gyroz = ((((rotZ)*0.01745) / 65.5));
}

