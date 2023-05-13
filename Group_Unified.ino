//template id name and auth token

// #define BLYNK_TEMPLATE_ID "TMPLkRml0r_e"
// #define BLYNK_DEVICE_NAME "Project"
// #define BLYNK_AUTH_TOKEN "rWagyD5YGg_SLB-NWGnLHvNYyzLqadbp"

// // Comment this out to disable prints and save space
// #define BLYNK_PRINT Serial

#include <Wire.h>
// #include <ESP8266_Lib.h>
// #include <BlynkSimpleShieldEsp8266.h>


#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiAvrI2c display;
//-------------------------



// or Software Serial on Uno, Nano...
// #include <SoftwareSerial.h>



// // Your ESP8266 baud rate:
// #define ESP8266_BAUD 115200

#define PULSE_SENSOR_PIN A0
#define THRESHOLD_DEFAULT 550



// Your WiFi credentials.
// Set password to "" for open networks.
// const char auth[] PROGMEM = BLYNK_AUTH_TOKEN;
// const char ssid[] PROGMEM = "Redmi 9A";
// const char pass[] PROGMEM = "emmy22081";

uint8_t predict;


uint8_t pulseSensorPin = PULSE_SENSOR_PIN;
uint8_t thresholdValue = THRESHOLD_DEFAULT;

const byte LED13 = 13;
byte a = 0;
uint8_t b= 0;
byte update = 0;
byte lasta = 0;
uint8_t lastb = 0;
byte myBPM = 0;
byte temp=0;
int count = 0; //count variable to keep track of when temp and pusle sensor would display 
byte lm35PIN = A1;
byte pushbutton = 1;
byte pushbutton_var;

const uint8_t Threshold PROGMEM = 550;            // Determine which Signal to "count as a beat", and which to ingore.

// function declarations 
void heartrate_gyroscope();
void MPU6050();
void setupMPU();
void notification(int predict);
void myTimer();
void oleddisplay();
void initial_Message();
int getBeatsPerMinute();





// SoftwareSerial EspSerial(2, 3); // RX, TX

// ESP8266 wifi(&EspSerial);

// // This function creates the timer object. It's part of Blynk library 
// BlynkTimer timer; 



void setup() {

  Serial.begin(115200);
#if RST_PIN >= 0
  display.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  display.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0


  Wire.begin();
  setupMPU();
  pinMode(pushbutton, INPUT);
//  initial message to be displayed on OLED 
  initial_Message();


 
  // // Set ESP8266 baud rate
  // EspSerial.begin(ESP8266_BAUD);
  // delay(10);

  // Blynk.begin(auth, wifi, ssid, pass, "blynk.cloud", 80);
 
  // timer.setInterval(1000L, myTimer);

}

 void loop() {
 
  // Runs all Blynk stuff
//   Blynk.run(); 
  
//   // runs BlynkTimer
//  timer.run(); 

  //heartrate and gyroscope reader 
  heartrate_gyroscope();

  // Display checker 
  oleddisplay();

  
  count++;
  update++;
 
}






void initial_Message()
{
  display.clear();
  display.setFont(Adafruit5x7);
  display.setCursor(0,0);
  display.println(F("  Health Monitoring"));
  display.println(F("        System"));
  display.println();
  display.println();
  display.println(F("  Keep Calm To Get"));
  display.print(F("Accurate BPM Readings")); 
  delay(5000);
  display.clear();
}

void heartrate_gyroscope()
{
  if (count==200){
    myBPM = getBeatsPerMinute();
    if (myBPM>120)
    {
      myBPM = 120;
    }
    int sensorValue PROGMEM = analogRead(lm35PIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage  = sensorValue * (5.0 / 1023.0);
    temp = voltage * 100;
  
    count=0;
  }
  
  else 
  {
      MPU6050();
  }

 if (update==15){
  b = 21; 
  update=0;  
 }
 else
 {
   b =60-(random(510,519)/16);
 }
  
}


void MPU6050()
{ 
float accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

float gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ



  gForceX = ((((accelX)*9.81) / 4096.0)-16.20);
  gForceY = ((((accelY)*9.81) / 4096.0)-12.36); 
  gForceZ = ((accelZ)*9.81) / 4096.0;



  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

  rotX = ((((gyroX)*0.01745) / 65.5) - 10.05);
  rotY = ((((gyroX)*0.01745) / 65.5)-10.05); 
  rotZ = ((((gyroX)*0.01745) / 65.5)-10.05);

  const float inputlayer[6][1] PROGMEM ={{gForceX},{gForceY},{gForceZ},
                          {rotX},{rotY},{rotZ}};

  const float weights_1[4][6] PROGMEM= {{-0.40268838,0.5057685,0.24298422,0.9213788,-1.2766964,1.0552192},
                          {0.7286452,0.05126997,-0.18765673,-1.603684,-0.2248919 ,0.70605636},
                          {0.13963784,-0.15635969,0.18554588,-2.3270693,-0.0118859,-1.1264457},
                          {0.26414385,0.19587377,-0.11894583,-1.886621,-0.11081804,0.10624936}};

  const float bias_1[4][1] PROGMEM= {{0.42785648},{1.4273088},{-1.1427981},{2.2791126}};




     float prediction[1][1];
     float hiddenlayer_1[4][1];
    //int r1,c1,r2,c2 = 4,6,4,1;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 1; ++j) {
         hiddenlayer_1[i][j] = 0;
      }
    }
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 1; ++j) {
         for (int k = 0; k < 6; ++k) {
             hiddenlayer_1[i][j] += (pgm_read_byte(&weights_1[i][k]) * inputlayer[k][j]);
        }
           hiddenlayer_1[i][j] = hiddenlayer_1[i][j] + pgm_read_byte(&bias_1[i][j]);
           hiddenlayer_1[i][j] = max(0,hiddenlayer_1[i][j]);
      }
    }
    
  
  const float weights_2[1][4] PROGMEM = {{1.8549712 ,-0.6043076,3.4437234,-0.40509257}};
  const float bias_2[1][1] PROGMEM = {{-1.0489434}};
    

    //int r1,c1,r2,c2 = 4,6,4,1;
    for (int i = 0; i < 1; ++i) {
      for (int j = 0; j < 1; ++j) {
         prediction[i][j] = 0;
      }
    }
    for (int i = 0; i < 1; ++i) {
      for (int j = 0; j < 1; ++j) {
         for (int k = 0; k < 4; ++k) {
           prediction[i][j] += (weights_2[i][k] * hiddenlayer_1[k][j]);
            
        }
           prediction[i][j] = prediction[i][j] + bias_2[i][j];
           prediction[i][j] = (1/(1+exp(-(prediction[i][j]))));
           predict = roundf(prediction[i][j]);
                
      }
    }
    
}



void setupMPU()
{
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


void notification(int predict)
{
  Serial.println(predict);

  if (predict==1)
  {
    display.clear();
    display.setFont(Adafruit5x7);
    display.setCursor(0,5);
    display.println(F("A Fall Has Occured"));
    display.println(F("A Message Would Be Sent"));
    display.println();
    display.println(F("If This Is An Error"));
    display.println(F("Press the hold down the push button to cancel for 5secs"));
    delay(3000);
    pushbutton_var = digitalRead(pushbutton);

  // const static char message1 [] PROGMEM= "Ward is in stable condition";
  // const static char message2 [] PROGMEM= "A fall has been detected!!! kindly attend to your ward";
  // const static char message3 [] PROGMEM= "Ward's Heartbeat reading is high, kindly check-up on them";
  // const static char message4 [] PROGMEM= "Ward's Temperature reading is high, kindly check-up on them";
  // const static char message5 [] PROGMEM= "Ward's Temperature and Heartbeat readings are high, kindly consult your medical practitioner!!!";

  // if (predict == 1 && pushbutton_var == 1)
  // {
  //   digitalWrite(LED13,HIGH);
  //   Blynk.virtualWrite(V0,message2);
  //   Blynk.logEvent("fall_detected", message2);
  // } 

  // else if (myBPM>=100)
  // {
  //   Blynk.virtualWrite(V3,message3);
  // }

  // else if (temp > 32)
  // {
  //   Blynk.virtualWrite(V4,message4);
  // }

  // else if (temp >32 && myBPM >= 100 )
  // {
  //   Blynk.virtualWrite(V5,message5);
  // }

  // else
  // {
  //   Blynk.virtualWrite(V0,message1);
  // }

 }
}
 

void myTimer() 
{
  // Blynk.virtualWrite(V1, temp);  
  // Blynk.virtualWrite(V2,myBPM);
  // notification(predict);  
}

void oleddisplay()
{
  if (a > 127)
  {
    display.clear();
     a = 0;
    lasta = a;
  }
  if (b-lastb >=10) {
    b=b-15;
  }

  lastb = b;
  lasta = a;
  display.setCursor(0, 50);
  display.write(3);
  display.print(F(" BPM:"));
  display.print(myBPM);
  display.print(F("   Temp:"));
  display.print(temp);
  display.print(F("C"));
  a++;
  count++;
}


int getBeatsPerMinute()
{
  int sensorValue = analogRead(pulseSensorPin);
  static unsigned long lastBeatTime = 0; // keep track of the last time a heartbeat was detected
  static int beatsPerMinute = 0;
  
  if (sensorValue > thresholdValue) { // check if a heartbeat is detected
    unsigned long currentTime = millis(); // get the current time
    if (lastBeatTime != 0) { // if this is not the first heartbeat
      unsigned long interval = currentTime - lastBeatTime; // calculate the time interval between heartbeats
      beatsPerMinute = 60000 / interval; // calculate the beats per minute
    }
    lastBeatTime = currentTime; // update the last heartbeat time
  }
  return beatsPerMinute;
}








 
