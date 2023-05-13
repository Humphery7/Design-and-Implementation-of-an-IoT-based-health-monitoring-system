

// /*
//   ReadAnalogVoltage

//   Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
//   Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
//   Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

//   This example code is in the public domain.

//   https://www.arduino.cc/en/Tutorial/BuiltInExamples/ReadAnalogVoltage
// */

// int lm35PIN = A1;

// // the setup routine runs once when you press reset:
// void setup() {
//   // initialize serial communication at 9600 bits per second:
//   Serial.begin(9600);
// }

// // the loop routine runs over and over again forever:
// void loop() {
//   // read the input on analog pin 0:
//   int sensorValue = analogRead(lm35PIN);
//   Serial.println(sensorValue);
//   // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
//   float voltage = (sensorValue * 5.0) / 1023.0;
//   float temperature = voltage * 100;
//   // print out the value you read:
//   // Serial.println(temperature);
//   delay(2000);
// }


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
	Serial.begin(9600);

	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  
	delay(100);
}

void loop() {
	/* Get new sensor events with the readings */
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
  a.acceleration.x = a.acceleration.x + 9.09;
  a.acceleration.y = a.acceleration.y + 0.04;
  a.acceleration.z = a.acceleration.z + 0.30;
  g.gyro.x = g.gyro.x;
  g.gyro.y = g.gyro.y;
  g.gyro.z = g.gyro.z;
	/* Print out the values */
	Serial.print("Acceleration X: ");
	Serial.println(a.acceleration.x);
	Serial.print(", Y: ");
	Serial.println(a.acceleration.y);
	Serial.print(", Z: ");
	Serial.println(a.acceleration.z);
	Serial.println(" m/s^2");

	Serial.print("Rotation X: ");
	Serial.println(g.gyro.x);
	Serial.print(", Y: ");
	Serial.println(g.gyro.y);
	Serial.print(", Z: ");
	Serial.println(g.gyro.z);
	Serial.println(" rad/s");

	Serial.print("Temperature: ");
	Serial.println(temp.temperature);
	Serial.println(" degC");

	Serial.println("");
 
  float inputlayer[6][1]={{a.acceleration.x},{a.acceleration.y},{a.acceleration.z},
                        {g.gyro.x},{g.gyro.y},{g.gyro.z}};

  float weights_1[4][6] = {{-0.40268838,0.5057685,0.24298422,0.9213788,-1.2766964,1.0552192},
                          {0.7286452,0.05126997,-0.18765673,-1.603684,-0.2248919 ,0.70605636},
                          {0.13963784,-0.15635969,0.18554588,-2.3270693,-0.0118859,-1.1264457},
                          {0.26414385,0.19587377,-0.11894583,-1.886621,-0.11081804,0.10624936}};

  float bias_1[4][1]= {{0.42785648},{1.4273088},{-1.1427981},{2.2791126}};
  float prediction[0][0];
  float hiddenlayer_1[4][1];
  //int r1,c1,r2,c2 = 4,6,4,1;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 1; ++j) {
        hiddenlayer_1[i][j] = 0;
      //  Serial.println("hiddenlayer 1 is running");
    }
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 1; ++j) {
        for (int k = 0; k < 6; ++k) {
          hiddenlayer_1[i][j] += (weights_1[i][k] * inputlayer[k][j]);
          // Serial.println("hiddenlayer1 compute is running");
      }
          hiddenlayer_1[i][j] = hiddenlayer_1[i][j] + bias_1[i][j];
          hiddenlayer_1[i][j] = max(0,hiddenlayer_1[i][j]);
        // Serial.println(hiddenlayer_1[i][j]);
    }
  }
  
  float weights_2[1][4] = {{1.8549712 ,-0.6043076,3.4437234,-0.40509257}};
  float bias_2[1][1] = {{-1.0489434}};
 
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
          
    }
  }
  Serial.println(roundf(prediction[0][0]));
	delay(3000);
}
