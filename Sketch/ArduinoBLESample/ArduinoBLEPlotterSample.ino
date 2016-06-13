/*
===============================================
Example sketch for CurieIMU library for Intel(R) Curie(TM) devices.
Copyright (c) 2015 Intel Corporation.  All rights reserved.

Based on I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050
class by Jeff Rowberg: https://github.com/jrowberg/i2cdevlib

===============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================

Genuino 101 CurieIMU Orientation Visualiser
Hardware Required:
Arduino/Genuino 101

Modified Nov 2015
by Helena Bisby <support@arduino.cc>
This example code is in the public domain
http://arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
*/

#include <CurieBLE.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

int Racc[3], Rgyr[3];
float yaw, pitch, roll;
int calibrateOffsets = 1; // int to determine whether calibration takes place or not
int factor = 800; // variable by which to divide gyroscope values, used to control sensitivity
				  // note that an increased baud rate requires an increase in value of factor

const int pinLed = 3;                        
const int pinTouchSensor = 4;
const int pinPotentiometer = 0;
const int pinBLE = 13;

int potentiometerValue = 0;
int buttonPressed = 0;

// initialise Madgwick object
Madgwick filter; 

// BLE Peripheral Device (the board you're programming)
BLEPeripheral blePeripheral;  

// Curie Service
BLEService curieService("19B10000-E8F2-537E-4F6C-D104768A1214"); 

// Yaw Characteristic - custom 128-bit UUID, readonly by central
BLECharacteristic yprCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);

//BLECharacteristic accCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);

void setup() {
	// initialize Serial communication
	Serial.begin(9600);

	// set led OUTPUT
	pinMode(pinLed, OUTPUT);    
	pinMode(pinTouchSensor, INPUT);
	pinMode(pinPotentiometer, INPUT);

	// set advertised local name and service UUID:
	blePeripheral.setLocalName("Curie");
	blePeripheral.setAdvertisedServiceUuid(curieService.uuid());

	// add service and characteristic:
	blePeripheral.addAttribute(curieService);
	blePeripheral.addAttribute(yprCharacteristic);
	//blePeripheral.addAttribute(accCharacteristic);

	// set the initial value for the characeristics:
	yprCharacteristic.setValue((unsigned char*)"0,0,0,0,0,0", 1);
	//accCharacteristic.setValue((unsigned char*)"0,0,0", 1);
	
	// begin advertising BLE service:
	blePeripheral.begin();

	// initialize device
	CurieIMU.begin();

	calibrate();
}

void loop() {
	int val;
	int brightness, gyroSensativity;
	char gyroBytes[20], accBytes[20];

	// turn off the BLE LED
	digitalWrite(pinBLE, LOW);

	// Set the Sensativity LED to off.
	analogWrite(pinLed, LOW);

	// Read from the potentiometer.
	potentiometerValue = analogRead(pinPotentiometer);

	// Check to see if the button is pressed.
	buttonPressed = digitalRead(pinTouchSensor);

	// Convert Potentiometer values to brightness values 0-255.
	brightness = map(potentiometerValue, 0, 1023, 255, 0);

	// Convert Potentiometer values to gyro sensativity values 1-800.
	gyroSensativity = 800; //map(potentiometerValue, 0, 1023, 1, 800);

	// Adjust LED brightness if button is not pressed.
	if (buttonPressed == 0) {
		analogWrite(pinLed, brightness);
	}

	// read raw accel/gyro measurements from device
	CurieIMU.readMotionSensor(Racc[0], Racc[1], Racc[2], Rgyr[0], Rgyr[1], Rgyr[2]);

	// use function from MagdwickAHRS.h to return quaternions
	filter.updateIMU(Rgyr[0] / gyroSensativity, Rgyr[1] / gyroSensativity, Rgyr[2] / gyroSensativity, Racc[0], Racc[1], Racc[2]);

	// functions to find yaw roll and pitch from quaternions
	yaw = filter.getYaw();
	roll = filter.getRoll();
	pitch = filter.getPitch();
	
	// convert the raw accelerometer data to G's
	Racc[0] = convertRawAcceleration(Racc[0]);
	Racc[1] = convertRawAcceleration(Racc[1]);
	Racc[2] = convertRawAcceleration(Racc[2]);

	if (Serial.available() > 0) {
		val = Serial.read();

		if (val == 's') { // if incoming serial is "s"
			Serial.print(yaw);
			Serial.print(","); // print comma so values can be parsed
			Serial.print(pitch);
			Serial.print(","); // print comma so values can be parsed
			Serial.print(roll);
			Serial.print(","); // print comma so values can be parsed
			Serial.print(Racc[0]);
			Serial.print(","); // print comma so values can be parsed
			Serial.print(Racc[1]);
			Serial.print(","); // print comma so values can be parsed
			Serial.println(Racc[2]);
		}

		if (val == 'r') {
			calibrate();
		}
	}

	// listen for BLE peripherals to connect:
	BLECentral central = blePeripheral.central();

	// if a central is connected to peripheral:
	if (central) {
		// turn on the LED to indicate the connection:
		digitalWrite(pinBLE, HIGH);

		//Check the Gyro and Accelerometer as long as the central is still connected:
		if (central.connected()) {
			char* buff = new char[10];

			// Write Gyrometer values to a string and then convert to byte array.
			String gyroString = "";
			if (yaw < 0) {
				gyroString += "-";
			}
			dtostrf(abs(yaw), 4, 2, buff);  //4 is mininum width, 2 is precision
			gyroString += buff;
			gyroString += " ";
			if (pitch < 0) {
				gyroString += "-";
			}
			dtostrf(abs(pitch), 4, 2, buff);
			gyroString += buff;
			gyroString += " ";
			if (roll < 0) {
				gyroString += "-";
			}
			dtostrf(abs(roll), 4, 2, buff);
			gyroString += buff;
			gyroString.toCharArray(gyroBytes, 20);

			// Set Gyro values and send notification.
			yprCharacteristic.setValue((unsigned char*)gyroBytes, 20);

			// Write Accelerometer values to a string and then convert to a byte array.
			String accString = "";
			accString += Racc[0];
			accString += ",";
			accString += Racc[1];
			accString += ",";
			accString += Racc[2];
			accString.toCharArray(accBytes, 20);

			// Set Accelerometer values and send notification.
			//accCharacteristic.setValue((unsigned char*)accBytes, 20);

			// cleanup string buffer.
			delete buff;
		}
	}
}

float convertRawAcceleration(int aRaw) {
	// since we are using 2G range
	// -2g maps to a raw value of -32768
	// +2g maps to a raw value of 32768

	float a = (aRaw * 2.0) / 32768.0;

	return a;
}

void calibrate() {
	// Set the accelerometer range to 2G
	CurieIMU.setAccelerometerRange(2);
	CurieIMU.setGyroRange(250);

	if (calibrateOffsets == 1) {
		// use the code below to calibrate accel/gyro offset values
		Serial.println("Internal sensor offsets BEFORE calibration...");
		Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getGyroOffset(X_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getGyroOffset(Y_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getGyroOffset(Z_AXIS)); Serial.print("\t");
		Serial.println("");

		// To manually configure offset compensation values, use the following methods instead of the autoCalibrate...() methods below
		//    CurieIMU.setGyroOffset(X_AXIS, 220);
		//    CurieIMU.setGyroOffset(Y_AXIS, 76);
		//    CurieIMU.setGyroOffset(Z_AXIS, -85);
		//    CurieIMU.setAccelerometerOffset(X_AXIS, -76);
		//    CurieIMU.setAccelerometerOffset(Y_AXIS, -235);
		//    CurieIMU.setAccelerometerOffset(Z_AXIS, 168);

		//IMU device must be resting in a horizontal position for the following calibration procedure to work correctly!

		Serial.print("Starting Gyroscope calibration...");
		CurieIMU.autoCalibrateGyroOffset();
		Serial.println(" Done");
		Serial.print("Starting Acceleration calibration...");
		CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
		CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
		CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
		Serial.println(" Done");

		Serial.println("Internal sensor offsets AFTER calibration...");
		Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
		Serial.println("");
	}
}

