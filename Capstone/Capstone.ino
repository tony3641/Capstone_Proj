/*
* SAUSS-ware embedded software
* This is an OSU ECE Capstone project.
* Team member: Tony Han, Dominic Oliverio, Colin Lee, Doug Van Arsdale, and Molly Loughridge
 */


// the setup function runs once when you press reset or power the board

#include <DFRobot_LIS2DW12.h>
#include <DFRobot_LIS2DH12.h>
#include <DFRobot_LIS.h>
DFRobot_H3LIS200DL_I2C acce;

#if defined(ESP32) || defined(ESP8266)
#define H3LIS200DL_CS  D3
#elif defined(__AVR__) || defined(ARDUINO_SAM_ZERO)
#define H3LIS200DL_CS 3
#elif (defined NRF5)
#define H3LIS200DL_CS 2  //The pin on the development board with the corresponding silkscreen printed as P2 
#endif

void setup() {
  Serial.begin(9600);
  while (!acce.begin()) {
	  Serial.println("I2C initialization failed.");
	  delay(1000);
  }
  Serial.print("Chip ID=");
  Serial.println(acce.getID(), HEX);
  acce.setRange(DFRobot_LIS::eH3lis200dl_200g);
  acce.setAcquireRate(DFRobot_LIS::eNormal_50HZ);
}

// the loop function runs over and over again forever
void loop() {
  long ax, ay, az;
  ax = acce.readAccX();
  ay = acce.readAccY();
  az = acce.readAccZ();
  Serial.print("X=");
  Serial.print(ax);
  Serial.print("Y=");
  Serial.print(ay);
  Serial.print("Z=");
  Serial.print(az);
  delay(50);
}
