/*
* SAUSS-ware embedded software
* This is an OSU ECE Capstone project.
* Team member: Tony Han, Dominic Oliverio, Colin Lee, Doug Van Arsdale, and Molly Loughridge
 */

#include <DFRobot_LIS2DW12.h>
#include <DFRobot_LIS2DH12.h>
#include <DFRobot_LIS.h>
#include <stdio.h>
#include <string.h>
DFRobot_H3LIS200DL_I2C acce; //Instantiate accelerometer object
#define TWO_SECOND 176
#define THIRTY_SECOND 2658
#define TRUE 1
#define FALSE 0

/*Accelerometer Configuration*/
#if defined(ESP32) || defined(ESP8266)
#define H3LIS200DL_CS  D3
#elif defined(__AVR__) || defined(ARDUINO_SAM_ZERO) //Using ATMega328p here, which is an AVR controller.
#define H3LIS200DL_CS 3
#elif defined (NRF5)
#define H3LIS200DL_CS 2  //The pin on the development board with the corresponding silkscreen printed as P2 
#endif

float peak = 0;
int peak_val_loop_counter = 0; //peak value clearing counter
int bt_loop_counter = 0; //bluetooth communication counter
int detect_counter = 0; //impact detection counter
int detected = 0; 

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200); //set baud rate to 115200 for BLE communication
  while (!acce.begin()) {
	  Serial.println("I2C initialization failed.");
	  delay(1000);
  }
  Serial.print("Chip ID=");
  Serial.println(acce.getID(), HEX);
  acce.setRange(DFRobot_LIS::eH3lis200dl_200g); //set reading range to 200g
  acce.setAcquireRate(DFRobot_LIS::eNormal_100HZ); //set refresh rate to 100Hz
}

// the loop function runs over and over again forever
void loop() {
  int32_t ax, ay, az;
  ax = acce.readAccX();
  ay = acce.readAccY();
  az = acce.readAccZ();
  float acc = sqrt(ax * ax + ay * ay + az * az); //calculate 3D acceleration based on XYZ axis
  if (acc > 10 && detect_counter == 0 ) {
	  detected = TRUE;
  }
  if (acc > peak) {
	  peak = acc;
	  peak_val_loop_counter = 0;
  }
  else {
	  peak_val_loop_counter++;
  }
  peak = peak_val_loop_counter >= TWO_SECOND ? 0 : peak; //clear previous peak recording after 2s cycles
  detected ? bt_loop_counter = 0 : bt_loop_counter; //If impact detected, sent data package to smartphone app immediately

  if (bt_loop_counter == 0) { //sent to bluetooth every 30s if no impact is detected
	  String acc_str = " " + String(acc);
	  String peak_str = " " + String(peak);
	  const char* acc_ch = acc_str.c_str(); //convert C++ String to C str
	  const char* peak_ch = peak_str.c_str();
	  /*---------------------------------------------------
		                     Note
		 IF you want to send data string to BLE serial, 
		 use Serial.write() instead of Serial.print()!
		 Only C string is supported!!
	   --------------------------------------------------*/
	  if (detected) {
		  detect_counter = 100;
		  Serial.write("IMPACT\n\0");
		  detected = FALSE;
	  }
	  else {
		  //const char* msg = ("3D_Acc" + acc_str + "Peak=" + peak_str).c_str();
		  Serial.write("3D_Acc=\0"); //Send data string to BLE
		  Serial.write(acc_ch);
		  Serial.write(" Peak=\0");
		  Serial.write(peak_ch);
		  Serial.write("\n");
	  }
	  bt_loop_counter = THIRTY_SECOND; //Send BT packages every ~30s, immediately if impact detected.
  }
  detect_counter > 0 ? detect_counter-- : detect_counter;
  bt_loop_counter--;
  delay(10); //Set loop frequency to ~100Hz
}
