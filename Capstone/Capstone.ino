/*
* SAUSS-ware embedded software
* This is an OSU ECE Capstone project.
* Team member: Tony Han, Dominic Oliverio, Colin Lee, Doug Van Arsdale, and Molly Loughridge
 */

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

float large = 0;
int peak_val_loop_ctrl;
int bt_loop_ctrl=20;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!acce.begin()) {
	  Serial.println("I2C initialization failed.");
	  delay(1000);
  }
  Serial.print("Chip ID=");
  Serial.println(acce.getID(), HEX);
  acce.setRange(DFRobot_LIS::eH3lis200dl_200g);
  acce.setAcquireRate(DFRobot_LIS::eNormal_100HZ);
}

// the loop function runs over and over again forever
void loop() {
  int32_t ax, ay, az;
  ax = acce.readAccX();
  ay = acce.readAccY();
  az = acce.readAccZ();
  float acc = sqrt(ax * ax + ay * ay + az * az);

  if (acc > large) {
	  large = acc;
	  peak_val_loop_ctrl = 0;
  }
  else {
	  peak_val_loop_ctrl++;
  }
  large = peak_val_loop_ctrl >= 100 ? 0 : large; //clear previous peak recording after 1s cycles

  if (bt_loop_ctrl == 0) { //sent to bluetooth every 200ms
	  String acc_str = String(acc);
	  String large_str = String(large);
	  const char* acc_ch = acc_str.c_str();
	  const char* large_ch = large_str.c_str();
	  Serial.write("3D_Acc=");
	  Serial.write(acc_ch);
	  Serial.write(" Peak=");
	  Serial.write(large_ch);
	  Serial.write("\n");
	  bt_loop_ctrl = 20;
  }
  bt_loop_ctrl--;
  delay(10);
}
