#define USE_USBCON //Eğer leonardo kullanmıyorsanız USE_USBCON satırına gerek yoktur. 
#include "ros.h"
#include "battery_from_arduino_msgs/BatteryStatusFromArduino.h"

ros::NodeHandle nh;

battery_from_arduino_msgs::BatteryStatusFromArduino battery_state_from_arduino; // Doğru türü kullan
ros::Publisher pub("battery_state_from_arduino", &battery_state_from_arduino);

void setup() {
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
  battery_state_from_arduino.voltage = 10;
  battery_state_from_arduino.current = 20;
  battery_state_from_arduino.temperature = 30;
  battery_state_from_arduino.capacity = 40;

  pub.publish(&battery_state_from_arduino);

  nh.spinOnce();
  delay(500);
}
