#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

const int ledPin5 = 5;  // Adjust the pin numbers based on your setup
const int ledPin6 = 6;

void x_axis_callback(const std_msgs::Int32& msg) {
  int brightness = msg.data;
  Serial.print("Received X-axis brightness level: ");
  Serial.println(brightness);
  update_led_brightness(ledPin5, brightness);
}

void Y_axis_callback(const std_msgs::Int32& msg) {
  int brightness = msg.data;
  Serial.print("Received Y-axis brightness level: ");
  Serial.println(brightness);
  update_led_brightness(ledPin6, brightness);
}

void update_led_brightness(int ledPin, int brightness) {
  // Ensure brightness is within a valid range (0 to 255)
  brightness = constrain(brightness, 0, 255);

  // Control the LED brightness using analogWrite
  analogWrite(ledPin, brightness);
}

ros::Subscriber<std_msgs::Int32> x_axis_subscriber("x_axis_command", &x_axis_callback);
ros::Subscriber<std_msgs::Int32> Y_axis_subscriber("Y_axis_command", &Y_axis_callback);

void setup() {
  nh.initNode();
  nh.subscribe(x_axis_subscriber);
  nh.subscribe(Y_axis_subscriber);

  pinMode(ledPin5, OUTPUT);
  pinMode(ledPin6, OUTPUT);

  Serial.begin(57600);  // Initialize serial communication for debugging
}

void loop() {
  nh.spinOnce();
  delay(10);
}
