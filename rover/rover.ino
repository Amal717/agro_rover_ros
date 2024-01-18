#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>


Servo camera;

int Xaxis=127;
int Yaxis=127;
int trigL;
int trigR;

ros::NodeHandle nh;

void X_axis_callback(const std_msgs::Int32& msg) {
  Xaxis = msg.data;
}

void Y_axis_callback(const std_msgs::Int32& msg) {
  Yaxis = msg.data;
}

void trigLeft_callback(const std_msgs::Int32& msg) {
  trigL = msg.data;
}

void trigRight_callback(const std_msgs::Int32& msg) {
  trigR = msg.data;
}

ros::Subscriber<std_msgs::Int32> X_axis_subscriber("X_axis_command", &X_axis_callback);
ros::Subscriber<std_msgs::Int32> Y_axis_subscriber("Y_axis_command", &Y_axis_callback);
ros::Subscriber<std_msgs::Int32> triggerR_subscriber("TriggerR_command", &trigRight_callback);
ros::Subscriber<std_msgs::Int32> triggerL_subscriber("TriggerL_command", &trigLeft_callback);

void setup() {
  nh.initNode();
  nh.subscribe(X_axis_subscriber);
  nh.subscribe(Y_axis_subscriber);
  nh.subscribe(triggerR_subscriber);
  nh.subscribe(triggerL_subscriber);

  initialize_motor();

  camera.attach(3);  // Assuming servo is connected to pin 3
  camera.write(90);

  Serial.begin(57600);
}

void loop() {
  nh.spinOnce();
  control_rover(Xaxis, Yaxis);
  adjust_camera();
}

void adjust_camera() {
  int valueL = map(trigL, 0, 255, 90, 0);
  int valueR = map(trigR, 0, 255, 90, 180);

  if (trigR < 10) {
    for (int i = 90; i > valueL; i--) {
      camera.write(i);
      delay(15);
    }
  }
  if (trigL < 10) {
    for (int i = 90; i < valueR; i++) {
      camera.write(i);
      delay(15);
    }
  }
}
