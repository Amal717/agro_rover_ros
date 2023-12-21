#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define IN1 4
#define IN2 5
#define ENA 6

#define IN3 8
#define IN4 9
#define ENB 10

// X-axis Values
#define min_x 0
#define mid_x 128
#define max_x 255


#define min_y 0
#define mid_y 128
#define max_y 255

#define min_speed 70

#define tempSensor A0
#define moistSensor A1

#define servopin 3

Servo camera;

int Xaxis;
int Yaxis;
int speedA;
int speedB;
int trigL;
int trigR;

ros::NodeHandle nh;

void X_axis_callback(const std_msgs::Int32& msg) {
  Xaxis = msg.data;
  }
void Y_axis_callback(const std_msgs::Int32& msg) {
  Yaxis = msg.data;
  }

void trigLeft_callback(const std_msgs::Int32& msg){
  trigL = msg.data;
}

void trigRight_callback(const std_msgs::Int32& msg){
  trigR = msg.data;
}

ros::Subscriber<std_msgs::Int32> X_axis_subscriber("X_axis_command", &X_axis_callback);
ros::Subscriber<std_msgs::Int32> Y_axis_subscriber("Y_axis_command", &Y_axis_callback);
ros::Subscriber<std_msgs::Int32> triggerR_subscriber("TriggerR_command", &trigRight_callback);
ros::Subscriber<std_msgs::Int32> triggerL_subscriber("TriggerL_command", &trigLeft_callback);

std_msgs::Float32 temp_msg;
std_msgs::Float32 moist_msg;

ros::Publisher temp_publisher("/temp", &temp_msg);
ros::Publisher moist_publisher("/moist", &moist_msg);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(X_axis_subscriber);
  nh.subscribe(Y_axis_subscriber);
  nh.subscribe(triggerR_subscriber);
  nh.subscribe(triggerL_subscriber);
  
  nh.advertise(temp_publisher);
  nh.advertise(moist_publisher);

  initialize_motor();
  camera.attach(servopin);
  camera.write(90);
  Serial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
    nh.spinOnce();
    delay(10);
    control_rover();
    adjust_camera();

    float tempValue = readTemperatureSensor();
    float moistValue = readMoistureSensor();

    temp_msg.data = tempValue;
    moist_msg.data = moistValue;

    temp_publisher.publish(&temp_msg);
    moist_publisher.publish(&moist_msg);
  /*
  if(!Serial.available()){
    stop_rover();
    speed();
  }
  */
}


float readTemperatureSensor() {
   int value = analogRead(tempSensor);
   float temp = map(value, 0, 1023, 0, 100);
   return temp;
}

float readMoistureSensor() {
  int value = analogRead(moistSensor);
  float moist = map(value, 0, 1023, 0, 100);
  return moist;
}

void adjust_camera(){
  int valueL = map(trigL, 0, 255, 90, 0);
  int valueR = map(trigR, 0, 255, 90, 180);
  
  if(trigR < 10){
    for(int i = 90; i> valueL; i--){
      camera.write(i);
      delay(15);
    }
  }
  if(trigL < 10){
    for(int i = 90; i< valueR; i++){
      camera.write(i);
      delay(15);
    }
  }
}