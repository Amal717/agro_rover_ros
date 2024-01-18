#define IN1 4
#define IN2 5
#define ENA 6

#define IN3 8
#define IN4 9
#define ENB 10

int speedA;
int speedB;

void initialize_motor() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void speed() {
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void move_forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void move_backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void move_left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,HIGH);
}

void move_right(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4,LOW);
}

void stop_rover() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  speedA = 0;
  speedB = 0;
}

void control_rover(int Xaxis, int Yaxis) {
  if (Yaxis > 127) {
    // Move forward
    move_forward();
    speedA = map(Yaxis, 127, 255, 0, 255);
    speedB = map(Yaxis, 127, 255, 0, 255);
    // Apply speed
    speed();
  } else if (Yaxis < 127) {
    // Move backward
    move_backward();
    speedA = map(Yaxis, 127, 0, 0, 255);
    speedB = map(Yaxis, 127, 0, 0, 255);
    // Apply speed
    speed();
  }
   else if(Yaxis == 127){
    // Stop
    stop_rover();
  }
  if(Xaxis > 127){
    speedA = map(Xaxis, 127, 255, 0, 255);
    speedB = map(Xaxis, 127, 255, 0, 255);
    move_right();
    // Apply speed
    speed();
  }
  else if(Xaxis < 127){
    speedA = map(Xaxis, 127, 0, 0, 255);
    speedB = map(Xaxis, 127, 0, 0, 255);
    move_left();
    // Apply speed
    speed();
  }
  else if(Xaxis == 127){
    stop_rover();
  }

  
}

