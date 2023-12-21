void initialize_motor(){
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
}
void speed(){
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void forward_speed(){
  speedA = map(Yaxis, mid_y, max_y, 0, 255);
  speedB = map(Yaxis, mid_y, max_y, 0, 255);
}

void move_forward(){
  //Move motor A forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  //Move motor B forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void backward_speed(){
  speedA = map(Yaxis, mid_y, min_y, 0, 255);
  speedB = map(Yaxis, mid_y, min_y, 0, 255);
}

void move_backward(){
  //Move motor B backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  //Move motor B backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  
}

void move_left(){
  move_forward(); 
  int x_left = map(Xaxis, min_x, mid_x, 0, 127);
  speedA = 128 - x_left;
  speedB = 128 + x_left;
  if(speedA < 0) speedA = 0;
  if(speedB > 255) speedB = 0;
}

void move_right(){
  move_forward();
  int x_right = map(Xaxis, mid_x, max_x, 0, 127);
  speedA = 128 + x_right;
  speedB = 128 - x_right;

  if(speedA > 255) speedA = 255;
  if(speedB < 0) speedB = 0;
}

void stop_rover(){
  //Move motor A forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  //Move motor B forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  speedA = 0;
  speedB = 0;
}

void keep_min_speed(){
  if(speedA < min_speed) speedA = 0;
  if(speedB < min_speed) speedB = 0;
}

void control_rover(){
  if(Yaxis > mid_y){
    move_forward();
    forward_speed();
    keep_min_speed();
    speed();
  }
  else if(Yaxis < mid_y){
    move_backward();
    backward_speed();
    keep_min_speed();
    speed();
  }
  else{
    stop_rover();
    speed();
  }

  if(Xaxis < mid_x){
    move_left();
    keep_min_speed();
    speed();
  }
  else if(Xaxis > mid_x){
    move_right();
    keep_min_speed();
    speed();
  }
  else{
    stop_rover();
    speed();
  }
}