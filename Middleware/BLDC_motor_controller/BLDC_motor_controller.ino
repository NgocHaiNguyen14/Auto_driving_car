const int INT_SPEED_1 = 22;
const int alarm_out_1 = 24; //input in controller
const int speed_out_1 = 26; //input in controller
const int alarm_reset_1 = 28;
const int rotation_1 = 30; 
const int brk_pulse_in_1 = 32;
const int start_stop_1 = 34;
const int pulse_out_1 = 2;

/* Motor 2 - right rear motor */
const int INT_SPEED_2 = 23;
const int alarm_out_2 = 25; //input in controller
const int speed_out_2 = 27; //input in controller
const int alarm_reset_2 = 29;
const int rotation_2 = 31; 
const int brk_pulse_in_2 = 33;
const int start_stop_2 = 35;
const int pulse_out_2 = 3;

/* Motor 3 - left front motor */
const int INT_SPEED_3 = 36;
const int alarm_out_3 = 38; //input in controller
const int speed_out_3 = 40; //input in controller
const int alarm_reset_3 = 42;
const int rotation_3 = 44; 
const int brk_pulse_in_3 = 46;
const int start_stop_3 = 48;
const int pulse_out_3 = 4;

/* Motor 4 - right front motor */
const int INT_SPEED_4 = 37;
const int alarm_out_4 = 39; //input in controller
const int speed_out_4 = 41; //input in controller
const int alarm_reset_4 = 43;
const int rotation_4 = 45; 
const int brk_pulse_in_4 = 47;
const int start_stop_4 = 49;
const int pulse_out_4 = 5;

void setup() {
  /* * Set up pinMode * */
  pinMode(INT_SPEED_1,OUTPUT); //input in the driver
  pinMode(alarm_out_1,INPUT); //output in the driver
  pinMode(speed_out_1,INPUT); //output in the driver
  pinMode(alarm_reset_1,OUTPUT); //input in the driver
  pinMode(rotation_1,OUTPUT); //input in the driver
  pinMode(brk_pulse_in_1,OUTPUT); //input in the driver
  pinMode(start_stop_1,OUTPUT); //input in the driver
  pinMode(pulse_out_1,OUTPUT); //input in the driver

  pinMode(INT_SPEED_2,OUTPUT); //input in the driver
  pinMode(alarm_out_2,INPUT); //output in the driver
  pinMode(speed_out_2,INPUT); //output in the driver
  pinMode(alarm_reset_2,OUTPUT); //input in the driver
  pinMode(rotation_2,OUTPUT); //input in the driver
  pinMode(brk_pulse_in_2,OUTPUT); //input in the driver
  pinMode(start_stop_2,OUTPUT); //input in the driver
  pinMode(pulse_out_2,OUTPUT); //input in the driver

  pinMode(INT_SPEED_3,OUTPUT); //input in the driver
  pinMode(alarm_out_3,INPUT); //output in the driver
  pinMode(speed_out_3,INPUT); //output in the driver
  pinMode(alarm_reset_3,OUTPUT); //input in the driver
  pinMode(rotation_3,OUTPUT); //input in the driver
  pinMode(brk_pulse_in_3,OUTPUT); //input in the driver
  pinMode(start_stop_3,OUTPUT); //input in the driver
  pinMode(pulse_out_3,OUTPUT); //input in the driver

  pinMode(INT_SPEED_4,OUTPUT); //input in the driver
  pinMode(alarm_out_4,INPUT); //output in the driver
  pinMode(speed_out_4,INPUT); //output in the driver
  pinMode(alarm_reset_4,OUTPUT); //input in the driver
  pinMode(rotation_4,OUTPUT); //input in the driver
  pinMode(brk_pulse_in_4,OUTPUT); //input in the driver
  pinMode(start_stop_4,OUTPUT); //input in the driver
  pinMode(pulse_out_4,OUTPUT); //input in the driver
  
  /* * Set up appropriate output of each port for controlling * */
  
  digitalWrite(INT_SPEED_1, LOW); // Set speed using external volume
  digitalWrite(rotation_1, LOW); // set the motor rotation is clockwise
  digitalWrite(brk_pulse_in_1, LOW); // int the dataset say ON(L), it may be LOW, just try and fix
  digitalWrite(start_stop_1, LOW); // the same with RUN/BRAKE

  digitalWrite(INT_SPEED_2, LOW); // Set speed using external volume
  digitalWrite(rotation_2, LOW); // set the motor rotation is clockwise
  digitalWrite(brk_pulse_in_2, LOW); // int the dataset say ON(L), it may be LOW, just try and fix
  digitalWrite(start_stop_2, LOW); // the same with RUN/BRAKE

  digitalWrite(INT_SPEED_3, LOW); // Set speed using external volume
  digitalWrite(rotation_3, LOW); // set the motor rotation is clockwise
  digitalWrite(brk_pulse_in_3, LOW); // int the dataset say ON(L), it may be LOW, just try and fix
  digitalWrite(start_stop_3, LOW); // the same with RUN/BRAKE

  digitalWrite(INT_SPEED_4, LOW); // Set speed using external volume
  digitalWrite(rotation_4, LOW); // set the motor rotation is clockwise
  digitalWrite(brk_pulse_in_4, LOW); // int the dataset say ON(L), it may be LOW, just try and fix
  digitalWrite(start_stop_4, LOW); // the same with RUN/BRAKE
}

/* * Run withh variable v (velocity) (from 0-255) * */
void run_motor_1(int v){
  digitalWrite(INT_SPEED_1, LOW); 
  digitalWrite(rotation_1, LOW); 
  digitalWrite(brk_pulse_in_1, LOW);
  digitalWrite(start_stop_1, LOW);
  analogWrite(pulse_out_1,v);
}

void run_motor_2(int v){
  digitalWrite(INT_SPEED_2, LOW); 
  digitalWrite(rotation_2, LOW);
  digitalWrite(brk_pulse_in_2, LOW); 
  digitalWrite(start_stop_2, LOW); 
  analogWrite(pulse_out_2,v);
}

void run_motor_3(int v){
  digitalWrite(INT_SPEED_3, LOW); 
  digitalWrite(rotation_3, LOW); 
  digitalWrite(brk_pulse_in_3, LOW); 
  digitalWrite(start_stop_3, LOW); 
  analogWrite(pulse_out_3,v);
}

void run_motor_4(int v){
  digitalWrite(INT_SPEED_4, LOW); 
  digitalWrite(rotation_4, LOW); 
  digitalWrite(brk_pulse_in_4, LOW); 
  digitalWrite(start_stop_4, LOW); 
  analogWrite(pulse_out_4,v);
}

/* ** Stop with brake * */

void stop_brake(){ // need to be checked later
  digitalWrite(brk_pulse_in_1, HIGH);
  digitalWrite(start_stop_1, LOW);
  //
  analogWrite(pulse_out_1,0);
  digitalWrite(brk_pulse_in_2, HIGH); 
  digitalWrite(start_stop_2, LOW); 
  //analogWrite(pulse_out_2,0);
  digitalWrite(brk_pulse_in_3, HIGH); 
  digitalWrite(start_stop_3, LOW); 
  //analogWrite(pulse_out_3,0);
  digitalWrite(brk_pulse_in_4, HIGH); 
  digitalWrite(start_stop_4, LOW); 
  //analogWrite(pulse_out_4,0);
}

void stop_without_brake(){
  digitalWrite(brk_pulse_in_1, LOW);
  digitalWrite(start_stop_1, HIGH);
  analogWrite(pulse_out_1,0);
  digitalWrite(brk_pulse_in_2, LOW); 
  digitalWrite(start_stop_2, HIGH); 
  analogWrite(pulse_out_2,0);
  digitalWrite(brk_pulse_in_3, LOW); 
  digitalWrite(start_stop_3, HIGH); 
  analogWrite(pulse_out_3,0);
  digitalWrite(brk_pulse_in_4, LOW); 
  digitalWrite(start_stop_4, HIGH); 
  analogWrite(pulse_out_4,0);
}

void loop() {
  /*run_motor_1(70);
  run_motor_2(70);
  run_motor_3(70);
  run_motor_4(70);
  delay(5000);
  stop_without_brake();
  delay(5000);*/
  int a = 255;
  run_motor_1(a);
  run_motor_2(a);
  run_motor_3(a);
  run_motor_4(a);
  delay(10000);
  //stop_without_brake();
  delay(5000);
}
