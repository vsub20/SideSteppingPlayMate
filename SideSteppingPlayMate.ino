#include <Servo.h>  // import servo library
#define RECV_PIN  12    // Infrared receiver module
#define ECHO_PIN  A4  //ultrasonic sensor
#define TRIG_PIN  A5 //ultrasonic sensor
#define ENA 6   // L298N Motor
#define ENB 5   // L298N Motor
#define IN1 7   // L298N Motor
#define IN2 8   // L298N Motor
#define IN3 9   // L298N Motor
#define IN4 11  // L298N Motor
#define carSpeed 250  // set car speed
Servo servo;        // servo object initialization
boolean PreviousState = LOW;    //used for Feather Spin to update motion state
const byte PIR = 2; //pir pin initialization
long randNumber;  // provides random number for servo motor to turn
int frontDistance = 0;  //used in obstacle avoidance to get distance
 
//initialization of pins
void setup() {
  Serial.begin(9600);
  servo.attach(3,500,2400);// 500: 0 degree  2400: 180 degree
  servo.write(90);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(PIR, INPUT);
}
 
// indication for bluetooth app for mode changes
enum FUNCTIONMODE{
  IDLE,     //stays still,no movement
  FeatherSpin,  
  ObstaclesAvoidance,
  Bluetooth,
  
} func_mode = IDLE;
// motor controls for the car
enum MOTIONMODE {
  STOP,
  FORWARD,
  BACK,
  LEFT,
  RIGHT
} mov_mode = STOP;
// calculation for delay slot
void delays(unsigned long t) {
  for(unsigned long i = 0; i < t; i++) {
    getBTData();
    delay(1);
  }
}
 
// getDistance is used in obstacle avoidance to calculate distance in front of sensor
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return (int)pulseIn(ECHO_PIN, HIGH) / 58;
}
 
// function to tell car to move forward
void forward(bool debug = false){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  if(debug) Serial.println("Go forward!");
}
 
// function to tell car to move back
void back(bool debug = false){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  if(debug) Serial.println("Go back!");
}
 
// function to tell car to turn left
void left(bool debug = false){
  analogWrite(ENA,carSpeed);
  analogWrite(ENB,carSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
  if(debug) Serial.println("Go left!");
}
 
// function to tell car to turn right
void right(bool debug = false){
  analogWrite(ENA,carSpeed);
  analogWrite(ENB,carSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  if(debug) Serial.println("Go right!");
}
 
// function to tell car to go forward in autonomous mode
void forwardavoid(bool debug = false){ 
  analogWrite(ENA, carSpeed/2);
  analogWrite(ENB, carSpeed/2);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  if(debug) Serial.println("Go forward!");
}
 
// function to tell car to go backwards in autonomous mode
void backavoid(bool debug = false){
  analogWrite(ENA, carSpeed/2);
  analogWrite(ENB, carSpeed/2);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  if(debug) Serial.println("Go back!");
}
 
// function to tell car to turn right a couple degrees in autonomous mode
void rightavoid(bool debug = false){
  analogWrite(ENA,carSpeed/2.5);
  analogWrite(ENB,carSpeed/2.5);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  if(debug) Serial.println("Go right!");
}
 
// function to tell car to stop whichever mode it's in
void stop(bool debug = false){
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  if(debug) Serial.println("Stop!");
}
 
//function to fetch bluetooth data indicating mode selection
void getBTData() {
  if(Serial.available()) {
    switch(Serial.read()) {
      case 'f': func_mode = Bluetooth; mov_mode = FORWARD;  break;
      case 'b': func_mode = Bluetooth; mov_mode = BACK;     break;
      case 'l': func_mode = Bluetooth; mov_mode = LEFT;     break;
      case 'r': func_mode = Bluetooth; mov_mode = RIGHT;    break;
      case 's': func_mode = Bluetooth; mov_mode = STOP;     break;
      case '2': func_mode = ObstaclesAvoidance;             break;
      case '3': func_mode = FeatherSpin;                    break;
      default:  break;
    } 
  }
}
 
// functions for move mode in bluetooth module
void bluetooth_mode() {
  if(func_mode == Bluetooth){
    switch(mov_mode){
      case FORWARD: forward();  break;
      case BACK:    back();     break;
      case LEFT:    left();     break;
      case RIGHT:   right();    break;
      case STOP:    stop();     break;
      default: break;
    }
  }
}
 
// function to make wand turn at a random angle if motion sensor is triggered
void Feather_Spin(){
if(func_mode == FeatherSpin){
    delays(500);
randNumber = random (10, 180);
 boolean state = digitalRead(PIR);
servo.write (90);
  if (state != PreviousState) {
    PreviousState = state;
    if (state == HIGH) {
      if (servo.read() == 90) {
        servo.write (randNumber);
      } else {
        servo.write (90);
      }
    }
  }
}
}
 
// function for autonomously avoiding obstacles in a confined space to interact with the cat
void obstacles_avoidance_mode() {
  if(func_mode == ObstaclesAvoidance){
    delays(500);
    frontDistance = getDistance();
    if(frontDistance <= 50) {
      stop();
      delays(500);
      backavoid();
      delays(500);
      rightavoid();
      delays(500);
      forwardavoid();
    }else {
        forwardavoid();
  }
  }
}
 
// main function
void loop() {
  getBTData();
  bluetooth_mode();
  obstacles_avoidance_mode();
  Feather_Spin();
}
