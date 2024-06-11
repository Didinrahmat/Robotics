#include <Arduino.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

BluetoothSerial serialBT;
char cmd;
int fw, bw, left, right, stop;
int rx;

int pwm;

// Pin untuk motor 1
const int motor1Pin1 = 26; // Pin IN1 dari motor driver untuk motor 1
const int motor1Pin2 = 27; // Pin IN2 dari motor driver untuk motor 1
const int PWMA = 18;

// Pin untuk motor 2
const int motor2Pin1 = 14; // Pin IN1 dari motor driver untuk motor 2
const int motor2Pin2 = 12; // Pin IN2 dari motor driver untuk motor 2
const int PWMB = 17;

Servo myservo;
Servo myservo1;  // create servo object to control a servo
int pos = 0;

int servoPin = 13;
int servoPin1 = 21;


void notify(void){
  if(cmd == 'F'){ 
   fw = 1;
  Serial.println("F");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(PWMA, 225);
  analogWrite(PWMB, 225);

}

  else fw = 0;

  if(cmd == 'B'){ 
    bw = 1; 
    Serial.println("B");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(PWMA, 225);
  analogWrite(PWMB, 225);
    }
  else bw = 0;

  if(cmd == 'R'){
    right = 1;
    Serial.println("R");
      Serial.println("F");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(PWMA, 140);
  analogWrite(PWMB, 140);
  }
  else right = 0;

  if(cmd == 'L'){
    left = 1;
    Serial.println("L");
      Serial.println("F");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(PWMA, 140);
  analogWrite(PWMB, 140);
  }

  else left = 0;

  if(cmd == 'S') {
  stop = 1;
  Serial.println("S"); 
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);}
  else stop = 0;

  if(cmd == 'W') {
  Serial.println("W"); 
  stop = 1;

    myservo.write(180);    // tell servo to go to position in variable 'pos'
    delay(900) ;
         
      }
  else stop = 0;

  if(cmd == 'w') {
  Serial.println("w"); 
  stop = 1;

    myservo.write(0);    // tell servo to go to position in variable 'pos'
    delay(900) ;
         
      }
  else stop = 0;

  if(cmd == 'U') {
  Serial.println("U"); 
  stop = 1;
    myservo1.write(-20);    // tell servo to go to position in variable 'pos'
    delay(800) ;             // waits 15ms for the servo to reach the position
      }
  else stop = 0;

  if(cmd == 'u') {
  Serial.println("u"); 
  stop = 1;
  myservo1.write(350);    // tell servo to go to position in variable 'pos'
    delay(800);             // waits 15ms for the servo to reach the position
              // waits 15ms for the servo to reach the position
      }
  else stop = 0;
}

void setup() {
  Serial.begin(9600);
  serialBT.begin("Esp32-BT");

  pinMode(2, OUTPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

    // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // 
  myservo1.setPeriodHertz(50);    // standard 50 hz servo
  myservo1.attach(servoPin1, 1000, 2000); // 
}


void loop() {
  notify();
  if(serialBT.available()){
  cmd = serialBT.read();
  }


  }