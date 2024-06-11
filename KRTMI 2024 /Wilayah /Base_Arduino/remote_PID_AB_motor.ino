#include <PS2X_lib.h>
PS2X ps2x;
#define Encoder_output_A 2 
#define Encoder_output_B 3 
#define Encoder_output_C 20
#define Encoder_output_D 21

volatile long Count_pulses = 0;
int posisi = 0;
unsigned long lastTime = 0;
const int TimeSampling = 30;

volatile long Count_pulses1 = 0;
int posisi1 = 0;
const int TimeSampling1 = 30; //ms
unsigned long lastTime1 = 0;

int pwmPin1 = 9;    // Pin 9 untuk PWM
int lpwmPin1 = 22;  // Pin 22 untuk LPWM
int rpwmPin1 = 24;  // Pin 24 untuk RPWM

int pwmPin2 = 10;
int lpwmPin2 = 26;
int rpwmPin2 = 28;

float kp = 0.0005;//0,2
float ki = 0.2;
float kd = 3;

int error = 0;
int last_error = 0;
int sum_error = 0; 
int motorSpeed = 0;

float kp1 = 0.0005;
float ki1 = 0.2;
float kd1 = 3;

int error1 = 0;
int last_error1 = 0;
int sum_error1 = 0; 
int motorSpeed1 = 0;

int sp;
int sp1;


//ps2
int errror;
byte vibrate = 0;

void setup()
{

  Serial.begin(115200);
  Serial.println("STRT");
  pinMode(pwmPin1, OUTPUT);
  pinMode(lpwmPin1, OUTPUT);
  pinMode(rpwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(lpwmPin2, OUTPUT);
  pinMode(rpwmPin2, OUTPUT);
  pinMode(Encoder_output_A,INPUT_PULLUP); 
  pinMode(Encoder_output_B,INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(Encoder_output_A),ISR_Kanan,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_D),ISR_Kiri,RISING);
  while (true) {
    errror = ps2x.config_gamepad(13, 11, 10, 12, true, true);
    Serial.println("Error = " + String(error));
    if (errror == 0) {
      break;
    }
    else if (errror == 1) {
      Serial.println("No controller found");
    }
    else if (errror > 1) {
      Serial.println("Controller found but not accepting commands");
    }
  }
  Serial.println("Found Controller, configured successful");
}
void loop() {

  if (errror == 1)
    return;

  else {
    ps2x.read_gamepad(false, vibrate);
    //=================MAJU=====================
    if (ps2x.Button(PSB_PAD_UP)) { 
      //posisi = Count_pulses/21.69426752;
  sp = 88;
  digitalWrite(lpwmPin1, HIGH);  
  digitalWrite(rpwmPin1, LOW);   
  //analogWrite(pwmPin1, 50);
  posisi = Count_pulses/21.69426752;
  unsigned long currentTime = millis();
  float deltaTime = currentTime - lastTime;
  if(deltaTime>=TimeSampling){
    float freqSignal = (float)Count_pulses/(deltaTime/1.0e3); 
    float rpm = abs(freqSignal * 60.0 / 250); //249,6 = p*N; p=step-up gear ratio; N=pulses per revolution
    lastTime = currentTime;
    Count_pulses = 0;
    //Serial.println(rpm);
    Count_pulses = 0;   
    error = sp - rpm;
    sum_error = sum_error + error;
    motorSpeed = abs((kp*error) + (ki*sum_error) + (error-last_error));
    if (motorSpeed > 255) motorSpeed =255;
    else if(motorSpeed < 0) motorSpeed = 0;
    analogWrite(pwmPin1, motorSpeed);
    last_error = error;
    //Serial.println(posisi);
  }

  sp1 = 88;
   digitalWrite (lpwmPin2,HIGH);
   digitalWrite (rpwmPin2,LOW);
   posisi1 = Count_pulses1/21.69426752;
   unsigned long currentTime1 = millis();
   float deltaTime1 = currentTime1 - lastTime1;
   if(deltaTime1>=TimeSampling1){
    float freqSignal1 = (float)Count_pulses1/(deltaTime1/1.0e3);
    float rpm1 = abs(freqSignal1 * 60.0 / 250); 
    lastTime1 = currentTime1;
    Count_pulses1 = 0;
    error1 = sp1 - rpm1;    
    sum_error1 = sum_error1 + error1;
    motorSpeed1 = abs((kp1*error1) + (ki1*sum_error1) + (error1-last_error1));
    if (motorSpeed1 > 255) motorSpeed1 =255;
    else if(motorSpeed1 < 0) motorSpeed1 = 0;  
    analogWrite(pwmPin2, motorSpeed1);
    last_error1 = error1;
    //Serial.println(rpm1); 
    }
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.println("RIGHT");
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.println("LEFT");
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.println("DOWN");
      //posisi = Count_pulses/21.69426752;
  sp = 88;
  digitalWrite(lpwmPin1, HIGH);  
  digitalWrite(rpwmPin1, LOW);   
  //analogWrite(pwmPin1, 50);
  posisi = Count_pulses/21.69426752;
  unsigned long currentTime = millis();
  float deltaTime = currentTime - lastTime;
  if(deltaTime>=TimeSampling){
    float freqSignal = (float)Count_pulses/(deltaTime/1.0e3); 
    float rpm = abs(freqSignal * 60.0 / 250); //249,6 = p*N; p=step-up gear ratio; N=pulses per revolution
    lastTime = currentTime;
    Count_pulses = 0;
    //Serial.println(rpm);
    Count_pulses = 0;   
    error = sp - rpm;
    sum_error = sum_error + error;
    motorSpeed = abs((kp*error) + (ki*sum_error) + (error-last_error));
    if (motorSpeed > 255) motorSpeed =255;
    else if(motorSpeed < 0) motorSpeed = 0;
    analogWrite(pwmPin1, motorSpeed);
    last_error = error;
    //Serial.println(posisi);
  }

  sp1 = 88;
   digitalWrite (lpwmPin2,HIGH);
   digitalWrite (rpwmPin2,LOW);
   posisi1 = Count_pulses1/21.69426752;
   unsigned long currentTime1 = millis();
   float deltaTime1 = currentTime1 - lastTime1;
   if(deltaTime1>=TimeSampling1){
    float freqSignal1 = (float)Count_pulses1/(deltaTime1/1.0e3);
    float rpm1 = abs(freqSignal1 * 60.0 / 250); 
    lastTime1 = currentTime1;
    Count_pulses1 = 0;
    error1 = sp1 - rpm1;    
    sum_error1 = sum_error1 + error1;
    motorSpeed1 = abs((kp1*error1) + (ki1*sum_error1) + (error1-last_error1));
    if (motorSpeed1 > 255) motorSpeed1 =255;
    else if(motorSpeed1 < 0) motorSpeed1 = 0;  
    analogWrite(pwmPin2, motorSpeed1);
    last_error1 = error1;
    //Serial.println(rpm1); 
    }
    }
    if ( (ps2x.Analog(PSS_LY) < 55) && (ps2x.Analog(PSS_LX) > 98) && (ps2x.Analog(PSS_LX) < 158))
    {
      Serial.println("Maju");
    }
    if ( (ps2x.Analog(PSS_LY) > 200) && (ps2x.Analog(PSS_LX) > 98) && (ps2x.Analog(PSS_LX) < 158))
    {
      Serial.println("Mundur");
    }
    if ( (ps2x.Analog(PSS_LY) > 60) && (ps2x.Analog(PSS_LY) < 200) && (ps2x.Analog(PSS_LX) > 200))
    {
      Serial.println("Kanan");
    }
    if ( (ps2x.Analog(PSS_LY) > 60) && (ps2x.Analog(PSS_LY) < 200) && (ps2x.Analog(PSS_LX) < 55))
    {
      Serial.println("Kiri");
    }
     if (ps2x.Button(PSB_CROSS))
    {
      Serial.println("kali");
    }
    if (ps2x.Button(PSB_CIRCLE))
    {
      Serial.println("bulat");
    }
     if (ps2x.Button(PSB_TRIANGLE))
    {
      Serial.println("segitiga");
    }
    if (ps2x.Button(PSB_SQUARE))
    {
      Serial.println("kotak");
    }
    if (ps2x.Button(PSB_L1))
    {
      Serial.println("L1");
    }
    if (ps2x.Button(PSB_L2))
    {
      Serial.println("L2");
    }
    if (ps2x.Button(PSB_R1))
    {
      Serial.println("R1");
    }
    if (ps2x.Button(PSB_R2))
    {
      Serial.println("R2");
    }
  }
}

void ISR_Kanan(){
  int b = digitalRead(Encoder_output_B);
  if(b > 0){
    Count_pulses++;
  }
  else{
    Count_pulses--;
  }
}

void ISR_Kiri(){
  int d = digitalRead(Encoder_output_C);
  if(d > 0){
    Count_pulses1++;
  }
  else{
    Count_pulses1--;
  }
}
