#define Encoder_output_A 2
#define Encoder_output_B 3
#define Encoder_output_C 19
#define Encoder_output_D 18

volatile long Count_pulses = 0;
volatile long cnt = 0;
int posisi = 0;
unsigned long lastTime = 0;
const int TimeSampling = 30;

volatile long Count_pulses1 = 0;
volatile long cnt1 = 0;
int posisi1 = 0;
const int TimeSampling1 = 30; //ms
unsigned long lastTime1 = 0;

int pwmPin1 = 4;    // Pin 9 untuk PWM
int lpwmPin1 = 49;  // Pin 22 untuk LPWM
int rpwmPin1 = 47;  // Pin 24 untuk RPWM

int pwmPin2 = 8;
int lpwmPin2 = 45;
int rpwmPin2 = 43;

float kp = 0.6;//0,2
float ki = 0.2;
float kd = 3;

int error = 0;
int last_error = 0;
int sum_error = 0;
int motorSpeed = 0;

float kp1 = 0.6;
float ki1 = 0.2;
float kd1 = 3;

int error1 = 0;
int last_error1 = 0;
int sum_error1 = 0;
int motorSpeed1 = 0;

int sp;
int sp1;

//relay
const int relay = 53;         // Pin relay terhubung

void setup() {
  pinMode(pwmPin1, OUTPUT);
  pinMode(lpwmPin1, OUTPUT);
  pinMode(rpwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(lpwmPin2, OUTPUT);
  pinMode(rpwmPin2, OUTPUT);

  Serial.begin(9600); // activates the serial communication
  pinMode(Encoder_output_A, INPUT_PULLUP);
  pinMode(Encoder_output_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_A), ISR_Kanan, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_D), ISR_Kiri, RISING);

  pinMode(relay, OUTPUT);     // Atur pin relay sebagai output
  digitalWrite(relay, LOW);   // Pastikan relay mati pada awalnya

}

void loop() {
    Baca data yang diterima dari serial
   String receivedData = Serial.readString();
   if (receivedData == "go") {

      motor_kanan(1);
      motor_kiri(1);

      if (posisi <= -18) {
        digitalWrite(relay, HIGH);
        sp = 0;
        digitalWrite(lpwmPin1, LOW);
        digitalWrite(rpwmPin1, LOW);
        analogWrite(pwmPin1, 0);

        sp1 = 0;
        digitalWrite (lpwmPin2, LOW);
        digitalWrite (rpwmPin2, LOW);
        analogWrite(pwmPin2, 0);
        motorSpeed = 0;
        motorSpeed1 = 0;

        motor_kanan(0);
        motor_kiri(0);
        Serial.println("stop");

      }
      Serial.println(motorSpeed);
      Serial.println(rpm);
    

  

}

void motor_kanan(int kanan) {
  if (kanan == 1) {
    sp = 70;
    digitalWrite(lpwmPin1, HIGH);
    digitalWrite(rpwmPin1, LOW);
    //analogWrite(pwmPin1, 50);
    posisi = Count_pulses / 21.69426752;
    unsigned long currentTime = millis();
    float deltaTime = currentTime - lastTime;
    if (deltaTime >= TimeSampling) {
      float freqSignal = (float)cnt / (deltaTime / 1.0e3);
      float rpm = abs(freqSignal * 60.0 / 250); //249,6 = p*N; p=step-up gear ratio; N=pulses per revolution
      lastTime = currentTime;
      cnt = 0;
      Serial.println(Count_pulses);
      error = sp - rpm;
      sum_error = sum_error + error;
      motorSpeed = abs((kp * error) + (ki * sum_error) + (error - last_error));
      if (motorSpeed > 255) motorSpeed = 255;
      else if (motorSpeed < 0) motorSpeed = 0;
      analogWrite(pwmPin1, motorSpeed);
      last_error = error;
      //    Serial.print("posisiA: ");
      //    Serial.println(posisi);
    }
  }
  else {
    sp = 0;
    digitalWrite(lpwmPin1, LOW);
    digitalWrite(rpwmPin1, LOW);
    //analogWrite(pwmPin1, 50);
    posisi = Count_pulses / 21.69426752;
    unsigned long currentTime = millis();
    float deltaTime = currentTime - lastTime;
    if (deltaTime >= TimeSampling) {
      float freqSignal = (float)cnt / (deltaTime / 1.0e3);
      float rpm = abs(freqSignal * 60.0 / 250); //249,6 = p*N; p=step-up gear ratio; N=pulses per revolution
      lastTime = currentTime;
      cnt = 0;
//      Serial.println(rpm);
      error = sp - rpm;
      sum_error = sum_error + error;
      motorSpeed = 0;
      if (motorSpeed > 255) motorSpeed = 255;
      else if (motorSpeed < 0) motorSpeed = 0;
      analogWrite(pwmPin1, motorSpeed);
      last_error = error;
      //    Serial.print("posisiA: ");
      //    Serial.println(posisi);
    }
  }
}

void motor_kiri (int kiri) {
  if (kiri == 1) {
    sp1 = 70;
    digitalWrite (lpwmPin2, HIGH);
    digitalWrite (rpwmPin2, LOW);
    //posisi1 = Count_pulses1 / 21.69426752;
    unsigned long currentTime1 = millis();
    float deltaTime1 = currentTime1 - lastTime1;
    if (deltaTime1 >= TimeSampling1) {
      float freqSignal1 = (float)cnt1 / (deltaTime1 / 1.0e3);
      float rpm1 = abs(freqSignal1 * 60.0 / 250);
      lastTime1 = currentTime1;
      cnt1 = 0;
      error1 = sp1 - rpm1;
      sum_error1 = sum_error1 + error1;
      motorSpeed1 = abs((kp1 * error1) + (ki1 * sum_error1) + (error1 - last_error1));
      if (motorSpeed1 > 255) motorSpeed1 = 255;
      else if (motorSpeed1 < 0) motorSpeed1 = 0;
      analogWrite(pwmPin2, motorSpeed1);
      last_error1 = error1;
      //    Serial.print("posisi: ");
        
    }
  }
  else {
    sp1 = 0;
    digitalWrite (lpwmPin2, LOW);
    digitalWrite (rpwmPin2, LOW);
    //posisi1 = Count_pulses1 / 21.69426752;
    unsigned long currentTime1 = millis();
    float deltaTime1 = currentTime1 - lastTime1;
    if (deltaTime1 >= TimeSampling1) {
      float freqSignal1 = (float)cnt1 / (deltaTime1 / 1.0e3);
      float rpm1 = abs(freqSignal1 * 60.0 / 250);
      lastTime1 = currentTime1;
      cnt1 = 0;
      error1 = sp1 - rpm1;
      sum_error1 = sum_error1 + error1;
      motorSpeed1 = 0;
      if (motorSpeed1 > 255) motorSpeed1 = 255;
      else if (motorSpeed1 < 0) motorSpeed1 = 0;
      analogWrite(pwmPin2, motorSpeed1);
      last_error1 = error1;
      //    Serial.print("posisi: ");
      //    Serial.println(posisi1);
    }
  }
}

void ISR_Kanan() {
  int b = digitalRead(Encoder_output_B);
  if (b > 0) {
    Count_pulses++;
    cnt++;
  }
  else {
    Count_pulses--;
    cnt--;
  }
}

void ISR_Kiri() {
  int d = digitalRead(Encoder_output_C);
  if (d > 0) {
    Count_pulses1++;
    cnt1++;
  }
  else {
    Count_pulses1--;
    cnt1--;
  }
}
