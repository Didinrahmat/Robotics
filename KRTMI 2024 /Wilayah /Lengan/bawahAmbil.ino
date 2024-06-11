int countb = 0;
int countC = 0;
void bawahAmbil(void) {
  if (nextKondisi == 1 && prox ==  0){
    digitalWrite(LPWMB, HIGH);
    digitalWrite(RPWMB, LOW);
    analogWrite(ENB, kecMotorX);
    nextKondisi = 0;
    countb = 11;
  } if(prox == 0 && countb == 11){
    digitalWrite(LPWMB, HIGH);
    digitalWrite(RPWMB, LOW);
    analogWrite(ENB, kecMotorX);
    int countC = 22;
    countb = 0;
  } if(countC == 22 && prox == 0){
    digitalWrite(LPWMB, LOW);
    digitalWrite(RPWMB, LOW);
    analogWrite(ENB, 0);
    nextKondisi = 0;
    countC = 0;
  }
  //
  if (kondisiX == 0) {
    digitalWrite(LPWMB, LOW);
    digitalWrite(RPWMB, HIGH);
    analogWrite(ENB, kecMotorX);
    if (prox == 0) {
      count = 1;
    }
    if (count == 1) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
      delay(500);
      ambilSampah();
    }
  }
}
void ambilSampah(void) {
  if (kondisiY == 1) {
    digitalWrite(LPWMA, LOW);
    digitalWrite(RPWMA, HIGH);
    analogWrite(ENA, kecMotorY);
    digitalWrite (LPWMV, HIGH);
    digitalWrite (RPWMV, LOW);
    analogWrite (ENV, kecMotorV);
    toggle = 0;
    //    Serial.println (toggle);
    if (limitBawah == 0) {
      toggle = 1;
      //      Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 1) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      delay(1000);
      kondisiY = 2;
      //      Serial.print ("kondisiY"), Serial.println (kondisiY);
    }
  }
  if (kondisiY == 2) {
    if (toggle == 1 && limitBawah == 0) {
      digitalWrite(LPWMA, HIGH);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, kecMotorY);
    }
    if (toggle == 1 && limitBawah == 1) {
      digitalWrite(LPWMA, HIGH);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, kecMotorY);
      toggle = 11;
      //       Serial.print ("toggle"), Serial.println (toggle);
    }
    if (limitAtas == 0 && toggle == 11) {
      toggle = 2;
      //       Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 2) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      delay(1000);
      Serial.println ("Sampah di Ambil");
//      stoploop = 1;
      ambil = 0;
      toggle = 0;
      count = 0;
      kondisiY = 1;
      //kondisiX = 1;
    }
  }
}
