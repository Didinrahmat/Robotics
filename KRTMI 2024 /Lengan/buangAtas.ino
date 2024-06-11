void buangAtas(void) {
  if (kondisiXatasb == 1) {
    if (prox == 0 ) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, HIGH);
      analogWrite(ENB, 60);
    }
    if (prox == 1) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, HIGH);
      analogWrite(ENB, 45);
      count2 = 22;
      //      Serial.println(count2);
    }
    if (prox == 0 && count2 == 22) {
      count2 = 3;
      //      Serial.println(count2);
    }
    if (count2 == 3) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
      delay(500);
      buangSampah ();
      
      //      kondisiXatas = 2;
    }
  }
  if (kondisiXatasb == 2) {
    if (prox == 0 && count2 == 3) {
      digitalWrite(LPWMB, HIGH);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 60);
    }
    if (prox == 1) {
      digitalWrite(LPWMB, HIGH);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 45);
      count2 = 33;
      //      Serial.println(count2);
    }
    if (prox == 0 && count2 == 33) {
      count3 = 4;
      //      Serial.println(count2);
    }
    if (count3 == 4) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
       buang2 = 0;
      count3 = 0;
      count = 0;
      toggle = 0;
      kondisiXatasb = 1;
      kondisiYatasb = 1;
      nextKondisi = 1;
      delay(1000);
      Serial.println ("Sampah dibuang");
     
      //      kondisiXatasb = 3;
    }
  }
}
void buangSampah(void) {
  if (kondisiYatasb == 1) {
    //    if ((toggle == 2 && limitAtas == 0) || (toggle == 2 && limitAtas == 1)) {
    digitalWrite(LPWMA, LOW);
    digitalWrite(RPWMA, HIGH);
    analogWrite(ENA, kecMotorY);
    toggle = 22;
    //    Serial.print ("toggle"), Serial.println (toggle);
    if (limitBawah == 0 && toggle == 22) {
      toggle = 3;
      //      Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 3) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      digitalWrite (LPWMV, LOW);
      digitalWrite (RPWMV, LOW);
      analogWrite (ENV, 0);
      delay(1000);
      kondisiYatasb = 2;
      //      Serial.print ("kondisiY"), Serial.println (kondisiY);
    }
  }
  if (kondisiYatasb == 2) {
    if ( toggle == 3 && limitBawah == 0) {
      digitalWrite(LPWMA, HIGH);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, kecMotorY);
      toggle = 33;
      //      Serial.print ("toggle"), Serial.println (toggle);
    }
    if (limitAtas == 0 && toggle == 33) {
      toggle = 4;
      //      Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 4) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      delay(1000);
      kondisiXatasb = 2;
    }
  }
}
