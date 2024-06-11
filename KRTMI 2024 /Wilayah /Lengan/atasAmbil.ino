void atasAmbil (void) {
  if (kondisiXatas == 0) {
    if (prox == 0) {
      if (nextKondisi == 1) {
        ambilSampah2 ();
      } else {
        digitalWrite(LPWMB, LOW);
        digitalWrite(RPWMB, HIGH);
        analogWrite(ENB, 37);
      }
    }
    if (prox == 1) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, HIGH);
      analogWrite(ENB, 36);
      count2 = 11;
      //Serial.println(count2);
    }
    if (prox == 0 && count2 == 11) {
      count2 = 2;
    }
    if (count2 == 2) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
      delay(1000);
      ambilSampah2 ();
      // kondisiXatas = 1;
    }
  }
}
void ambilSampah2(void) {
  if (kondisiYatas == 1) {
    digitalWrite(LPWMA, LOW);
    digitalWrite(RPWMA, HIGH);
    analogWrite(ENA, kecMotorY);
    digitalWrite (LPWMV, HIGH);
    digitalWrite (RPWMV, LOW);
    analogWrite (ENV, kecMotorV);
    if (limitBawah == 0) {
      toggle = 1;
      //      Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 1) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      delay(1000);
      kondisiYatas = 2;
      //      Serial.print ("kondisiY"), Serial.println (kondisiY);
    }
  }
  if (kondisiYatas == 2) {
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
      //      Serial.print ("toggle"), Serial.println (toggle);
    }
    if (limitAtas == 0 && toggle == 11) {
      toggle = 2;
      //      Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 2) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      delay(1000);
      Serial.println ("Sampah di Ambil");
      ambil2 = 0;
      count2 = 0;
      toggle = 0;
      kondisiXatas = 0;
      kondisiYatas = 1;

      //      kondisiXatas = 1;
    }
  }
}
