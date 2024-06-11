void buangBawah (void) {
  if (kondisiXb == 1) {
    if (prox == 0) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, HIGH);
      analogWrite(ENB, kecMotorX);
    }
    if (prox == 1) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, HIGH);
      analogWrite(ENB, kecMotorX);
      count = 11;
      //      Serial.println(count);
    }
    if (prox == 0 && count == 11) {
      count = 2;
      //      Serial.println(count);
    }
    if (count == 2) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
      delay(1);
      kondisiXb = 2;
    }
  }
  if (kondisiXb == 2) {
    if (prox == 0 && count == 2) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, HIGH);
      analogWrite(ENB, 50);
    }
    if (prox == 1) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, HIGH);
      analogWrite(ENB, 50);
      count = 22;
      //      Serial.println(count);
    }
    if (prox == 0 && count == 22) {
      count = 3;
      //      Serial.println(count);
    }
    if (count == 3) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
      delay(1000);
      //kondisiY = 3;
      buangSampah2();
      //      kondisiXb = 3;
    }
  }
  if (kondisiXb == 3) {
    if (prox == 0 && count == 3) {
      digitalWrite(LPWMB, HIGH);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, kecMotorX);
    }
    if (prox == 1) {
      digitalWrite(LPWMB, HIGH);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, kecMotorX);
      count = 33;
      //    Serial.println(count);
    }
    if (prox == 0 && count == 33) {
      count = 4;
      //  Serial.println(count);
    }
    if (count == 4) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
      delay(1);
      kondisiXb = 4;
    }
  }
  if (kondisiXb == 4) {
    if (prox == 0 && count == 4) {
      digitalWrite(LPWMB, HIGH);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, kecMotorX);
    }
    if (prox == 1) {
      digitalWrite(LPWMB, HIGH);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, kecMotorX);
      count = 44;
      //      Serial.println(count);
    }
    if (prox == 0 && count == 44) {
      count = 5;
      // Serial.println(count);
    }
    if (count == 5) {
      digitalWrite(LPWMB, LOW);
      digitalWrite(RPWMB, LOW);
      analogWrite(ENB, 0);
      delay(1000);
      Serial.print ("Sampah dibuang");
      buang = 0;
      count = 0;
      toggle = 0;
      kondisiYb = 1;
      kondisiXb = 1;
    }
  }
}
void buangSampah2 (void) {
  if (kondisiYb == 1) {
    //    if ((toggle == 2 && limitAtas == 0) || (toggle == 2 && limitAtas == 1)) {
    digitalWrite(LPWMA, LOW);
    digitalWrite(RPWMA, HIGH);
    analogWrite(ENA, kecMotorY);
    toggle = 22;
    //Serial.print ("toggle"), Serial.println (toggle);
    if (limitBawah == 0 && toggle == 22) {
      toggle = 3;
      //Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 3) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      digitalWrite (LPWMV, LOW);
      digitalWrite (RPWMV, LOW);
      analogWrite (ENV, 0);
      delay(1000);
      kondisiYb = 2;
      // Serial.print ("kondisiY"), Serial.println (kondisiY);
    }
  }
  if (kondisiYb == 2) {
    if ((toggle == 3 && limitBawah == 0) || (toggle == 3 && limitBawah == 1)) {
      digitalWrite(LPWMA, HIGH);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, kecMotorY);
      toggle = 33;
      // Serial.print ("toggle"), Serial.println (toggle);
    }
    if (limitAtas == 0 && toggle == 33) {
      toggle = 4;
      //Serial.print ("toggle"), Serial.println (toggle);
    }
    if (toggle == 4) {
      digitalWrite(LPWMA, LOW);
      digitalWrite(RPWMA, LOW);
      analogWrite(ENA, 0);
      delay(1000);
      kondisiXb = 3;
    }
  }
}


//void buangBawah (void) {
//  if (kondisiXb == 1) {
//    if (prox == 0) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, HIGH);
//      analogWrite(ENB, kecMotorX);
//    }
//    if (prox == 1) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, HIGH);
//      analogWrite(ENB, kecMotorX);
//      count = 11;
//      //      Serial.println(count);
//    }
//    if (prox == 0 && count == 11) {
//      count = 2;
//      //      Serial.println(count);
//    }
//    if (count == 2) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, 0);
//      delay(1);
//      kondisiXb = 2;
//    }
//  }
//  if (kondisiXb == 2) {
//    if (prox == 0 && count == 2) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, HIGH);
//      analogWrite(ENB, kecMotorX);
//    }
//    if (prox == 1) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, HIGH);
//      analogWrite(ENB, kecMotorX);
//      count = 22;
//      //      Serial.println(count);
//    }
//    if (prox == 0 && count == 22) {
//      count = 3;
//      //      Serial.println(count);
//    }
//    if (count == 3) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, 0);
//      delay(1000);
//      //kondisiY = 3;
//      buangSampah2();
//      //kondisiX = 3;
//    }
//  }
//  if (kondisiXb == 3) {
//    if (prox == 0 && count == 3) {
//      digitalWrite(LPWMB, HIGH);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, kecMotorX);
//    }
//    if (prox == 1) {
//      digitalWrite(LPWMB, HIGH);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, kecMotorX);
//      count = 33;
//      //    Serial.println(count);
//    }
//    if (prox == 0 && count == 33) {
//      count = 4;
//      //  Serial.println(count);
//    }
//    if (count == 4) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, 0);
//      delay(1);
//      kondisiXb = 4;
//    }
//  }
//  if (kondisiXb == 4) {
//    if (prox == 0 && count == 4) {
//      digitalWrite(LPWMB, HIGH);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, kecMotorX);
//    }
//    if (prox == 1) {
//      digitalWrite(LPWMB, HIGH);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, 50);
//      count = 44;
//      //      Serial.println(count);
//    }
//    if (prox == 0 && count == 44) {
//      count = 5;
//      // Serial.println(count);
//    }
//    if (count == 5) {
//      digitalWrite(LPWMB, LOW);
//      digitalWrite(RPWMB, LOW);
//      analogWrite(ENB, 0);
//      delay(1000);
//      Serial.print ("Sampah dibuang");
////      stoploop == 1;
//      buang = 0;
//      count = 0;
//      toggle = 0;
//      kondisiYb = 1;
//      kondisiXb = 1;
//    }
//  }
//}
//void buangSampah2 (void) {
//  if (kondisiYb == 1) {
//    //    if ((toggle == 2 && limitAtas == 0) || (toggle == 2 && limitAtas == 1)) {
//    digitalWrite(LPWMA, LOW);
//    digitalWrite(RPWMA, HIGH);
//    analogWrite(ENA, kecMotorY);
//    toggle = 22;
//    //Serial.print ("toggle"), Serial.println (toggle);
//    if (limitBawah == 0 && toggle == 22) {
//      toggle = 3;
//      //Serial.print ("toggle"), Serial.println (toggle);
//    }
//    if (toggle == 3) {
//      digitalWrite(LPWMA, LOW);
//      digitalWrite(RPWMA, LOW);
//      analogWrite(ENA, 0);
//      digitalWrite (LPWMV, LOW);
//      digitalWrite (RPWMV, LOW);
//      analogWrite (ENV, 0);
//      delay(1000);
//      kondisiYb = 2;
//      // Serial.print ("kondisiY"), Serial.println (kondisiY);
//    }
//  }
//  if (kondisiYb == 2) {
//    if ((toggle == 3 && limitBawah == 0) || (toggle == 3 && limitBawah == 1)) {
//      digitalWrite(LPWMA, HIGH);
//      digitalWrite(RPWMA, LOW);
//      analogWrite(ENA, kecMotorY);
//      toggle = 33;
//      // Serial.print ("toggle"), Serial.println (toggle);
//    }
//    if (limitAtas == 0 && toggle == 33) {
//      toggle = 4;
//      //Serial.print ("toggle"), Serial.println (toggle);
//    }
//    if (toggle == 4) {
//      digitalWrite(LPWMA, LOW);
//      digitalWrite(RPWMA, LOW);
//      analogWrite(ENA, 0);
//      delay(1000);
//      kondisiXb = 3;
//    }
//  }
//}
