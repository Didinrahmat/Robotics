void normal() {
  digitalWrite(LPWMB, HIGH);
  digitalWrite(RPWMB, LOW);
  analogWrite(ENB, 60);

  if (prox == 0 ) {
    digitalWrite(LPWMB, HIGH);
    digitalWrite(RPWMB, LOW);
    analogWrite(ENB, 40);
  }
  if (prox == 1) {
    digitalWrite(LPWMB, HIGH);
    digitalWrite(RPWMB, LOW);
    analogWrite(ENB, 40);
    count2 = 22;
    //      Serial.println(count2);
  }
  if (prox == 0 && count2 == 22) {
    digitalWrite(LPWMB, LOW);
    digitalWrite(RPWMB, LOW);
    analogWrite(ENB, 0);
    delay(1000);
  }
}
