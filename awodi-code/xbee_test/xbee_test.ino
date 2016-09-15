void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);

}

void loop() {
  // put your main code here, to run repeatedly:
  /*if (Serial.available() > 0) {
    Serial.write(Serial.read());
    Serial.println(Serial.read());
  }
  
  else {
    Serial.write('*');
    Serial.println('*');
    Serial.write("stop");
    Serial.println("stop");
  }
  */
  Serial.write('*');
  Serial.println('*');
  Serial.write("stop");
  Serial.println("stop");
  
  delay(5000);
}
