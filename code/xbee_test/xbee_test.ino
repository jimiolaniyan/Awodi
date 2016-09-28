void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0){
    char ch = Serial.read();
    Serial.write(ch);
  }
    //Serial.println(Serial.read());

  
  /*else {
    Serial.write('*');
    Serial.println('*');
    Serial.write("stop");
    Serial.println("stop");
  }
  
  Serial.write('*');
  Serial.println('*');
  Serial.write("stop");
  Serial.println("stop");
  */
  delay(2000);
}
