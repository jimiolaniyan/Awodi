void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  
  while (Serial.available() > 0){
    char ch = Serial.read();
    Serial.write(ch);
  }
    //Serial.println(Serial.read());
 
  
  //Serial.write('*');
  //Serial.println('*');
  Serial.write("stop");
  //Serial.println("stop");

  delay(2000);
}
