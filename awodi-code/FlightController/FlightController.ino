#define TURN_OFF 1100
int arm_time = 0;
int count = 500;
int Pulse = 1000;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  DDRD  |=  (_BV(PORTD3)) | (_BV(PORTD7)) | (_BV(PORTD4)) | (_BV(PORTD5));
  //delay(10000);
  for (arm_time = 0; arm_time < count; ++arm_time){
    PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
    delayMicroseconds(1100);
    PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
    delay(20 - (Pulse/1000));
    
    //analogWrite(pin, 14);
  }
}

void loop() {
  Pulse = 1250;
   if ( Serial.available() > 0) { // if there are bytes waiting on the serial port
   char inByte = Serial.read(); // read a byte
   if (inByte == '*') { // if that byte is the desired character
   int len = 4; // expected string is 6 bytes long
   char inString[len]; // declare string variable
   for (int i = 0; i < len; i++) {
   inString[i] = Serial.read();
 }
   if ( strstr(inString, "stop") != NULL ) // check to see if the respose is "reset"
     Pulse = TURN_OFF; // reset the chip after waiting for the specified # of milliseconds
   }
  }
  
  PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
  delayMicroseconds(Pulse);
  PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
  delay(20 - (Pulse/1000));
}
