#define TURN_OFF 1100
#define STEP 5
#define START 1230
int arm_time = 0;
int count = 500;
int Pulse = 1000;
bool isStart = false;

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
  Pulse = START;
}

void loop() {
  
   if ( Serial.available() > 0) { // if there are bytes waiting on the serial port
     char inByte = Serial.read(); // read a byte
     
     switch (inByte) {
       case '*':
         Pulse = TURN_OFF;
         break;
       case '+':
         Pulse += STEP;
         break;
       case '-':
         Pulse -= STEP;
         break;
       case 's':
         Pulse = START;
         break;
     }
   }
  
  PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
  delayMicroseconds(Pulse);
  PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
  delay(20 - (Pulse/1000));
}
