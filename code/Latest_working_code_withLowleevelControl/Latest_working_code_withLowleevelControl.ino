int STATE = 1;
int arm_time = 0;
int pin1 = 7;
//int pin2 = 5;
int pin3 = 6;
//int pin4 = 7;
int count = 500;
int Pulse = 1000;

void setup() {
  // put your setup code here, to run once:
  
  //pinMode(pin1, OUTPUT);
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
  // put your main code here, to run repeatedly:
 for (Pulse = 1220; Pulse <= 1500; ++Pulse){
    PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
    delayMicroseconds(Pulse);
    PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
    delay(20 - (Pulse/1000));
  }
  
  for (Pulse = 1500; Pulse >= 1220; --Pulse){
    PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
    delayMicroseconds(Pulse);
    PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
    delay(20 - (Pulse/1000));   
  }
}
