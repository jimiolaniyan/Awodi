int STATE = 1;
int arm_time = 0;
int count = 500;
int Pulse = 1000;

void setup() {
  // put your setup code here, to run once:
  
  //pinMode(pin1, OUTPUT);
  Serial.begin(9600);
  DDRD  |=  _BV(PORTD7);
  for (arm_time = 0; arm_time < count; ++arm_time){
    PORTD |= (1 << PORTD7 );
    delayMicroseconds(1100);
    PORTD &= ~(_BV(PORTD7));
    delay(20 - (Pulse/1000));
    
    //analogWrite(pin, 14);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
 /*for (Pulse = 1150; Pulse <= 1400; ++Pulse){
    Serial.println(Pulse);
    PORTD |= (1 << PORTD7 );
    delayMicroseconds(Pulse);
    PORTD &= ~(_BV(PORTD7));
    delay(20 - (Pulse/1000));
  }
  
  for (Pulse = 1400; Pulse >= 1150; --Pulse){
    PORTD |= (1 << PORTD7 );
    delayMicroseconds(Pulse);
    PORTD &= ~(_BV(PORTD7));
    delay(20 - (Pulse/1000));   
  }*/

  Pulse = 1219;
  
  PORTD |= (1 << PORTD7 );
  delayMicroseconds(Pulse);
  PORTD &= ~(_BV(PORTD7));
  delay(20 - (Pulse/1000));
}
