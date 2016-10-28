#include <Servo.h>

#define TURN_OFF 1100
#define START 1500
int arm_time = 0;
int count = 500;
int Pulse = 1000;


Servo servo1, servo2, servo3, servo4;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(57600);
  DDRD  |=  _BV(PORTD3) | _BV(PORTD4)| _BV(PORTD5)| _BV(PORTD7);
  for (arm_time = 0; arm_time < count; ++arm_time){
    PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
    delayMicroseconds(1100);
    PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
    delay(20 - (Pulse/1000));
  }
 
  Pulse = START;
  servo1.attach(7);
  servo2.attach(5);
  servo3.attach(3);
  servo4.attach(4);
  
  allSpin(Pulse);
}



void loop() {
  
  if (Serial.available()) {
    char input = Serial.read();
    
    switch (input) {
      case 'e':
        Pulse = TURN_OFF;
        break;
      case 'p':
        Pulse += 100;
        break;
      case 'm':
        Pulse -= 100;
        break;
      case 's':
        Pulse = START; 
    }
  }
  
  allSpin(Pulse);
}

void allSpin(int Pulse) {
  servo1.write(Pulse);
  servo2.write(Pulse);
  servo3.write(Pulse);
  servo4.write(Pulse);
}
