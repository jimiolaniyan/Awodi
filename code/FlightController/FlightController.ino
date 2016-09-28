#include <Servo.h>
#define TURN_OFF 1100
#define STEP 10
#define START 1230
int arm_time = 0;
int count = 500;
int Pulse = 1000;
int Pulse3, Pulse4, Pulse5, Pulse7;

Servo servo3, servo4, servo5, servo7;   //this should be changed to front, right, rear, left in the future

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  DDRD  |=  (_BV(PORTD3)) | (_BV(PORTD7)) | (_BV(PORTD4)) | (_BV(PORTD5));
  for (arm_time = 0; arm_time < count; ++arm_time){
    PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
    delayMicroseconds(1100);
    PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
    delay(20 - (Pulse/1000));
  }
  
  Pulse3 = Pulse4 = Pulse5 = Pulse7 = START;
  servo3.attach(3);
  servo4.attach(4);
  servo5.attach(5);
  servo7.attach(7);
  
  //Pulse = START;
}

void spin(const int &, const int &, const int &, const int &);

void loop() {
  
   if ( Serial.available() > 0) { // if there are bytes waiting on the serial port
     char inByte = Serial.read(); // read a byte
     
     switch (inByte) {
       case '*':
         Pulse3 = Pulse4 = Pulse5 = Pulse7 = TURN_OFF;
         //Pulse = TURN_OFF;
         break;
       case '+':
         Pulse3 = Pulse4 = Pulse5 = Pulse7 += STEP;
         //Pulse += STEP;
         break;
       case '-':
         Pulse3 = Pulse4 = Pulse5 = Pulse7 -= STEP;
         //Pulse -= STEP;
         break;
       case 's':
         Pulse3 = Pulse4 = Pulse5 = Pulse7 = START;
         //Pulse = START;
         break;
       case 'r':
         Pulse4 -= STEP;
         Pulse5 += STEP;
         Pulse3 = Pulse7 = TURN_OFF;
         break;
       case 'p':
         Pulse3 -= STEP;
         Pulse7 += STEP;
         Pulse4 = Pulse5 = TURN_OFF;
         break;   
     }
   }
   
   spin(Pulse3, Pulse4, Pulse5, Pulse7);
  
  /*PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
  delayMicroseconds(Pulse);
  PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
  delay(20 - (Pulse/1000));*/
}

void spin(const int &Pulse3, const int &Pulse4, const int &Pulse5, const int &Pulse7) {
  servo3.write(Pulse3);
  servo4.write(Pulse4);
  servo5.write(Pulse5);
  servo7.write(Pulse7);
}
