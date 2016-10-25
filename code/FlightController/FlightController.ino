#include <Servo.h>
#include "SoftI2CMaster.h"
#include "imu.h"
#define TURN_OFF 1100
#define STEP 10
#define START 1450
#define STARTBIG 1400
#define STEPBIG 10
const byte sclPin = 10;
const byte sdaPin = A3;

SoftI2CMaster i2c;

int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;

int arm_time = 0;
int count = 500;
int Pulse = 1000;
int Pulse3, Pulse4, Pulse5, Pulse7;

Servo servo3, servo4, servo5, servo7;   //this should be changed to front, right, rear, left in the future

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(57600);
//  i2c = SoftI2CMaster(sclPin, sdaPin);
//  writeGyroReg(L3G4200D_CTRL_REG1, 0x4F);
//  writeAccReg(LSM303_CTRL_REG1_A, 0b01010111);
  DDRD  |=  (_BV(PORTD3)) | (_BV(PORTD7)) | (_BV(PORTD4)) | (_BV(PORTD5));
  for (arm_time = 0; arm_time < count; ++arm_time){
    PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
    delayMicroseconds(1100);
    PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
    delay(20 - (Pulse/1000));
  }
  
  
  Pulse7 = Pulse4 = Pulse5 = START;
  Pulse3 = STARTBIG;
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
       case 'e':
         Pulse3 = Pulse4 = Pulse5 = Pulse7 = TURN_OFF;
         //Pulse = TURN_OFF;
         break;
       case 'p':
         Pulse4 = Pulse5 = Pulse7 += STEP;
         Pulse3 += STEPBIG;
         //Pulse += STEP;
         break;
       case 'm':
         Pulse4 = Pulse5 = Pulse7 -= STEP;
         Pulse3 -= STEPBIG;
         //Pulse -= STEP;
         break;
       case 's':
         Pulse4 = Pulse5 = Pulse7 = START;
         Pulse3 = STARTBIG;
         //Pulse = START;
         break;
//       case 'r':
//         Pulse4 -= STEP;
//         Pulse5 += STEP;
//         Pulse3 = Pulse7 = TURN_OFF;
//         break;
//       case 'p':
//         Pulse3 += STEP;
//         Pulse7 -= STEP;
//         Pulse4 = Pulse5 = TURN_OFF;
//         break;   
     }
   }
   
   spin(Pulse3, Pulse4, Pulse5, Pulse7);

//   readGyro();
//   readAcc();
//   printIMUData();
}

void spin(const int &Pulse3, const int &Pulse4, const int &Pulse5, const int &Pulse7) {
  servo3.write(Pulse3);
  servo4.write(Pulse4);
  servo5.write(Pulse5);
  servo7.write(Pulse7);
}


void printIMUData(){

  Serial.print("gyro_x is: ");
  Serial.println(gyro_x);

  Serial.print("gyro_y is: ");
  Serial.println(gyro_y);

  Serial.print("gyro_z is: ");
  Serial.println(gyro_z);

  Serial.println(" ");
  
  Serial.print("accel_x is: ");
  Serial.println(accel_x);

  Serial.print("accel_y is: ");
  Serial.println(accel_y);

  Serial.print("accel_z is: ");
  Serial.println(accel_z);

  Serial.println(" ");
}


void readGyro()
{
  i2c.beginTransmission(GYR_ADDRESS);
  i2c.write(L3G4200D_OUT_X_L | (1 << 7)); 
  i2c.endTransmission();
  
  i2c.requestFrom(GYR_ADDRESS);

  uint8_t xla = i2c.read();
  uint8_t xha = i2c.read();
  uint8_t yla = i2c.read();
  uint8_t yha = i2c.read();
  uint8_t zla = i2c.read();
  uint8_t zha = i2c.read();
  
   gyro_x = xha << 8 | xla;
   gyro_y = yha << 8 | yla;
   gyro_z = zha << 8 | zla;

}

void readAcc(void)
{
  i2c.beginTransmission(ACC_ADDRESS);
  // assert the MSB of the address to get the accelerometer 
  // to do slave-transmit subaddress updating.
  i2c.write(LSM303_OUT_X_L_A | (1 << 7)); 
  i2c.endTransmission();
  i2c.requestFrom(ACC_ADDRESS);

  uint8_t xla = i2c.read();
  uint8_t xha = i2c.read();
  uint8_t yla = i2c.read();
  uint8_t yha = i2c.read();
  uint8_t zla = i2c.read();
  uint8_t zha = i2c.read();

  accel_x = (xha << 8 | xla) >> 4;
  accel_y = (yha << 8 | yla) >> 4;
  accel_z = (zha << 8 | zla) >> 4;
}

void writeGyroReg(byte reg, byte value)
{
  i2c.beginTransmission(GYR_ADDRESS);
  i2c.write(reg);
  i2c.write(value);
  i2c.endTransmission();
}

void writeAccReg(byte reg, byte value)
{
  i2c.beginTransmission(ACC_ADDRESS);
  i2c.write(reg);
  i2c.write(value);
  i2c.endTransmission();
}
