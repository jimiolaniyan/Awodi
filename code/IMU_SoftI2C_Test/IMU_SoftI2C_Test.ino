#include "SoftI2CMaster.h"
#include "imu.h"

const byte sclPin = 3;
const byte sdaPin = 5;

SoftI2CMaster i2c;

int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;

void setup() {
  // put your setup code here, to run once:
  i2c = SoftI2CMaster(sclPin, sdaPin);
  Serial.begin(57600);
  
  writeGyroReg(L3G4200D_CTRL_REG1, 0x4F);
  writeAccReg(LSM303_CTRL_REG1_A, 0b01010111);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  readGyro();
  readAcc();
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

  delay(200);
  
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
