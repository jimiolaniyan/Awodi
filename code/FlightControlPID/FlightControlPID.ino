#include <Servo.h>
#include "SoftI2CMaster.h"
#include "imu.h"

#define AA 0.99         // complementary filter constant

#define MIN_PULSE 1350
#define MIN_PID_PULSE 1550
#define MAX_PULSE 1850
#define PULSE_OFFSET 1400
#define LEFT_PULSE_OFFSET 1350
#define TURN_OFF 1100
#define PULSE_RANGE 500.0
#define DT 0.03
#define STOP 501
#define START 502

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.00875     // [deg/s/LSB]

#define MAX_ANGLE 30.0     // [deg]
#define MAX_RATE 180.0     // [deg/s]

// 4in x 2.5in 2-blade props, + mode
#define KP 0.85             // [command/deg]
#define KI 0.0             //
#define KD 0.22             // [command/deg/s]
#define KY 0.22             // [command/deg/s]


boolean turn_off = false;
const byte sclPin = 10;
const byte sdaPin = A3;

SoftI2CMaster i2c;

int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;

int arm_time = 0;
int count = 500;
int Pulse = 1000;
int front_command = PULSE_OFFSET; 
int rear_command = PULSE_OFFSET;
int left_command = LEFT_PULSE_OFFSET;
int right_command = PULSE_OFFSET;

Servo front_motor, rear_motor, left_motor, right_motor;   //this should be changed to front, right, rear, left in the future

unsigned int SQRT_LUT[501];

float angle_pitch = 0.0;  // [deg]
float angle_roll = 0.0;   // [deg]
float rate_pitch = 0.0;   // [deg/s]
float rate_roll = 0.0;    // [deg/s]
float rate_yaw = 0.0;     // [deg/s]

float pitch_error_integral = 0.0;
float roll_error_integral = 0.0;

int throttle_command = 0;
int pitch_command = 0;
int roll_command = 0;
int yaw_command = 0;

void setup()
{
  //initialization routines
  Serial.begin(57600);
  i2c = SoftI2CMaster(sclPin, sdaPin);
  writeGyroReg(L3G4200D_CTRL_REG1, 0x4F);
  writeAccReg(LSM303_CTRL_REG1_A, 0b01010111);
  for (int i = 0; i <= PULSE_RANGE; ++i) {
    SQRT_LUT[i] = (unsigned int) (PULSE_OFFSET + (PULSE_RANGE * sqrt((float) i / PULSE_RANGE)));
  }
  DDRD  |=  (_BV(PORTD3)) | (_BV(PORTD7)) | (_BV(PORTD4)) | (_BV(PORTD5));
  for (arm_time = 0; arm_time < count; ++arm_time){
  PORTD |= (1 << PORTD7 ) | (1 << PORTD3) | (1 << PORTD4 ) | (1 << PORTD5);
  delayMicroseconds(1100);
  PORTD &= ~(_BV(PORTD3)) & ~(_BV(PORTD7)) & ~(_BV(PORTD4)) & ~(_BV(PORTD5));
  delay(20 - (Pulse/1000));
  }

  front_motor.attach(5);
  rear_motor.attach(4);
  left_motor.attach(3);
  right_motor.attach(7);
}

void loop()
{
  int a_roll = 0;
  int a_pitch = 0;
  int g_roll = 0;
  int g_pitch = 0;
  int g_yaw = 0;

  float pitch_error = 0.0;
  float roll_error = 0.0;
  float yaw_error = 0.0;
  
  float front_command_f = 0.0;
  float rear_command_f = 0.0;
  float left_command_f = 0.0;
  float right_command_f = 0.0;
  
  //read from serial for Pad commands and then read IMU
  if (Serial.available()) {
    //do all the fetching of commands in here
    fetchPadCommands();
  }
  readGyro();
  readAcc();  
   
  a_pitch = -accel_x;
  a_roll = accel_y;
  g_pitch = gyro_y;  
  g_roll = gyro_x;
  g_yaw = gyro_z;
  
  rate_pitch = (float) g_pitch * G_GAIN;
  rate_roll = (float) g_roll * G_GAIN;
  rate_yaw = (float) g_yaw * G_GAIN;


  angle_pitch = AA * (angle_pitch + rate_pitch * DT); 
  angle_pitch += (1.0 - AA) * (float) a_pitch * A_GAIN; //complementary filter for pitch angle estimate
  pitch_error = pitch_command * MAX_ANGLE - angle_pitch;
  //pitch_error_integral += pitch_error * DT;
  //if(pitch_error_integral >= INT_SAT) { pitch_error_integral = INT_SAT; }
  //if(pitch_error_integral <= -INT_SAT) { pitch_error_integral = -INT_SAT; }
  
  angle_roll = AA * (angle_roll + rate_roll * DT); 
  angle_roll += (1.0 - AA) * (float) a_roll * A_GAIN; //complementary filter for roll angle estimate
  roll_error = roll_command * MAX_ANGLE - angle_roll;
  //roll_error_integral += roll_error * DT;
  //if(roll_error_integral >= INT_SAT) { roll_error_integral = INT_SAT; }
  //if(roll_error_inttegral <= -INT_SAT) { roll_error_integral = -INT_SAT; }

  yaw_error = yaw_command * MAX_RATE - rate_yaw;
  
  //PID Control
  front_command_f -= pitch_error * KP;
  front_command_f -= pitch_error_integral * KI;
  front_command_f += rate_pitch * KD;
  front_command_f -= yaw_error * KY;
  
  rear_command_f += pitch_error * KP;
  rear_command_f += pitch_error_integral * KI;
  rear_command_f -= rate_pitch * KD;
  rear_command_f -= yaw_error * KY;
  
  right_command_f -= roll_error * KP;
  right_command_f -= roll_error_integral * KI;
  right_command_f += 1.3 * rate_roll * KD;
  right_command_f += yaw_error * KY;
  
  left_command_f += roll_error * KP;
  left_command_f += roll_error_integral * KI;
  left_command_f -= rate_roll * KD;
  left_command_f += yaw_error * KY;
  
  if(SQRT_LUT[throttle_command] > MIN_PID_PULSE)
  {
    front_command = (int) (front_command_f + throttle_command + PULSE_OFFSET);
    rear_command = (int) (rear_command_f + throttle_command + PULSE_OFFSET);
    left_command = (int) (left_command_f + throttle_command + PULSE_OFFSET);
    right_command = (int) (right_command_f + throttle_command + PULSE_OFFSET);
    
    if(front_command > MAX_PULSE) { front_command = MAX_PULSE; }
    if(front_command < MIN_PULSE) { front_command = MIN_PULSE; }
    if(rear_command > MAX_PULSE) { rear_command = MAX_PULSE; }
    if(rear_command < MIN_PULSE) { rear_command = MIN_PULSE; }
    if(left_command > MAX_PULSE) { left_command = MAX_PULSE; }
    if(left_command < MIN_PULSE) { left_command = MIN_PULSE; }
    if(right_command > MAX_PULSE) { right_command = MAX_PULSE; }
    if(right_command < MIN_PULSE) { right_command = MIN_PULSE; }

    front_command = SQRT_LUT[front_command - PULSE_OFFSET];
    rear_command = SQRT_LUT[rear_command - PULSE_OFFSET];
    left_command = SQRT_LUT[left_command - PULSE_OFFSET];
    right_command = SQRT_LUT[right_command - PULSE_OFFSET];
    
    if(front_command < MIN_PID_PULSE) { front_command = MIN_PID_PULSE; }
    if(front_command > MAX_PULSE) { front_command = MAX_PULSE; }
    if(rear_command < MIN_PID_PULSE) { rear_command = MIN_PID_PULSE; }
    if(rear_command > MAX_PULSE) { rear_command = MAX_PULSE; }
    if(left_command < MIN_PID_PULSE) { left_command = MIN_PID_PULSE; }
    if(left_command > MAX_PULSE) { left_command = MAX_PULSE; }
    if(right_command < MIN_PID_PULSE) { right_command = MIN_PID_PULSE; }
    if(right_command > MAX_PULSE) { right_command = MAX_PULSE; }
  }

  else {
    if (turn_off) {
      front_command = rear_command = left_command = right_command = TURN_OFF;
    }
    else {
      front_command = throttle_command + PULSE_OFFSET;
      rear_command = throttle_command + PULSE_OFFSET;
      left_command = throttle_command + LEFT_PULSE_OFFSET;
      right_command = throttle_command + PULSE_OFFSET;
    }
}
   
  allSpin();
  printData();

}

void allSpin(){
  front_motor.writeMicroseconds(front_command);
  rear_motor.writeMicroseconds(rear_command);
  left_motor.writeMicroseconds(left_command);
  right_motor.writeMicroseconds(right_command);
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

void fetchPadCommands(){
   unsigned long input = readULongFromByte(); // read input
//   Serial.print("input is ");
//   Serial.print(input, DEC);
//   Serial.println("");
   if (input == STOP) {
     turn_off = true;
   }
   else if (input == START) {
     turn_off = false;
//     throttle_command = 0;
   }
   else {
     throttle_command = input;
     if (throttle_command < 0) throttle_command = 0;
     if (throttle_command > 500) throttle_command = 500;
   }
   //also read pitch_command, roll_command, yaw_command
}

void printData(){
//  Serial.print("Loop time == ");
//  Serial.println(DT);
//  Serial.print("Pitch angle == ");
//  Serial.println(angle_pitch);
//  Serial.print("Roll angle == ");
//  Serial.println(angle_roll);
  
  Serial.print("Throttle is ");
  Serial.println(throttle_command);
  Serial.print("Front command == ");
  Serial.println(front_motor.readMicroseconds());
//  Serial.print("Rear command == ");
//  Serial.println(rear_motor.readMicroseconds());
//  Serial.print("Left command == ");
//  Serial.println(left_motor.readMicroseconds());
//  Serial.print("Rear command == ");
//  Serial.println(rear_motor.readMicroseconds());  
}

unsigned long readULongFromByte(){
  union u_tag {
    byte b[4];
    unsigned long ulval;
  } u;
  
  u.b[0] = Serial.read();
  u.b[1] = Serial.read();
  u.b[2] = Serial.read();
  u.b[3] = Serial.read();
  
  return u.ulval;
}
