#include <Servo.h> 
#include <Wire.h>
#include <SPI.h>

#define I2C_BRAIN_ADDRESS 0x42

#define I2C_COMMAND_ULTRASONIC_ROTATION 0x10
#define I2C_COMMAND_GRIPPER_ROTATION 0x11
#define I2C_COMMAND_GRIPPER_GRIP 0x12
#define I2C_COMMAND_MOVE_FORWARD 0x20
#define I2C_COMMAND_MOVE_BACKWARD 0x21
#define I2C_COMMAND_MOVE_LEFT 0x22
#define I2C_COMMAND_MOVE_RIGHT 0x23
#define I2C_COMMAND_MOVE_STOP 0x24
#define I2C_COMMAND_MOVE_SET_SPEED 0x25

#define ULTRASONIC_ROTATION_SERVO_PIN 5
#define GRIPPER_ROTATION_SERVO_PIN 11
#define GRIPPER_GRIP_SERVO_PIN 6


#define ULTRASONIC_ROTATION_INITIAL_ANGLE 45
#define GRIPPER_ROTATION_INITIAL_ANGLE 90
#define GRIPPER_GRIP_INITIAL_ANGLE 90

Servo ultrasonic_rotation;
Servo gripper_grip;
Servo gripper_rotation;

int ultra_angle = 90;

void setup() {
  
  Serial1.begin(9600);


  
  // Setup I2C
  Wire.begin(I2C_BRAIN_ADDRESS);
  Wire.onReceive(i2cEvent);
  
  // Setup SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); 
  
  // Attach servos
  ultrasonic_rotation.attach(ULTRASONIC_ROTATION_SERVO_PIN);
  gripper_rotation.attach(GRIPPER_ROTATION_SERVO_PIN);
  gripper_grip.attach(GRIPPER_GRIP_SERVO_PIN);
  
  
  // Set servos to initial angles
  ultrasonic_rotation.write(ULTRASONIC_ROTATION_INITIAL_ANGLE);
  gripper_rotation.write(ULTRASONIC_ROTATION_INITIAL_ANGLE);
  gripper_grip.write(ULTRASONIC_ROTATION_INITIAL_ANGLE);
  
  DDRD |= (1<<6);
  
  uint8_t shift_outputs[2];
  shift_outputs[0] = 0;
    shift_outputs[1] = 0;
  //output values to shift registers
    PORTD &= ~(1<<6); // pull PD6 (shift-register latch) low
    SPI.transfer(shift_outputs[1]);
    SPI.transfer(shift_outputs[0]);
    PORTD |= (1<<6); // pull PD6 (shift-register latch) high
    
  Serial1.println("Init completed");
}

void loop() {

}


void i2cEvent(int bytes) {
  Serial1.print("I2C Received. Length: ");
  Serial1.print(bytes);
  Serial1.print(" Data: ");
  
  int data[bytes];
  byte i = 0;
  while (Wire.available() > 0){
    int b = Wire.read();
    data[i++] = b;
    Serial1.print(b);
    Serial1.print(" ");
  }
  
  Serial1.println();
  
  switch (data[0]) {
    case I2C_COMMAND_ULTRASONIC_ROTATION:
      ultrasonic_rotation.write(data[1]);
      break;
    case I2C_COMMAND_GRIPPER_ROTATION:
      gripper_rotation.write(data[1]);
      break;
    case I2C_COMMAND_GRIPPER_GRIP:
      gripper_grip.write(data[1]);
      break;
  }
}