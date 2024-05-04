#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x05

#define NEUTRAL_ANGLE 90
#define RC_SERVO_PIN   8

#define encodPinA1   2
#define encodPinB1   3

#define MOTOR_DIR    4
#define MOTOR_PWM    5

Servo Steeringservo;

union 
{
  short steering_angle_data;
  byte angle_byte[2];
} Steering;

union 
{
  int speed_data;
  byte speed_byte[2];
} Car_Speed;

void setup() 
{
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  
  Serial.begin(115200);
  Steeringservo.attach(RC_SERVO_PIN);   
}

void motor_control(int speed)
{
  if (speed >= 0) 
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, speed);
  }
  else 
  {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, -speed);
  }
  Serial.print("speed: ");
  Serial.println(speed);
  Serial.println();    
}

void loop() 
{
  delay(100);
}

void receiveData(int byteCount) 
{
  if (Wire.available() >= 9) 
  {
    byte receivedData[9];
    for (int i = 0; i < 9; i++) 
    {
      receivedData[i] = Wire.read(); 
    }

    if (receivedData[0] == '#' && receivedData[1] == 'C' && receivedData[8] == '*') 
    {
      Steering.angle_byte[0] = receivedData[2];
      Steering.angle_byte[1] = receivedData[3];
      short angle = Steering.steering_angle_data;

      Car_Speed.speed_byte[0] = receivedData[4];
      Car_Speed.speed_byte[1] = receivedData[5];
      int speed = Car_Speed.speed_data;
      
      Steeringservo.write(NEUTRAL_ANGLE + angle);
  
      Serial.print("angle_offset: ");
      Serial.println(angle);
      Serial.print("angle: ");
      Serial.println(NEUTRAL_ANGLE + angle);
      Serial.println();
      delay(1000);
      
      motor_control(speed);
    } 
    else 
    {
      Serial.println("Invalid protocol");
    }
  }
}
