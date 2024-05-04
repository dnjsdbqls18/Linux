#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x05

#define NEUTRAL_ANGLE 90
#define RC_SERVO_PIN   2

#define encodPinA1   2
#define encodPinB1   3
#define MOTOR_DIR    4
#define MOTOR_PWM    5

int Steering_Angle = 0;
Servo Steeringservo;

union 
{
  short data;
  byte bytes[2];
} car_speed, car_servo;

void setup() 
{
  #if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  #endif
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
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, speed);
  }
  else 
  {
    digitalWrite(MOTOR_DIR, HIGH);
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
      car_servo.bytes[0] = receivedData[2];
      car_servo.bytes[1] = receivedData[3];
      short angle = car_servo.data;

      car_speed.bytes[0] = receivedData[4];
      car_speed.bytes[1] = receivedData[5];
      float speed = car_speed.data;
      
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
