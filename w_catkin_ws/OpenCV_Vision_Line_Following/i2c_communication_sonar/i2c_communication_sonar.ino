#include <Wire.h>
#include <Servo.h>
//////////////////////////// Sonar 관련 ///////////////////
#include <NewPing.h>
#define MAX_DISTANCE 1500

float UltrasonicSensorData[2];
////////////////////////////////////////////////////////////

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

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

union
{
  int encoder_data;
  byte encoder_byte[2];
} EncoderPos;

unsigned char encoderPos_data[6] = {'D', 0, 0, 0, 0, '*'}; // 시작 바이트 '#' - 끝 바이트 '*'

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
  Wire.onRequest(requestData);

  interrupt_setup();

  Serial.begin(115200);
  Steeringservo.attach(RC_SERVO_PIN);
}

///////////////////////////////// UltrasonicSensor ////////////////////////
NewPing sonar[2] =
{
  NewPing(9, 9, MAX_DISTANCE),
  NewPing(10, 10, MAX_DISTANCE),
};

void read_ultrasonic_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm() * 10.0;  // 전방
  UltrasonicSensorData[1] = sonar[1].ping_cm() * 10.0;  // 왼쪽

  if (  UltrasonicSensorData[0] == 0)
  {
    UltrasonicSensorData[0] = MAX_DISTANCE;
  }
  if (  UltrasonicSensorData[1] == 0)
  {
    UltrasonicSensorData[1] = MAX_DISTANCE;
  }
}

void ultrasonic_sensor_serial_print(void)
{
  read_ultrasonic_sensor();

  Serial.print("F_sonar_Left: ");
  Serial.print(UltrasonicSensorData[0]);
  Serial.print(" ");

  Serial.print("F_sonar_Right: ");
  Serial.print(UltrasonicSensorData[1]);
  Serial.println(" ");
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

/////////////////////////////////////// ENCODER ////////////////////////
volatile long encoderPos = 0;
//signed int encoderPos = 0;

void encoderB()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == HIGH)
  {
    encoderPos++;
  }
  else
  {
    encoderPos--;
  }
}

void reset_encoder(void)
{
  encoderPos = 0;
}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);        // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);        // quadrature encoder input B
  attachInterrupt(0, encoderB, FALLING);    // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;         // To prevent Motor Noise
}

void encoder_serial_print(void)
{
  Serial.print("encoderPos: ");
  Serial.println(encoderPos);
  //delay(2000);
  /*
    Serial.print("encoderPos00: ");
    Serial.println( EncoderPos.encoder_byte[0]);
    Serial.print("encoderPos11: ");
    Serial.println( EncoderPos.encoder_byte[1]);
  */
}

void loop()
{
  //encoder_serial_print();
  read_ultrasonic_sensor();
  ultrasonic_sensor_serial_print();
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
      /*
            Serial.print("angle_offset: ");
            Serial.println(angle);
            Serial.print("angle: ");
            Serial.println(NEUTRAL_ANGLE + angle);
            Serial.println();
      */
      //delay(1000);

      if (UltrasonicSensorData[1] > 200)
      {
        motor_control(speed);
      }
      else
      {
        motor_control(0);
      }
    }
    else
    {
      Serial.println("Invalid protocol");
    }
  }
}

void requestData()
{
  EncoderPos.encoder_data = encoderPos;
  encoderPos_data[0] = 'D';
  encoderPos_data[1] = EncoderPos.encoder_byte[0];
  encoderPos_data[2] = EncoderPos.encoder_byte[1];
  encoderPos_data[3] = 0;
  encoderPos_data[4] = 0;
  encoderPos_data[5] = '*';
  Wire.write(encoderPos_data, sizeof(encoderPos_data)); // 배열의 크기를 함께 전달
}
