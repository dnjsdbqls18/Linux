#include <MsTimer2.h>
#include "sbus.h"

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

////////////////// Serial data //////////////////////////
#define NO_OF_RECEIVED_BUFFER 7

// Serial data
unsigned char buf[NO_OF_RECEIVED_BUFFER];  // Buffer to store serial data

union
{
  short data;
  byte bytedata[2];
} m_car_speed_int16;

union
{
  short data;
  byte bytedata[2];
} m_car_angle_int16;

/////////////////////////////////// sbus ///////////
#define Sbus_Speed_max  200
#define Sbus_Speed_zero  1000
#define Sbus_Speed_min   1800

#define Sbus_str_max  1800
#define Sbus_str_zero  1040
#define Sbus_str_min   240

/* SBUS 객체, SBUS 읽기 */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS 객체, SBUS 쓰기 */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS 데이터 */
bfs::SbusData data;


struct MotorData
{
  int motor_speed;
  int str_position;
};
MotorData MD;

/////////////////////////// Steering_Control//////////////////
#define STEER_MOTOR_PWM              2
#define STEER_MOTOR_IN1              3
#define STEER_MOTOR_IN2              4

#define RIGHT_MOTOR_PWM              5
#define RIGHT_MOTOR_IN1              6
#define RIGHT_MOTOR_IN2              7

#define LEFT_MOTOR_PWM               8
#define LEFT_MOTOR_IN1               9
#define LEFT_MOTOR_IN2              10

#define STEERING_ANGLE_PIN          A0
#define STEERING_ANGLE_CONTROL_PIN  A2

#define NEURAL_ANGLE               450
#define LEFT_STEER_ANGLE           530
#define RIGHT_STEER_ANGLE          370

#define Max_Speed                  240
#define SBUS_Max_Speed              50

#define NO_OF_DATA                  15

int Steering_Angle;
int Steering_Angle_Control;
float Filter_Steering_Angle;

float Kp_Steer         = 4.0;
float Ki_Steer         = 0.0;
float Kd_Steer         = 4.0;

float error            = 0.0;
float error_s          = 0.0;
float error_d          = 0.0;
float error_old        = 0.0;

float steering_control = 0.0;
float control_pwm      = 0.0;
float target_steering  = 0.0;
int target_angle       = 0;

float Data[NO_OF_DATA] = {0,};

float recursive_moving_average(float ad_value)
{
  static float avg = 0.0;

  for (int i = 0; i < NO_OF_DATA - 1; i++)
  {
    Data[i] = Data[i + 1];
    //Serial.print(data[i]);
    //Serial.print("  ");
  }
  Data[NO_OF_DATA - 1] = ad_value;
  //Serial.println(data[NO_OF_DATA - 1]);
  //Serial.print("Average_old2: "); Serial.println(avg);

  avg = avg + ((Data[NO_OF_DATA - 1] - Data[0]) / (float)NO_OF_DATA);

  return avg;
}

void read_Potentiometer(void)
{
  Steering_Angle = analogRead(STEERING_ANGLE_PIN);
  Filter_Steering_Angle = recursive_moving_average(Steering_Angle);
  Steering_Angle_Control = analogRead(STEERING_ANGLE_CONTROL_PIN);
}

void Ms2Timer_setup(void)
{
  MsTimer2::set(10, control_steering);
  MsTimer2::start();
}

////////////////////////////// SETUP ////////////////////////////////
void setup()
{
#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  pinMode(STEER_MOTOR_PWM, OUTPUT);
  pinMode(STEER_MOTOR_IN1, OUTPUT);
  pinMode(STEER_MOTOR_IN2, OUTPUT);

  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  pinMode(LEFT_MOTOR_PWM,  OUTPUT);
  pinMode(LEFT_MOTOR_IN1,  OUTPUT);
  pinMode(LEFT_MOTOR_IN2,  OUTPUT);

  //시리얼 통신이 확립될 때까지 기다리기 위함
  //디버깅 목적으로 시리얼 모니터를 사용할 때 중요
  while (!Serial) {}
  /* SBUS 통신 시작 */
  sbus_rx.Begin();
  sbus_tx.Begin();

  Ms2Timer_setup();

  Serial.begin(115200);
  Serial2.begin(115200); // recieve_car_control_command
}

void steer_motor_control(int speed)
{
  if (speed >= 0)
  {
    if (speed >= Max_Speed)
    {
      speed = Max_Speed;
    }
    digitalWrite(STEER_MOTOR_IN1, LOW);
    digitalWrite(STEER_MOTOR_IN2, HIGH);
    analogWrite(STEER_MOTOR_PWM, speed);
  }
  else
  {
    if (speed <= -Max_Speed)
    {
      speed = -Max_Speed;
    }
    digitalWrite(STEER_MOTOR_IN1, HIGH);
    digitalWrite(STEER_MOTOR_IN2, LOW);
    analogWrite(STEER_MOTOR_PWM, -speed);
  }
}

void motor_control(int speed)
{
  if (speed >= 0)
  {
    if (speed >= Max_Speed)
    {
      speed = Max_Speed;
    }

    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_PWM, speed);

    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_PWM, speed);
  }
  else
  {
    if (speed <= -Max_Speed)
    {
      speed = -Max_Speed;
    }
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_PWM, -speed);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_PWM, -speed);
  }
}

void PID_Control(void)
{
  error = target_angle - Filter_Steering_Angle;
  error_d = error - error_old;

  steering_control = Kp_Steer * error + Kd_Steer * error_d;

  error_old = error;

  control_pwm = -steering_control;
  steer_motor_control((int)control_pwm);
}

void target_angle_limit(void)
{
  if (target_angle < RIGHT_STEER_ANGLE)
  {
    target_angle = RIGHT_STEER_ANGLE;
  }
  else if (target_angle > LEFT_STEER_ANGLE)
  {
    target_angle = LEFT_STEER_ANGLE;
  }
  else
  {
    target_angle = target_angle;
  }
}

void Sbus_motor_control()
{
  if (sbus_rx.Read())
  {
    data = sbus_rx.data();


    for (int8_t i = 0; i < 20; i++)
    {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    Serial.println("");


    sbus_tx.data(data);
    sbus_tx.Write();

    if (data.ch[1] >= Sbus_Speed_zero + 30)
    {
      MD.motor_speed = map(data.ch[1], Sbus_Speed_zero, Sbus_Speed_min, 0, -SBUS_Max_Speed);
    }
    else if (data.ch[1] < Sbus_Speed_zero - 30)
    {
      MD.motor_speed = map(data.ch[1], Sbus_Speed_max, Sbus_Speed_zero, SBUS_Max_Speed, 0);
    }
    else
    {
      MD.motor_speed = 0;
    }

    if (data.ch[0] > Sbus_str_zero)
    {
      MD.str_position = map(data.ch[0], Sbus_str_min, Sbus_str_zero, LEFT_STEER_ANGLE, NEURAL_ANGLE);
    }
    else
    {
      MD.str_position = map(data.ch[0], Sbus_str_zero, Sbus_str_max, NEURAL_ANGLE, RIGHT_STEER_ANGLE);
    }
  }
}

void serialEvent() // Use serialEvent2 for Serial2
{
  int received_data_no = Serial2.available();

  if (Serial2.available() <= 0)
  {
    return;
  }

  // Shift buffer and read new data
  for (int i = 0; i < received_data_no && i < sizeof(buf); i++)
  {
    // Shift buffer
    for (int j = 0; j < NO_OF_RECEIVED_BUFFER - 1; j++)
    {
      buf[j] = buf[j + 1];
    }
    // Read new byte into the last position
    buf[NO_OF_RECEIVED_BUFFER - 1] = Serial2.read();

    Serial.write(buf[NO_OF_RECEIVED_BUFFER - 1]); // Echo to Serial monitor
  }

  if ((buf[0] == '#') && (buf[1] == 'C') && (buf[6] == '*'))
  {
    //Serial.println("통신");
    //Serial.println(" ");
    m_car_angle_int16.bytedata[0] = buf[2];
    m_car_angle_int16.bytedata[1] = buf[3];

    m_car_speed_int16.bytedata[0] = buf[4];
    m_car_speed_int16.bytedata[1] = buf[5];

    //Serial.print(m_car_speed_int16.data);
    //Serial.print(" ");
    //Serial.println(m_car_angle_int16.data);

  }
}

void joy_contorl()
{
  Sbus_motor_control();
  if (data.ch[6] == 1800)
  {
    motor_control(MD.motor_speed);
    target_angle = MD.str_position;
  }
  else
  {
    motor_control(0);
  }
}

void Steering_Angle_Print(void)
{
  Serial.print("Steering_Angle: ");
  Serial.println(Steering_Angle);
  //Serial.print("Steering_Angle_Control: ");
  //Serial.println(Steering_Angle_Control);
  Serial.print("steering_control: ");
  Serial.println(steering_control);
  Serial.print("target_angle: ");
  Serial.println(target_angle);
  //Serial.print("Filter_Steering_Angle: "); // 필터링된 조향각 출력
  //Serial.println(Filter_Steering_Angle);

  /////////////////////// 조종기 확인 ////////////////////////
  //Serial.print("motor_speed: ");
  //Serial.println(motor_speed);
  //Serial.print("str_position: ");
  //Serial.println(str_position);
  Serial.print("\n");
}

void teleop_control()
{
  motor_control(m_car_speed_int16.data);
  target_angle = m_car_angle_int16.data;
}

void control_steering(void)
{
  //joy_contorl();  
  read_Potentiometer();  
  serialEvent();
  teleop_control();
  PID_Control();
}

void loop()
{
  Steering_Angle_Print();

  //target_angle = 465;
  target_angle_limit();
}
