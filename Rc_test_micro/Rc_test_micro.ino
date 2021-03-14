#include <Servo.h>
//#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.

const int CHANNEL_YAW_PIN = 2;
const int CHANNEL_THROTTLE_PIN = 3;

const unsigned long YAW_DEADBAND = 50;
const unsigned long YAW_MIN = 1000;
const unsigned long YAW_MID = 1500;
const unsigned long YAX_MAX = 2000;
const unsigned long YAW_REVERSE = 1;

const unsigned long THROTTLE_MID = 1500;
const unsigned long THROTTLE_FORWARD = 2000;
const unsigned long THROTTLE_BACKWARD = 1000;

const int CHANNEL_LEFT_MOTOR = 10;
const int CHANNEL_RIGHT_MOTOR = 9;

const unsigned long LEFT_MOTOR_MID = 1500;
const unsigned long LEFT_MOTOR_FORWARD = 1200;
const unsigned long LEFT_MOTOR_BACKWARD = 1800;

const unsigned long RIGHT_MOTOR_MID = 1500;
const unsigned long RIGHT_MOTOR_FORWARD = 1800;
const unsigned long RIGHT_MOTOR_BACKWARD = 1200;

volatile unsigned long pulse_time_left_motor = 0;
char bufferLeftMotor[20];

volatile unsigned long pulse_time_right_motor = 0;
char bufferRightMotor[20];

const unsigned long REFRESH_RATE = 100; // in ms
unsigned long _now;
unsigned long last_loop;

float yaw_modifier = 0;
float left_yaw_modifier = 0;
float right_yaw_modifier = 0;

volatile unsigned long rising_edge_throttle = 0;
volatile unsigned long pulse_time_throttle = 0;
char bufferThrottle[20];
void calcSignalThrottle() 
{
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(CHANNEL_THROTTLE_PIN) == HIGH) 
    { 
        rising_edge_throttle = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(rising_edge_throttle != 0)
        { 
            //record the pulse time
            pulse_time_throttle = (micros() - rising_edge_throttle);
            //Serial.println("Trottle : " + pulse_time_throttle);
        }
    } 
}

volatile unsigned long rising_edge_yaw = 0;
volatile unsigned long pulse_time_yaw = 0;
char bufferYaw[20];

void calcSignalYaw() 
{
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(CHANNEL_YAW_PIN) == HIGH) 
    { 
        rising_edge_yaw = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(rising_edge_yaw != 0)
        { 
            //record the pulse time
            pulse_time_yaw = (micros() - rising_edge_yaw);
        }
    }
} 

Servo left_motor;
Servo right_motor;

void setup() {

  _now = 0;
  last_loop = 0;
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(CHANNEL_YAW_PIN,INPUT);
  pinMode(CHANNEL_THROTTLE_PIN,INPUT);

  attachInterrupt(digitalPinToInterrupt(CHANNEL_YAW_PIN), calcSignalYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_THROTTLE_PIN), calcSignalThrottle, CHANGE);

  left_motor.attach(CHANNEL_LEFT_MOTOR, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
  right_motor.attach(CHANNEL_RIGHT_MOTOR, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);

  pulse_time_left_motor = 1500;
  pulse_time_right_motor = 1500;
  

  Serial.println("Ready");
}

bool ascending = 1;
int step_pulse = 5;

void loop() {

  _now = millis();

  // write value
  left_motor.writeMicroseconds((int)pulse_time_left_motor);
  right_motor.writeMicroseconds((int)pulse_time_right_motor);

  if (_now - last_loop > REFRESH_RATE)
  {
    // disable interrupts to avoid werid behavior
    noInterrupts();

    

    if (ascending)
    {
      pulse_time_left_motor += step_pulse; 
      pulse_time_right_motor += step_pulse; 
    } 
    else 
    {
      pulse_time_left_motor -= step_pulse; 
      pulse_time_right_motor -= step_pulse; 
    }

    if (pulse_time_left_motor > 1600)
    {
      ascending = 0;
    }

    if (pulse_time_left_motor < 1400)
    {
      ascending = 1;
    }

    

  /*  
    Serial.print("Yaw : ");
    Serial.println(pulse_time_yaw);
    Serial.print("Trottle : ");
    Serial.println(pulse_time_throttle);
    Serial.print("LEFT throttle : ");
    Serial.println(pulse_time_left_motor);
    Serial.print("RIGHT throttle : ");
    Serial.println(pulse_time_right_motor);
    Serial.print("left_yaw_modifier : ");
    Serial.println("---------------");
    */
    
    last_loop = _now;

    // reset pulse_time measuration 
    rising_edge_throttle = 0;
    rising_edge_yaw = 0;
    pulse_time_throttle = 0;
    pulse_time_yaw = 0;
    interrupts();
  }
}
