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

volatile unsigned long pulse_time_left_motor = 1500;
char bufferLeftMotor[20];

volatile unsigned long pulse_time_right_motor = 1500;
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

  left_motor.attach(CHANNEL_LEFT_MOTOR);
  right_motor.attach(CHANNEL_RIGHT_MOTOR);

  Serial.println("Ready");
}

// Modifier : between 0 and -0.5 in our case (-1 and 1 for other usage)
unsigned long compute_mixed_output(unsigned long forward_value, unsigned long backward_value, unsigned long mid_value, unsigned long input_throttle, float modifier) {

  unsigned long current_displacement_extremum, current_throttle_extremum, output_throttle;
  float float_output_throttle, normalized_modified_input;

  // WATCH SAFETY
  if (input_throttle == THROTTLE_MID)
  {
    input_throttle = THROTTLE_MID + 1;
  }

  // check if we are in backward or forward movement
  if (input_throttle > THROTTLE_MID)
  {
    current_displacement_extremum = forward_value;
    current_throttle_extremum = THROTTLE_FORWARD;
  }
  else
  {
    current_displacement_extremum = backward_value;
    current_throttle_extremum = THROTTLE_BACKWARD;
  }

  // apply modifier and normalize
  normalized_modified_input = (float)(((float)input_throttle - THROTTLE_MID) * (1 + modifier));

  // Scale input (useless if throttle input max = output max)
  // note that normalized_modified_input is already normalized
  float_output_throttle = (float)((normalized_modified_input * ((float)current_displacement_extremum - (float)mid_value)) / ((float)current_throttle_extremum - (float)THROTTLE_MID)) + mid_value;

  // SAFETY clip value
  output_throttle = constrain((unsigned long)float_output_throttle, min(forward_value, backward_value), max(forward_value, backward_value));

  return(output_throttle);
}



void loop() {

  _now = millis();

  // write value (outside the refresh func, otherwise we have strange behavuior)
  left_motor.writeMicroseconds((int)pulse_time_left_motor);
  right_motor.writeMicroseconds((int)pulse_time_right_motor);

  if (_now - last_loop > REFRESH_RATE)
  {
    // disable interrupts to avoid werid behavior
    noInterrupts();

    // WATCH SECURITY : go to no throttle and no yaw when no input
    if (pulse_time_throttle == 0 || pulse_time_yaw == 0)
    {
      pulse_time_throttle = THROTTLE_MID;
      pulse_time_yaw = YAW_MID;
    }

    if(pulse_time_yaw > YAW_MID + YAW_DEADBAND || pulse_time_yaw < YAW_MID - YAW_DEADBAND)
    {

      yaw_modifier = abs(((float)pulse_time_yaw-1500.0)/(float)1000) * YAW_REVERSE;

      // check if we are in backward or forward movement
      if (pulse_time_yaw > YAW_MID + YAW_DEADBAND)
      {
        left_yaw_modifier = 0;
        right_yaw_modifier = -yaw_modifier;
      }
      else if (pulse_time_yaw < YAW_MID - YAW_DEADBAND)
      {
        left_yaw_modifier = -yaw_modifier;
        right_yaw_modifier = 0;
      }
    } 
    else
    {
      yaw_modifier = 0;
      left_yaw_modifier = 0;
      right_yaw_modifier = 0;
    }

    // calculate transformation
    pulse_time_left_motor = compute_mixed_output(LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, LEFT_MOTOR_MID, pulse_time_throttle, left_yaw_modifier);
    pulse_time_right_motor = compute_mixed_output(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, RIGHT_MOTOR_MID, pulse_time_throttle, right_yaw_modifier);
   
    
    Serial.print("Yaw : ");
    Serial.println(pulse_time_yaw);
    Serial.print("Trottle : ");
    Serial.println(pulse_time_throttle);
    Serial.print("LEFT throttle : ");
    Serial.println(pulse_time_left_motor);
    Serial.print("RIGHT throttle : ");
    Serial.println(pulse_time_right_motor);
    Serial.print("left_yaw_modifier : ");
    Serial.println(left_yaw_modifier);
    Serial.print("right_yaw_modifier : ");
    Serial.println(right_yaw_modifier);
    Serial.println("---------------");
    

    
    last_loop = _now;

    // reset pulse_time measuration 
    rising_edge_throttle = 0;
    rising_edge_yaw = 0;
    pulse_time_throttle = 0;
    pulse_time_yaw = 0;
    interrupts();
  }
}
