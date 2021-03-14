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

const int THROTTLE_ARRAY_SIZE = 10;

volatile unsigned long pulse_time_left_motor = LEFT_MOTOR_MID;
int t_throttle_left_motor[THROTTLE_ARRAY_SIZE];
int index_throttle_left_motor = 0;
int throttle_left_motor = LEFT_MOTOR_MID;

volatile unsigned long pulse_time_right_motor = LEFT_MOTOR_MID;
int t_throttle_right_motor[THROTTLE_ARRAY_SIZE];
int index_throttle_right_motor = 0;
int throttle_right_motor = LEFT_MOTOR_MID;

const unsigned long REFRESH_TIME= 10; // in ms
const unsigned long SERIAL_TIME = 200; // in ms
const unsigned long SERIAL_TIME_CYCLE = SERIAL_TIME/REFRESH_TIME;
int cycle_counter = 0;
unsigned long _now;
unsigned long last_output_refresh_time;
unsigned long last_input_read_time;

float yaw_modifier = 0;
float left_yaw_modifier = 0;
float right_yaw_modifier = 0;

volatile unsigned long v_rising_edge_throttle = 0;
volatile unsigned long v_pulse_time_throttle = 0;
unsigned long pulse_time_throttle;
void calcSignalThrottle() 
{
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(CHANNEL_THROTTLE_PIN) == HIGH) 
    { 
        v_rising_edge_throttle = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(v_rising_edge_throttle != 0)
        { 
            //record the pulse time
            v_pulse_time_throttle = (micros() - v_rising_edge_throttle);
            //Serial.println("Trottle : " + v_pulse_time_throttle);
        }
    } 
}

volatile unsigned long v_rising_edge_yaw = 0;
volatile unsigned long v_pulse_time_yaw = 0;
unsigned long pulse_time_yaw;

void calcSignalYaw() 
{
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(CHANNEL_YAW_PIN) == HIGH) 
    { 
        v_rising_edge_yaw = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(v_rising_edge_yaw != 0)
        { 
            //record the pulse time
            v_pulse_time_yaw = (micros() - v_rising_edge_yaw);
        }
    }
} 

void init_throttle_array(int *array, int init_value)
{
  int i;
  for(i=0; i<THROTTLE_ARRAY_SIZE; i++)
  {
    array[i] = init_value;
  }
}

void update_throttle_array(int *array, int value, int * current_index)
{
  if (*current_index >= THROTTLE_ARRAY_SIZE)
  {
    *current_index = 0;
  }
  array[*current_index] = value;
  *current_index = *current_index + 1;
}

int get_mean_throttle_value(int *array) 
{
  int i, mean = 0;
  for(i=0; i<THROTTLE_ARRAY_SIZE; i++)
  {
    mean += array[i];
  }
  mean = round((float)mean/THROTTLE_ARRAY_SIZE);
  return(mean);
}

Servo left_motor;
Servo right_motor;

void setup() {

  _now = 0;
  last_output_refresh_time = 0;
  last_input_read_time = 0;

  init_throttle_array(t_throttle_left_motor, LEFT_MOTOR_MID);
  init_throttle_array(t_throttle_right_motor, RIGHT_MOTOR_MID);
  
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
  left_motor.writeMicroseconds((int)throttle_left_motor);
  right_motor.writeMicroseconds((int)throttle_right_motor);

  if (_now - last_output_refresh_time > REFRESH_TIME)
  {
    // disable interrupts to avoid werid behavior
    noInterrupts();
    pulse_time_yaw = v_pulse_time_yaw;
    pulse_time_throttle = v_pulse_time_throttle;
    interrupts();

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

    // add new value to arrays
    update_throttle_array(t_throttle_left_motor, (int) pulse_time_left_motor, &index_throttle_left_motor);
    update_throttle_array(t_throttle_right_motor, (int) pulse_time_right_motor, &index_throttle_right_motor);

    // update mean value
    throttle_left_motor = get_mean_throttle_value(t_throttle_left_motor);
    throttle_right_motor = get_mean_throttle_value(t_throttle_right_motor);

    // if time to print logs:
    if (cycle_counter == SERIAL_TIME_CYCLE - 1)
    {
      Serial.print("Yaw : ");
      Serial.println(pulse_time_yaw);
      Serial.print("Trottle : ");
      Serial.println(pulse_time_throttle);
      Serial.print("LEFT throttle : ");
      Serial.println(throttle_left_motor);
      Serial.print("RIGHT throttle : ");
      Serial.println(throttle_right_motor);
      Serial.print("left_yaw_modifier : ");
      Serial.println(left_yaw_modifier);
      Serial.print("right_yaw_modifier : ");
      Serial.println(right_yaw_modifier);
      Serial.println("---------------");


      // SAFETY once in a while, we reset pulse_time measurement (in case we have no signal)
      noInterrupts();
      // reset pulse_time measuration 
      v_rising_edge_throttle = 0;
      v_rising_edge_yaw = 0;
      v_pulse_time_throttle = 0;
      v_pulse_time_yaw = 0;
      interrupts();

      cycle_counter = 0;
    }
    else
    {
      cycle_counter++;
    }
    last_output_refresh_time = _now;
  }
}
