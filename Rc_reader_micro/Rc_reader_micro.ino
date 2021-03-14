#include <Servo.h>
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.

/*
TODO : 

1) Add servo output
2) add safety mecanism (stop if no input for some time)

*/

const int CHANNEL_YAW_PIN = 2;
const int CHANNEL_THROTTLE_PIN = 3;

const int CHANNEL_LEFT_MOTOR = 10;
const int CHANNEL_RIGHT_MOTOR = 9;

Servo left_motor;
Servo right_motor;

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(CHANNEL_YAW_PIN,INPUT);
  pinMode(CHANNEL_THROTTLE_PIN,INPUT);
  pinMode(13, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(CHANNEL_YAW_PIN), calcSignalYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_THROTTLE_PIN), calcSignalThrottle, CHANGE);

  left_motor.attach(CHANNEL_LEFT_MOTOR);
  right_motor.attach(CHANNEL_RIGHT_MOTOR);

  Serial.println("Ready");
}

unsigned long last_blink = 0, _now;
const unsigned long BLINK_DURATION = 250;
const unsigned long BLINK_PERIOD = 10;

byte LED_STATE = LOW;

void loop() {

  _now = millis();

   left_motor.writeMicroseconds((int)1500);
   right_motor.writeMicroseconds((int)1500);
  
/*
  // read input pwm from throttle and yaw
  pulse_time_throttle = pulseIn(CHANNEL_THROTTLE_PIN, HIGH);
  pulse_time_yaw = pulseIn(CHANNEL_YAW_PIN,HIGH);

  LED_STATE = !LED_STATE;
  digitalWrite(13, LED_STATE);

  Serial.print("Yaw : ");
  Serial.println(pulse_time_yaw);
  Serial.print("Trottle : ");
  Serial.println(pulse_time_throttle);
*/

  if (_now - last_blink < BLINK_PERIOD) {
    /*
    if (_now - last_blink > BLINK_DURATION) {
      digitalWrite(13, LOW);
    }
    */
  } else {
    //digitalWrite(13, HIGH);
    
    last_blink = _now;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      Serial.print("Yaw : ");
      Serial.println(pulse_time_yaw);
      Serial.print("Trottle : ");
      Serial.println(pulse_time_throttle);
    }
  }


  
}
