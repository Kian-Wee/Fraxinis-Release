// Remember to run these before first build
// pio lib install # Install dependencies
// pio run # Build the firmware
// pio run --target upload # Flash the firmware


#include <Arduino.h>
// #include "Wire.h"
// #include <VL53L0X.h>
#include <ESP32_Servo.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include <rc.cpp>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

const int8_t RCin_PR = 1; //Payload Release Pin (Disarmed-Offboard-Override)
const int8_t RCin_TP = 2; // Thruster Pin(Reverse-Disarmed-Forward)
const int8_t RCin_TM = 3; // Thruster Modifier Pin(20%-80%), necessary to ramp up throttle and prevent jerks
const int8_t RCin_AP = 9; // Adhesive Pin(Off, Inject Adhesive, Turn on UV Light)

const int8_t P_Switch = 10, C_Switch = 8;
const int8_t P_LED = 6, T_LED=7;
const int8_t Payload_pin = 5, Thruster_pin = 4;

//-----------------------------------------------------------------------------

/**
Uncomment this line to use long range mode. This
increases the sensitivity of the sensor and extends its
potential range, but increases the likelihood of getting
an inaccurate reading because of reflections from objects
other than the intended target. It works best in dark
conditions.
 */

//#define LONG_RANGE

/**
Uncomment ONE of these two lines to get
- higher speed at the cost of lower accuracy OR
- higher accuracy at the cost of lower speed
*/

//#define HIGH_SPEED
#define HIGH_ACCURACY




#define PAYLOAD_SERVO 4
#define PAYLOAD_SERVO_OPEN 1700
#define PAYLOAD_SERVO_CLOSED 1000
#define COUNTER_SERVO 5
#define COUNTER_SERVO_OPEN 1000
#define COUNTER_SERVO_CLOSED 1800
#define PAYLOAD_LED 7
#define COUNTER_LED 6
#define PAYLOAD_SWITCH 8
#define COUNTER_SWITCH 10

#define WAIT_FOR_DROP 0 //(m)waits until the height is below a certain threshold to prevent misfires, set to 0 to disable

#define DEBUGPRINT 1 //prints debug statement

int secondreleasetimer=2000;
int resettimer=0;
int payloadstate=0; //1 -> open, 0 -> closed
int counterstate=0;
bool dropbool=0;

//print function to reduce serial overhead at runtime
void print(String message, bool debug=DEBUGPRINT){
  if(debug==1){
    Serial.println(message);
  }
}

void println(String message, bool debug=DEBUGPRINT){
  if(debug==1){
    Serial.println(message);
  }
}

/**
 * caculates timing for second payload release, in this case the counterweight drops first to give time to wrap around the post
 * assuming the lidar sensor is around the middle of the payload and the counterweight,
 * the payload is dropped after x(ms) delay later than the counterweight to allow time for the counter weight to wrap
 * the function then returns the timing at which the payload should be dropped
 * 
 */
float caculatesecondrelease(float distance){
  float counterspeed=1;
  return millis() + 1000; //temporary
}

//simple test function
bool a =0;
void testdrop(int droptime=10){

  if (millis() > 3*1000 && a==0){
    println("TESTING DROP");
    dropbool=1;
    a=1;
  }
}

void setup() {

    Serial.begin(115200);
    println("Microcontroller booting up");

    pinMode(PAYLOAD_SWITCH,INPUT_PULLDOWN);
    pinMode(COUNTER_SWITCH,INPUT_PULLDOWN);

    pinMode(PAYLOAD_LED,OUTPUT);
    pinMode(COUNTER_LED,OUTPUT);

    servo_class payload(Payload_pin, 1940,1400);
    servo_class counter(Thruster_pin, 1940,1400);

    setupRC(RCin_PR); //Payload Release Pin (Disarmed-Offboard-Override)
    setupRC(RCin_TP); // Thruster Pin(Reverse-Disarmed-Forward)
    setupRC(RCin_TM); // Thruster Modifier Pin(20%-80%), necessary to ramp up throttle and prevent jerks
    setupRC(RCin_AP); // Adhesive Pin(Off, Inject Adhesive, Turn on UV Light)

    // Wire.begin(8,9);

    // sensor.setTimeout(500);
    // if (!sensor.init())
    // {
    // println("Failed to detect and initialize sensor!");
    // while (1) {}
    // }

    // #if defined LONG_RANGE
    // // lower the return signal rate limit (default is 0.25 MCPS)
    // sensor.setSignalRateLimit(0.1);
    // // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    // sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    // sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    // #endif

    // #if defined HIGH_SPEED
    // // reduce timing budget to 20 ms (default is about 33 ms)
    // sensor.setMeasurementTimingBudget(20000);
    // #elif defined HIGH_ACCURACY
    // // increase timing budget to 200 ms
    // sensor.setMeasurementTimingBudget(200000);
    // #endif
}

void loop(){

  readRC(RCin_PR); //Payload Release Pin (Disarmed-Offboard-Override)
  readRC(RCin_TP); // Thruster Pin(Reverse-Disarmed-Forward)
  readRC(RCin_TM); // Thruster Modifier Pin(20%-80%), necessary to ramp up throttle and prevent jerks
  readRC(RCin_AP); // Adhesive Pin(Off, Inject Adhesive, Turn on UV Light)


//     if (Serial.available() > 0) {

//     incomingByte = Serial.readStringUntil('\n');
//     println("Reading String: " + incomingByte);

//       if (incomingByte == "D"){
//           dropbool=1;
//           }
//     }

//     // Drop counterweight first
//     if (dropbool==1){
//       // int sensorreading=sensor.readRangeSingleMillimeters();
//       // Serial.print(sensorreading);
//       // if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
//       // Serial.println();
//       int sensorreading=WAIT_FOR_DROP;
//       if (WAIT_FOR_DROP == 0){
//         secondreleasetimer = caculatesecondrelease(sensorreading);
//         dropbool=0;
//         println("Releasing Counterweight");
//         counterservo.write(COUNTER_SERVO_OPEN);
//         digitalWrite(COUNTER_LED,HIGH);
//       }else{
//         if (sensorreading <= WAIT_FOR_DROP){
//           secondreleasetimer = caculatesecondrelease(sensorreading);
//           dropbool=0;
//           println("Releasing Counterweight");
//           counterservo.write(COUNTER_SERVO_OPEN);
//           digitalWrite(COUNTER_LED,HIGH);
//         }
//       }
//     }

//     // Release payloadweight at right time
//     if (millis() >= secondreleasetimer && secondreleasetimer != -1){
//       println("Releasing Payload");
//       payloadservo.write(PAYLOAD_SERVO_OPEN);
//       digitalWrite(PAYLOAD_LED,HIGH);
//       delay(15); // waits for the servo to get there
//       secondreleasetimer=-1;
//       resettimer=millis()+1000;
//     }

//     // Closes the servo after release
//     if (resettimer >= millis()){
//       println("Resetting Servos");
//       counterservo.write(COUNTER_SERVO_CLOSED);
//       digitalWrite(COUNTER_LED,LOW);
//       payloadservo.write(PAYLOAD_SERVO_CLOSED);
//       digitalWrite(PAYLOAD_LED,LOW); 
//       delay(15); // waits for the servo to get there
//       resettimer=0;
//     }

    // Opens the servo if button pressed
    if (digitalRead(COUNTER_SWITCH) == 1){
      println("Opening Counter Servo");
    //   counterservo.write(COUNTER_SERVO_OPEN);
	    counterservo.write(1000);
      digitalWrite(COUNTER_LED,HIGH);
      delay(15); // waits for the servo to get there
      counterstate=1;
    }else if (digitalRead(COUNTER_SWITCH) == 0 && counterstate == 1){
      counterservo.write(COUNTER_SERVO_CLOSED);
      digitalWrite(COUNTER_LED,LOW); 
      counterstate=0;
    }

    if (digitalRead(PAYLOAD_SWITCH) == 1){
      println("Opening Payload Servo");
      payloadservo.write(PAYLOAD_SERVO_OPEN);
      digitalWrite(PAYLOAD_LED,HIGH); 
      delay(15); // waits for the servo to get there
	  payloadstate=1;
    } else if (digitalRead(PAYLOAD_SWITCH) == 0 && payloadstate == 1){
      payloadservo.write(PAYLOAD_SERVO_CLOSED);
      digitalWrite(PAYLOAD_LED,LOW); 
      payloadstate=0;
    }
}

class servo_class {
  public:
    servo_class(int8_t servo_pin,int PWM_OPEN_OVERWRITE,int PWM_CLOSED_OVERWRITE);
    int PWM_OPEN;
    int PWM_CLOSED;
    Servo servo_obj;
    int state; //Open = 1, Closed = 0
    void WriteState(int8_t new_state);
};

servo_class::servo_class(int8_t servo_pin,int PWM_OPEN_OVERWRITE = 1900,int PWM_CLOSED_OVERWRITE = 1100){
  servo_obj.attach(servo_pin);
  int PWM_OPEN = PWM_OPEN_OVERWRITE;
  int PWM_CLOSED = PWM_CLOSED_OVERWRITE;
}


void servo_class::WriteState(int8_t new_state){
  if(new_state != state){
    if(new_state == 1) servo_obj.write(PWM_OPEN);
    if(new_state == 0) servo_obj.write(PWM_CLOSED);
    state=new_state;
  }
}