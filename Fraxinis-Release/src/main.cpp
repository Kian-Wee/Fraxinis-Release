// Remember to run these before first build
// pio lib install # Install dependencies
// pio run # Build the firmware
// pio run --target upload # Flash the firmware

#define ESP32C3
// #define ESP32S3

#include <Arduino.h>
// #include "Wire.h"
// #include <VL53L0X.h>
#include <ESP32Servo.h>

#include <pins.h>
#include <misc_fn.h>
#include <rc.h>
#include <ros.h>

// extern std_msgs__msg__Bool payload_in_msg;
// extern std_msgs__msg__Bool counter_in_msg;
// extern std_msgs__msg__Int32 thrust_in_msg;

#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include "secrets.h"
#include "misc_fn.h"

// rcl_service_impl_t payload_subscriber;
rcl_subscription_t payload_subscriber;
std_msgs__msg__Bool payload_in_msg;
// rcl_publisher_t payload_publisher;
// std_msgs__msg__Bool payload_out_msg;

rcl_subscription_t counter_subscriber;
// rcl_service_impl_t counter_subscriber;
std_msgs__msg__Bool counter_in_msg;
// rcl_publisher_t counter_publisher;
// std_msgs__msg__Bool counter_out_msg;

rcl_subscription_t thrust_subscriber;
std_msgs__msg__Int32 thrust_in_msg;
// rcl_publisher_t thrust_publisher;
// std_msgs__msg__Int32 thrust_out_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop()
{
    // while (1)
    // {
    //     // digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
    //     println("Stuck in RC Check");
    //     delay(100);
    // }
}

// #define RCCHECK(fn)                  \
//     {                                \
//         rcl_ret_t temp_rc = fn;      \
//         if ((temp_rc != RCL_RET_OK)) \
//         {                            \
//             error_loop();            \
//         }                            \
//     }
// #define RCSOFTCHECK(fn)              \
//     {                                \
//         rcl_ret_t temp_rc = fn;      \
//         if ((temp_rc != RCL_RET_OK)) \
//         {                            \
//         }                            \
//     }

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){println("Stuck in RCCHECK");error_loop();}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){println("Stuck in RCSOFTHECK");}}

IPAddress agent_ip(192, 168, 1, 3);
size_t agent_port = 8888;
extern char ssid[];
extern char psk[];

void payload_subscription_callback(const void *msgin)
{
	const std_msgs__msg__Bool *payload_in_msg = (const std_msgs__msg__Bool *)msgin;
}

void counter_subscription_callback(const void *msgin)
{
	const std_msgs__msg__Bool *counter_in_msg = (const std_msgs__msg__Bool *)msgin;
}

void thrust_subscription_callback(const void *msgin)
{
	const std_msgs__msg__Int32 *counter_in_msg = (const std_msgs__msg__Int32 *)msgin;
}

void setupROS()
{

    println("Setting up ROS");
    
    print("Wifi Credentials: "); print(ssid); print("|"); println(psk);

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    println("Added wifi transport");
    allocator = rcl_get_default_allocator();
    println("Added allocator");

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    println("Init options");

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
    println("Created ROS node");

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    println("Created ROS executor");

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &payload_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "payload_drop"));
    RCCHECK(rclc_executor_add_subscription(&executor, &payload_subscriber, &payload_in_msg, &payload_subscription_callback, ON_NEW_DATA));

    RCCHECK(rclc_subscription_init_default(
        &counter_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "counter_drop"));
    RCCHECK(rclc_executor_add_subscription(&executor, &counter_subscriber, &counter_in_msg, &counter_subscription_callback, ON_NEW_DATA));

    RCCHECK(rclc_subscription_init_default(
        &thrust_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "thruster_speed"));
    RCCHECK(rclc_executor_add_subscription(&executor, &thrust_subscriber, &thrust_in_msg, &thrust_subscription_callback, ON_NEW_DATA));
    // RCCHECK(rclc_publisher_init_default(
    //   &thrust_publisher,
    //   &node,
    //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    //   "micro_ros_control_node"));

    // There are 2 main sources of blocking code, the reading of PWM and the release of servo, which are both time dependant
    // If using a dual core S3, seperate these 2 tasks to run on a thread so that the other 2 tasks are responsive
    #ifdef ESP32S3
        xTaskCreatePinnedToCore(doTask0,
                                "Task 0",
                                237680,
                                NULL,
                                1,
                                NULL,
                                pro_cpu);

        // Start Task 1 (in Core 1)
        xTaskCreatePinnedToCore(doTask1,
                                "Task 1",
                                6000,
                                NULL,
                                1,
                                NULL,
                                app_cpu);
    #endif

    println("Finish ROS setup");
}



//-----------------------------------------------------------------------------

// Core definitions for multithreading (assuming you have dual-core ESP32)
static const BaseType_t pro_cpu = 0; // wifi_core
static const BaseType_t app_cpu = 1;

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

const bool DEBUGPRINT =1; //prints debug statement

int secondreleasetimer=2000;
int resettimer=0;


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
    a=1;
  }
}

class servo_class {
  public:
    servo_class(int8_t servo_pin,int PWM_OPEN_OVERWRITE,int PWM_CLOSED_OVERWRITE, int8_t led_pin);
    int PWM_OPEN;
    int PWM_CLOSED;
    Servo servo_obj;
    int state; //Open = 1, Closed = 0
    void WriteState(int8_t new_state);
    int8_t led_pin = led_pin; //TEST THIS
    int store_state; //Used to store a new state temporarily, updated when UpdateState is called
    void UpdateState();
};

servo_class::servo_class(int8_t servo_pin,int PWM_OPEN_OVERWRITE = 1900,int PWM_CLOSED_OVERWRITE = 1100, int8_t led_pin=2){
  servo_obj.attach(servo_pin);
  int PWM_OPEN = PWM_OPEN_OVERWRITE;
  int PWM_CLOSED = PWM_CLOSED_OVERWRITE;
  if (led_pin!=2) pinMode(led_pin,OUTPUT);
}

void servo_class::WriteState(int8_t new_state){
  if(new_state != state){
    if(new_state == 1) servo_obj.write(PWM_OPEN); digitalWrite(led_pin,HIGH); println("SERVO OPEN");
    if(new_state == 0) servo_obj.write(PWM_CLOSED); digitalWrite(led_pin,LOW);
    state=new_state;
  }
}

void servo_class::UpdateState(){
  if(store_state != state){
    if(store_state == 1) servo_obj.write(PWM_OPEN); digitalWrite(led_pin,HIGH);
    if(store_state == 0) servo_obj.write(PWM_CLOSED); digitalWrite(led_pin,LOW);
    state=store_state;
  }
}

servo_class payload(Payload_pin, 1940,1400, PAYLOAD_LED);
servo_class counter(Thruster_pin, 1940,1400, COUNTER_LED);

void setup() {

    Serial.begin(115200);
    println("Microcontroller booting up");
    delay(2000);

    pinMode(PAYLOAD_SWITCH,INPUT_PULLDOWN);
    pinMode(COUNTER_SWITCH,INPUT_PULLDOWN);

    setupRC(RCin_PR); //Payload Release Pin (Disarmed-Offboard-Override)
    setupRC(RCin_TP); // Thruster Pin(Reverse-Disarmed-Forward)
    setupRC(RCin_TM); // Thruster Modifier Pin(20%-80%), necessary to ramp up throttle and prevent jerks
    setupRC(RCin_AP); // Adhesive Pin(Off, Inject Adhesive, Turn on UV Light)

    setupROS();

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

  // There are 2 main sources of blocking code, the reading of PWM and the release of servo, which are both time dependant
  // If using a dual core S3, seperate these 2 tasks to run on a thread so that the other 2 tasks are responsive
  #ifdef ESP32S3
    xTaskCreatePinnedToCore(doTask0,
                "Task 0",
                237680,
                NULL,
                1,
                NULL,
                pro_cpu);

    // Start Task 1 (in Core 1)
    xTaskCreatePinnedToCore(doTask1,
                "Task 1",
                6000,
                NULL,
                1,
                NULL,
                app_cpu);
  #endif

  println("Finish setup");
}

void caseloop(){

  // Opens the servo if button pressed
  if (digitalRead(COUNTER_SWITCH) == 1){
    println("Opening Counter Servo");
    counter.WriteState(1);
    delay(15); // waits for the servo to get there
  }else if (digitalRead(COUNTER_SWITCH) == 0 && counter.state == 1){
    counter.WriteState(0);
    delay(15); // waits for the servo to get there
  }

  if (digitalRead(PAYLOAD_SWITCH) == 1){
    println("Opening Payload Servo");
    payload.WriteState(1);
  } else if (digitalRead(PAYLOAD_SWITCH) == 0 && payload.state == 1){
    payload.WriteState(0);
    delay(15); // waits for the servo to get there
  }
  
  if (payload_in_msg.data == 1) payload.store_state = 1; println("HAHA");
  if (payload_in_msg.data == 0) payload.store_state = 0;  
  if (counter_in_msg.data == 1) counter.store_state = 1;
  if (counter_in_msg.data == 0) counter.store_state = 0;

}

// MultiThreading task 0
void doTask0(void *parameters)
{
	while (1)
	{
    caseloop();
		RCCHECK(rclc_executor_spin(&executor));
	}
}

// MultiThreading task 1
void doTask1(void *parameters)
{
	while (1)
	{
    readRCAll();
    payload.UpdateState();
    counter.UpdateState();
	}
}

void loop(){

// Run single core loop only if ESP32C3 is defined
#ifdef ESP32C3
  readRCAll();
  caseloop();
  payload.UpdateState();
  counter.UpdateState();
  RCCHECK(rclc_executor_spin(&executor));
#endif
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


}
