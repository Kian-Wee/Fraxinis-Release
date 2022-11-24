#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32_Servo.h>

Servo counterservo;
Servo payloadservo;
VL53L0X sensor;

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
#define COUNTER_SERVO_OPEN 1700
#define COUNTER_SERVO_CLOSED 1000
#define PAYLOAD_LED 6
#define COUNTER_LED 7
#define PAYLOAD_SWITCH 8
#define COUNTER_SWITCH 10

#define WAIT_FOR_DROP 0.9 //(m)waits until the height is below a certain threshold to prevent misfires, set to 0 to disable

#define DEBUGPRINT 1 //prints debug statement

int secondreleasetimer=0;
int resettimer=0;
bool dropbool=0;
String incomingByte; //Using .readstring currently, use int(ascii) for .read

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
void checkanalogvalue(int analogpin){
  Serial.println(analogRead(analogpin));
}

void setup() {

    Serial.begin(115200);

    pinMode(PAYLOAD_SWITCH,INPUT);
    pinMode(COUNTER_SWITCH,INPUT);

    counterservo.attach(COUNTER_SERVO);
    payloadservo.attach(PAYLOAD_SERVO);

    Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init())
    {
    println("Failed to detect and initialize sensor!");
    while (1) {}
    }

    #if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif

    #if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
    #endif
}

void loop(){

    if (Serial.available() > 0) {

    incomingByte = Serial.readStringUntil('\n');
    println("Reading String: " + incomingByte);

      if (incomingByte == "D"){
          dropbool=1;
          }
    }

    // Drop counterweight first
    if (dropbool=1){
      int sensorreading=sensor.readRangeSingleMillimeters();
      Serial.print(sensorreading);
      if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
      Serial.println();
      if (WAIT_FOR_DROP == 0){
        secondreleasetimer = caculatesecondrelease(sensorreading);
        dropbool=0;
      }else{
        if (sensorreading <= WAIT_FOR_DROP){
          secondreleasetimer = caculatesecondrelease(sensorreading);
          dropbool=0;
        }
      }
    }

    // Release payloadweight at right time
    if (millis() >= secondreleasetimer){
        payloadservo.write(PAYLOAD_SERVO_OPEN);
        delay(15); // waits for the servo to get there
        resettimer=millis()+1000;
    }

    // Closes the servo after release
    if (millis() >= resettimer){
        counterservo.write(COUNTER_SERVO_CLOSED);
        payloadservo.write(PAYLOAD_SERVO_CLOSED);
        delay(15); // waits for the servo to get there
        resettimer= millis();
    }

    // Opens the servo if button pressed
    if (digitalRead(COUNTER_SWITCH) == 1){
        counterservo.write(COUNTER_SERVO_OPEN);
        delay(15); // waits for the servo to get there
        dropbool=0;
    }
    if (digitalRead(PAYLOAD_SWITCH) == 1){
        payloadservo.write(PAYLOAD_SERVO_OPEN);
        delay(15); // waits for the servo to get there
        dropbool=0;
    }

}


