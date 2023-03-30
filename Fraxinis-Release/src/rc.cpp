/**
 * This class reads the various PWM RC inputs and stores it
 * 
 */
#include <ESP32PWM.h>
#include <pins.h>

void setupRC(const int8_t RC_pin){
    pinMode(RC_pin, INPUT); // Manual Overide Pin (1 Stage)
}

int readRC(const int8_t RC_pin){
    return pulseIn(RC_pin, HIGH, 200); // Payload Release Pin (Disarmed-Offboard-Override)
}

void readRCAll(){
  readRC(RCin_PR); //Payload Release Pin (Disarmed-Offboard-Override)
  readRC(RCin_TP); // Thruster Pin(Reverse-Disarmed-Forward)
  readRC(RCin_TM); // Thruster Modifier Pin(20%-80%), necessary to ramp up throttle and prevent jerks
  readRC(RCin_AP); // Adhesive Pin(Off, Inject Adhesive, Turn on UV Light)
}