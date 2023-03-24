/**
 * This class reads the various PWM RC inputs and stores it
 * 
 */
#include <ESP32PWM.h>

void setupRC(const int8_t RC_pin){
    pinMode(RC_pin, INPUT); // Manual Overide Pin (1 Stage)
}

int readRC(const int8_t RC_pin){
    return pulseIn(RC_pin, HIGH); // Payload Release Pin (Disarmed-Offboard-Override)
}