#pragma once

const int8_t RCin_PR = 1; //Payload Release Pin (Disarmed-Offboard-Override)
const int8_t RCin_TP = 2; // Thruster Pin(Reverse-Disarmed-Forward)
const int8_t RCin_TM = 3; // Thruster Modifier Pin(20%-80%), necessary to ramp up throttle and prevent jerks
const int8_t RCin_AP = 9; // Adhesive Pin(Off, Inject Adhesive, Turn on UV Light)

const int8_t P_Switch = 10, C_Switch = 8;
const int8_t P_LED = 6, T_LED=7;
const int8_t Payload_pin = 5, Thruster_pin = 4;