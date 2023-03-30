/**
 * This class reads the various PWM RC inputs and stores it
 * 
 */
#pragma once
#include <ESP32PWM.h>

void setupRC(const int8_t RC_pin);

int readRC(const int8_t RC_pin);

void readRCAll();