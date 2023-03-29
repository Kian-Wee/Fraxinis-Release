#pragma once
#include <Arduino.h>
extern bool DEBUGPRINT;

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
