#include <ESP32Servo.h>

int RPL_1_min = 1400;
int RPL_1_max = 1940;
Servo Payload_1;

void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  Payload_1.attach(5);
  Serial.begin(115200);
}

void loop() {
  Payload_1.write(RPL_1_max);
  delay(250);
  Serial.println(analogRead(4));
  if (analogRead(4)>100){
    Serial.println("Unjamming");
    for (int i =0;i<10;i++){
      Payload_1.write(RPL_1_max);
      delay(250);
      Payload_1.write(RPL_1_min);
      delay(80);
    }

  }
  Payload_1.write(RPL_1_min);
  delay(5000);
//  Serial.println(analogRead(4));
//  delay(200);
}
