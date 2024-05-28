/*
  test_Motors SU 6/4/2023
*/

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

int Mot_L_EN = 4;
int Mot_L_PH = 5;
int Mot_R_EN = 3;
int Mot_R_PH = 2;
int Encode_L1 = 17;
int Encode_L2 = 16;
int Encode_R1 = 0;
int Encode_R2 = 1;

long encoder_l;
long encoder_r;
long previous_l = 0;
long previous_r = 0;

Encoder encoder_left(Encode_L1, Encode_L2); 
Encoder encoder_right(Encode_R2, Encode_R1); 

int test_Speed = 200;
int test_Duration = 1000;

void setup() {
  analogWrite(Mot_L_EN, 0); // left speed pin
  pinMode(Mot_L_PH, OUTPUT);  // left direction pin
  digitalWrite(Mot_L_PH, HIGH);  // left forward
  analogWrite(Mot_R_EN, 0); // right speed pin
  pinMode(Mot_R_PH, OUTPUT);  //right direction pin
  digitalWrite(Mot_R_PH, LOW);  // right forward
  encoder_left.write(0);
  encoder_right.write(0);
}

void loop() {
  // Move the left wheel forward for 2 seconds
  Serial.println("Left wheel forward encoder values:");
  digitalWrite(Mot_L_PH, HIGH);  // left forward
  analogWrite(Mot_L_EN, test_Speed); // left speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_l = encoder_left.read();
    Serial.println(encoder_l);
  }
  analogWrite(Mot_L_EN, 0); // left speed pin
  Serial.println();
  delay(2000);
  // Move the left wheel backward for 2 seconds
  Serial.println("Left wheel backward encoder values:");
  digitalWrite(Mot_L_PH, LOW);  // left backward
  analogWrite(Mot_L_EN, test_Speed); // left speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_l = encoder_left.read();
    Serial.println(encoder_l);
  }
  analogWrite(Mot_L_EN, 0); // left speed pin
  Serial.println();
  delay(2000);
  // Move the right wheel forward for 2 seconds
  Serial.println("Right wheel forward encoder values:");
  digitalWrite(Mot_R_PH, LOW);  // right forward
  analogWrite(Mot_R_EN, test_Speed); // right speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_r = encoder_right.read();
    Serial.println(encoder_r);
  }
  analogWrite(Mot_R_EN, 0); // right speed pin
  Serial.println();
  delay(2000);
  // Move the right wheel backward for 2 seconds
  Serial.println("Right wheel backward encoder values:");
  digitalWrite(Mot_R_PH, HIGH);  // right backward
  analogWrite(Mot_R_EN, test_Speed); // right speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_r = encoder_right.read();
    Serial.println(encoder_r);
  }
  analogWrite(Mot_R_EN, 0); // right speed pin
  Serial.println();
  delay(2000);
}
