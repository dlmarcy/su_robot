/*
  test_LED_PB SU 6/4/2023
*/

int LED_GRN1 = 14; // SOC LEDs
int LED_RED1 = 15;
int LED_GRN2 = 22; // Current LEDs
int LED_RED2 = 23;
int Teensy_LED = 13;
int Push_But = 21;

void setup() {
  analogWrite(LED_RED1, 0); // red SOC LED
  analogWrite(LED_GRN1, 0); // green SOC LED
  analogWrite(LED_RED2, 0); // red current LED
  analogWrite(LED_GRN2, 0); // green current LED
  pinMode(Teensy_LED, OUTPUT); // Teensy LED
  digitalWrite(Teensy_LED, LOW); 
  pinMode(Push_But, INPUT_PULLDOWN); // Pushbutton
}

void loop() {
  // Test the LEDs
  // Test the SOC LED
  Serial.println("Watch the LEDs");
  Serial.println();
  for(int i=0; i<=255; i++) {
    analogWrite(LED_RED1, i); // red SOC LED
    analogWrite(LED_GRN1, 255-i); // green SOC LED
    delay(10);
  }
  analogWrite(LED_RED1, 0); // red SOC LED
  analogWrite(LED_GRN1, 0); // green SOC LED
  
  // Test the current LED
  for(int i=0; i<=255; i++) {
    analogWrite(LED_GRN2, i); // green current LED
    delay(5);
  }
  for(int i=0; i<=255; i++) {
    analogWrite(LED_GRN2, 255-i); // green current LED
    delay(5);
  }
  for(int i=0; i<=255; i++) {
    analogWrite(LED_RED2, i); // red current LED
    delay(5);
  }
  for(int i=0; i<=255; i++) {
    analogWrite(LED_RED2, 255-i); // red current LED
    delay(5);
  }
  analogWrite(LED_RED2, 0); // red current LED
  analogWrite(LED_GRN2, 0); // green current LED
  delay(2000);
 
  // Test the Teensy_LED
  digitalWrite(Teensy_LED, HIGH); 
  delay(1000);
  digitalWrite(Teensy_LED, LOW); 

  // Test the pushbutton
  Serial.println("Press the pushbutton");
  for(int i=0; i<=20; i++) {
    int pb = digitalRead(Push_But);
    Serial.print(pb);
    delay(250);
  }
  Serial.println();
  Serial.println();
  delay(2000);
}
