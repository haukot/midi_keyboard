#include "SevSeg.h"
SevSeg sevseg; //Instantiate a seven segment controller object
//static unsigned long timer = 0;

// const int pins[] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40};
// const int numPins = sizeof(pins) / sizeof(pins[0]);

// void setup() {
//   digitalWrite(2, HIGH);
//   digitalWrite(3, HIGH);

//   for (int i = 0; i < numPins; i++) {
//     pinMode(pins[i], OUTPUT);
//     digitalWrite(pins[i], HIGH);  // Assuming common cathode displays
//   }

//   Serial.begin(9600);
//   Serial.println("Starting test...");
//   delay(2000);  // Give a bit of time before starting
// }

// void loop() {
//   for (int i = 0; i < numPins; i++) {
//     digitalWrite(pins[i], LOW);

//     for (int j = 0; j < numPins; j++) {
//       if (i != j) {
//         digitalWrite(pins[j], HIGH);  // Light up potential segment
//         Serial.print("Testing with pin ");
//         Serial.print(pins[i]);
//         Serial.print(" set to LOW and pin ");
//         Serial.print(pins[j]);
//         Serial.println(" set to HIGH.");
//         delay(5000);  // Give time to observe
//         digitalWrite(pins[j], LOW);  // Reset pin
//       }
//     }

//     digitalWrite(pins[i], HIGH);  // Reset pin after test cycle
//   }
// }

void setup() {
  byte numDigits = 2; 
  byte digitPins[] = {3, 2};
  // A B C D E F G DP
  byte segmentPins[] = {26, 28, 30, 32, 34, 36, 38, 40};
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = N_TRANSISTORS; // See README.md for options
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected
  
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
  updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(90);


  // сегменты освещаются по разному. Мб 22 пин с конденсатором помогал сглаживать мерцания?
  // но хз как его выставить
  // pinMode(22, OUTPUT);  
  pinMode(24, OUTPUT);  
}

void loop() {
  static unsigned long timer = millis();
  static int deciSeconds = 0;

  if (millis() - timer >= 1000) {
    timer += 1000;
    deciSeconds++; // 100 milliSeconds is equal to 1 deciSecond
    
    if (deciSeconds == 100000) { // Reset to 0 after counting for 1000 seconds.
      deciSeconds=0;
    }
    sevseg.setNumber(20 + deciSeconds);
  }
  sevseg.refreshDisplay(); // Must run repeatedly

}
