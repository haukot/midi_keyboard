#include <MIDI.h>
#include "SevSeg.h"

SevSeg sevseg;

// Create and bind the MIDI interface to the default hardware Serial port
MIDI_CREATE_DEFAULT_INSTANCE();

const int hallSensorPin = A0;   // The pin number of the Hall sensor.

float minVoltage = 500;        // Minimum expected voltage from the Hall sensor.
float maxVoltage = 650;  // could be up to 800      // Maximum expected voltage from the Hall sensor.


void setup() {
  setup_display();

  pinMode(hallSensorPin, INPUT);  

  MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  Serial.begin(115200); // will change baud rate of MIDI traffic from 31250 to 115200
}

void setup_display() {
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
  static unsigned long timer10 = millis();
  static unsigned long timer100 = millis();

  int hallValue = analogRead(hallSensorPin);
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = hallValue * (5.0 / 1023.0);

  if (millis() - timer100 >= 100) {
    timer100 += 100;

    Serial.println(voltage);
    // вольтаж 2.46 - хз почему для экрана надо умножать
    sevseg.setNumber(voltage*10, 1);
  }
  
  // delay(10)
  if (millis() - timer10 >= 10) {
    timer10 += 10;

    // Map the voltage to a MIDI velocity (0 - 127):
    int velocity = mapVoltageToVelocity(hallValue);
    
    MIDI.sendNoteOn(64, velocity, 1);  // Send a Note with the computed velocity.

    // Serial.print("Hall: ");
    // Serial.println(velocity);

    //delay(1000);  // Wait for a second.
    if (velocity <= 2) { 
      MIDI.sendNoteOff(64, 0, 1);  // Stop the note.
    }
  }

  sevseg.refreshDisplay(); // Must run repeatedly
}

int mapVoltageToVelocity(float voltage) {
  // Linearly map the voltage value to the MIDI velocity range:
  return constrain(map(voltage, minVoltage, maxVoltage, 0, 127), 0, 127);
}
