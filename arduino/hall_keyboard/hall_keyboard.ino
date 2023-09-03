#include <MIDI.h>

// Create and bind the MIDI interface to the default hardware Serial port
MIDI_CREATE_DEFAULT_INSTANCE();

const int hallSensorPin = A0;   // The pin number of the Hall sensor.

float minVoltage = 500;        // Minimum expected voltage from the Hall sensor.
float maxVoltage = 650;  // could be up to 800      // Maximum expected voltage from the Hall sensor.


void setup() {
  MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  Serial.begin(115200); // will change baud rate of MIDI traffic from 31250 to 115200
}

void loop() {
  int hallValue = analogRead(hallSensorPin);

  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  // float voltage = hallValue * (5.0 / 1023.0);


  // Map the voltage to a MIDI velocity (0 - 127):
  int velocity = mapVoltageToVelocity(hallValue);
  
  MIDI.sendNoteOn(64, velocity, 1);  // Send a Note with the computed velocity.

  // Serial.print("Hall: ");
  // Serial.println(velocity);

  //delay(1000);  // Wait for a second.
  if (velocity <= 2) { 
    MIDI.sendNoteOff(64, 0, 1);  // Stop the note.
  }
  
  delay(10);  // Short delay for stability.
}

int mapVoltageToVelocity(float voltage) {
  // Linearly map the voltage value to the MIDI velocity range:
  return constrain(map(voltage, minVoltage, maxVoltage, 0, 127), 0, 127);
}
