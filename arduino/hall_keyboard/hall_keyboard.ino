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
  // TODO: на самом деле надо будет иметь небольшой промежуток между сенсором и магнитом - 
  // и самая пиковая часть мб и уйдет - и станет ровнее
  // Сейчас все равно несколько криво
  // Или замерять более продолжительно, и строить регрессию не по трем точкам,
  // а по десятку, скажем.

  // calibration
  // double C [3];
  // //y(0) = 504, y(100) = 862, y(50) = 560 // - эти данные считал руками,
  // примерно определяя расстоние
  // double b1 = 504;
  // double b2 = 560;
  // double b3 = 862;
  // linalg_least_squares_cubic(b1, 0, b2, 50, b3, 100, &C[0], &C[1], &C[2]);

  // Serial.println("Start");
  // Serial.println(C[0]);
  // Serial.println(C[1]);
  // Serial.println(C[2]);
  // Serial.println("Solved");
  // Это с широкой точки hall sensor'a
  float C[] = { 14.31, -14.49, 0.45 };

  int hallValue = analogRead(hallSensorPin);
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = hallValue * (5.0 / 1023.0);

  float distance_calc = C[0]*cbrtf(hallValue) + C[1]*sqrtf(hallValue) + C[2]*hallValue;

  // чтобы было положительным, и целым
  float distance = (50 + distance_calc) * 100; 

  if (millis() - timer100 >= 50) {
    timer100 += 50;

    // Serial.print(voltage*100);
    // Serial.print(",");
    Serial.println(getDistance(distance));
    // вольтаж 2.46 - хз почему для экрана надо умножать
    sevseg.setNumber(voltage*10, 1);
  }
  
  // // delay(10)
  // if (millis() - timer10 >= 10) {
  //   timer10 += 10;

  //   // Map the voltage to a MIDI velocity (0 - 127):
  //   int velocity = mapVoltageToVelocity(hallValue);
    
  //   MIDI.sendNoteOn(64, velocity, 1);  // Send a Note with the computed velocity.

  //   // Serial.print("Hall: ");
  //   // Serial.println(velocity);

  //   //delay(1000);  // Wait for a second.
  //   if (velocity <= 2) { 
  //     MIDI.sendNoteOff(64, 0, 1);  // Stop the note.
  //   }
  // }

  sevseg.refreshDisplay(); // Must run repeatedly
}

int getDistance(float linearedHallValue) {
  // Linearly map the voltage value to the MIDI velocity range:
  return constrain(map(linearedHallValue, 2000, 6500, 0, 100), 0, 100);
}

int mapVoltageToVelocity(float voltage) {
  // Linearly map the voltage value to the MIDI velocity range:
  return constrain(map(voltage, minVoltage, maxVoltage, 0, 127), 0, 127);
}



/*
 * algebraic least-squares fit for 5-point cubic equation system
 *
 * y0 = C0*cbrtf(x0) + C1*sqrtf(x0) + C2*x0
 * y1 = C0*cbrtf(x1) + C1*sqrtf(x1) + C2*x1
 * y2 = C0*cbrtf(x2) + C1*sqrtf(x2) + C2*x2
 * y3 = C0*cbrtf(x3) + C1*sqrtf(x3) + C2*x3
 * y4 = C0*cbrtf(x4) + C1*sqrtf(x4) + C2*x4
 *
 * boundary conditions:
 * y0 = x0 = 0
 * y4 = x4 = 1
 *
 * returns C0, C1
 *
 * Mathematica is your friend:
 *  FullSimplify[
 *    Refine[
 *      LeastSquares[{{0, 0, 0}, {a^2, a^3, a^6}, {b^2, b^3, b^6}, {c^2, c^3, c^6}, {1, 1, 1}}, {0, y1, y2, y3, 1}],
 *      {a, b, c} \[Element] Reals
 *    ]
 *  ]
 */
// from chimaera
void
linalg_least_squares_cubic(double x1, double y1, double x2, double y2, double x3, double y3, double *C0, double *C1, double *C2)
{
	double a12= pow(x1, 12.0/6.0);
	double a9 = pow(x1, 9.0/6.0);
	double a8 = pow(x1, 8.0/6.0);
	//double a7 = pow(x1, 7.0/6.0);
	double a6 = pow(x1, 6.0/6.0);
	double a5 = pow(x1, 5.0/6.0);
	double a4 = pow(x1, 4.0/6.0);
	double a3 = pow(x1, 3.0/6.0);
	double a2 = pow(x1, 2.0/6.0);
	//double a  = pow(x1, 1.0/6.0);

	double b12= pow(x2, 12.0/6.0);
	double b9 = pow(x2, 9.0/6.0);
	double b8 = pow(x2, 8.0/6.0);
	double b7 = pow(x2, 7.0/6.0);
	double b6 = pow(x2, 6.0/6.0);
	double b5 = pow(x2, 5.0/6.0);
	double b4 = pow(x2, 4.0/6.0);
	double b3 = pow(x2, 3.0/6.0);
	double b2 = pow(x2, 2.0/6.0);
	double b  = pow(x2, 1.0/6.0);

	double c12= pow(x3, 12.0/6.0);
	double c9 = pow(x3, 9.0/6.0);
	double c8 = pow(x3, 8.0/6.0);
	double c7 = pow(x3, 7.0/6.0);
	double c6 = pow(x3, 6.0/6.0);
	double c5 = pow(x3, 5.0/6.0);
	double c4 = pow(x3, 4.0/6.0);
	double c3 = pow(x3, 3.0/6.0);
	double c2 = pow(x3, 2.0/6.0);
	double c  = pow(x3, 1.0/6.0);

	double divisor = b4*c4*pow(b+b4*(-1+c)-c+c4-b*c4,2.0)+a12*(pow(-1+c,2.0)*c4+b6*(1+c4)-2*b5*(1+c5)+b4*(1+c6))-2*a9*(pow(-1+c,2)*c4*(1+c)*(1+c2)+b9*(1+c4)-b8*(1+c5)-b5*(1+c8)+b4*(1+c9))+2*a8*(pow(-1+c,2.0)*c5*(1+c+c2)+b9*(1+c5)-b8*(1+c6)-b6*(1+c8)+b5*(1+c9))+a6*(c4*pow(-1+c4,2.0)+b12*(1+c4)-2*b8*(1+c8)+b4*(1+c12))-2*a5*(c5+c8*(-1-c+c4)+b12*(1+c5)-b9*(1+c8)-b8*(1+c9)+b5*(1+c12))+a4*(c6*pow(-1+c3,2.0)+b12*(1+c6)-2*b9*(1+c9)+b6*(1+c12));

	*C0 =(a3*((-b12)*(1+c5)+c5*(-1+c3+c4-c7)+b9*(1+c8)+b8*(1+c9)-b5*(1+c12))*y1+a2*(c6*pow(-1+c3,2.0)+b12*(1+c6)-2*b9*(1+c9)+b6*(1+c12))*y1+a12*((-1+b)*b5+(-1+c)*c5+b2*(1+c6-b*(1+c5))*y2+c2*(1+b6-(1+b5)*c)*y3)+a5*(b9-b12+c9-c12+b6*(1+c9)*y2-b3*(1+c12)*y2+c3*(-1-b12+(1+b9)*c3)*y3)+a9*(c5+c8-2*c9+b2*(b3+b6-2*b7+b4*(1+c5)*y2+b*(1+c8)*y2-2*(1+c9)*y2)+c2*(-2-2*b9+c+b8*c+(1+b5)*c4)*y3)+(-1+b)*b2*(-1+c)*c2*(b+b2+b3-c*(1+c+c2))*(c3*(-1+c3)*y2+b6*(c3-y3)+b3*(-c6+y3))+a6*(c12+c5*y1+c9*y1+b9*(1+c5)*y1+b5*(1+c9)*y1-c8*(1+y1)+b2*(1+c12)*y2-b6*(1+c8)*(y1+y2)+c2*y3-c6*(y1+y3)+b12*(1+c2*y3)-b8*(1+y1+c6*y1+c6*y3))+a8*(b3*(1+c9)*y2+c3*(-1+c3)*(c3-y3)+b9*(1+c3*y3)-b6*(1+y2+c6*y2+c6*y3))) / divisor;

	*C1 =(a6*((-b9)*(1+c4)+c4*(-1+c+c4-c5)+b8*(1+c5)+b5*(1+c8)-b4*(1+c9))*y1+a3*(c4*pow(-1+c4,2.0)+b12*(1+c4)-2*b8*(1+c8)+b4*(1+c12))*y1+a2*((-b12)*(1+c5)+c5*(-1+c3+c4-c7)+b9*(1+c8)+b8*(1+c9)-b5*(1+c12))*y1+a4*(c9*(-1+c3)+b3*(-b6+b9+y2+c12*y2-b3*(1+c9)*y2)+c3*(1+b12-(1+b9)*c3)*y3)+a9*(-c4+c8+b2*(-b2+b6+y2+c8*y2-b4*(1+c4)*y2)+c2*(1+b8-(1+b4)*c4)*y3)+a8*(b5-2*b8+b9+c5-2*c8+c9+b2*(1+c9+b4*(1+c5)-2*b*(1+c8))*y2+c2*(1+b9-2*(1+b8)*c+(1+b5)*c4)*y3)+a5*(b8-b12+c8-c12+b6*(1+c8)*y2-b2*(1+c12)*y2+c2*(-1-b12+(1+b8)*c4)*y3)+a12*((-(-1+b))*b4-(-1+c)*c4+b2*(-1+b+b*c4-c5)*y2+c2*(-1+c+b4*(-b+c))*y3)-(-1+b)*b2*(-1+c)*c2*(b+b2+b3-c*(1+c+c2))*(c2*(-1+c4)*y2+b6*(c2-y3)+b2*(-c6+y3))) / divisor;

	*C2 =(a3*((-b9)*(1+c4)+c4*(-1+c+c4-c5)+b8*(1+c5)+b5*(1+c8)-b4*(1+c9))*y1+a2*(pow(-1+c,2.0)*c5*(1+c+c2)+b9*(1+c5)-b8*(1+c6)-b6*(1+c8)+b5*(1+c9))*y1+(-1+b)*b2*(-1+c)*c2*(b+b2+b3-c*(1+c+c2))*(c2*(b3-b2*c+(-1+c)*y2)-(-1+b)*b2*y3)+a9*((-1+b)*b4+(-1+c)*c4+b2*(1+c5-b*(1+c4))*y2+c2*(1+b5-(1+b4)*c)*y3)+a5*(-2*c5+c8+c9+b2*(-2*b3+b6+b7+y2+c9*y2-2*b4*(1+c5)*y2+b*(1+c8)*y2)+c2*(1+b9+c+b8*c-2*(1+b5)*c4)*y3)+a8*((-(-1+b))*b5-(-1+c)*c5+b2*(-1+b+b*c5-c6)*y2+c2*(-1+c+b5*(-b+c))*y3)+a6*(-2*b5*(1+c5)*y1+c4*(1-c4+pow(-1+c,2.0)*y1)-b2*(1+c8)*y2+b6*(1+c4)*(y1+y2)+c2*(-1+c4)*y3-b8*(1+c2*y3)+b4*(1+y1+c6*y1+c6*y3))+a4*((-b3)*(1+c9)*y2-c3*(-1+c3)*(c3-y3)-b9*(1+c3*y3)+b6*(1+y2+c6*y2+c6*y3))) / divisor;
}
