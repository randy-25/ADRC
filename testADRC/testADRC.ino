#include "ADRC.h"
#include <Arduino.h>

// Define ADRC object
ADRC adrc;
MatrixXf xhat;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  
  // Initialize ADRC with example parameters
  float t_sampling = 0.01;  // Sampling time in seconds
  float b0 = 3.0;           // K/T^2 dari K/(T^2s^2 + 2DT s + 1) fungsi transfer plant
  float t_settling = 1.0;   // Settling time in seconds
  int k_eso = 4;            // ESO tuning parameter

  

  initADRC(&adrc, t_sampling, b0, t_settling, k_eso, -255, 255); // batas PWM 8 bit
}

void loop() {
  // Check if data is available
  if (Serial.available() > 0) {
    // Read plant output (e.g., position) from Python
    float plant_output = Serial.parseFloat();

    // Compute control signal using ADRC
    float setpoint = 1.0; // Example setpoint
    updateEso(&adrc, plant_output, adrc.u);
    float control_signal = computeControlSignal(&adrc, setpoint);
    xhat = getXHAT(&adrc);

    // Send control signal back to Python
    Serial.println(control_signal);
    Serial.print(xhat(0,0));
    Serial.print(", ");
    Serial.print(xhat(1,0));
    Serial.print(", ");
    Serial.println(xhat(2,0));
  }

  delay(10); // Sampling interval
}
