#include <iostream>
#include <fstream>
#include "ADRCMatrix.h"

// Plant parameters
float a0 = 1.0, a1 = 0.5, b0 = 0.1; // Plant transfer function coefficients
float Ts = 0.01; // Sampling time (s)

// ADRC parameters
float control_bw = 5.0;  // Control bandwidth
float observer_bw = 25.0; // Observer bandwidth

int main() {
    // ADRC object
    ADRCMatrix adrc(b0, Ts, control_bw, observer_bw);

    // Simulation parameters
    float x1 = 0.0, x2 = 0.0; // Plant state variables
    float u = 0.0;            // Control input
    float y = 0.0;            // Plant output
    float setpoint = 1.0;     // Desired output
    float simulationTime = 5.0; // Total simulation time
    int steps = simulationTime / Ts;

    // Log output to a file
    std::ofstream outputFile("adrc_simulation.csv");
    outputFile << "Time,Output,Control\n";

    // Simulation loop
    for (int i = 0; i < steps; i++) {
        float t = i * Ts;

        // Simulate plant dynamics
        float x1_next = x1 + Ts * x2;
        float x2_next = x2 + Ts * (-a1 * x2 - a0 * x1 + b0 * u);

        x1 = x1_next;
        x2 = x2_next;
        y = x1;

        // Update ADRC
        adrc.updateESO(y, u);
        u = adrc.computeControl(setpoint, y);

        // Log the results
        outputFile << t << "," << y << "," << u << "\n";

        // Print progress
        if (i % 100 == 0) {
            std::cout << "Time: " << t << " s, Output: " << y << ", Control: " << u << std::endl;
        }
    }

    outputFile.close();
    std::cout << "Simulation complete. Results saved to 'adrc_simulation.csv'." << std::endl;
    return 0;
}
