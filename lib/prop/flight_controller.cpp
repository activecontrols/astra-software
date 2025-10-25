#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
using namespace std;

/*
    Flight Controller: Thrust & Torque Distribution
    Purpose: Calculates thrust split between two propellers based on total thrust and torque
             It can run in a simulation loop to mimic real-time flight control.
*/

// Struct to hold thrust allocation
struct ThrustDistribution {
    double leftThrust;
    double rightThrust;
};

// Flight Controller class
class FlightController {
private:
    double maxThrust; // max thrust per prop (lbs)
    double armLength; // distance between propellers (ft)

public:
    FlightController(double maxThrust, double armLength)
        : maxThrust(maxThrust), armLength(armLength) {}

    // Core function: calculates thrust distribution based on desired torque
    ThrustDistribution calculateDistribution(double totalThrust, double torque) {
        ThrustDistribution dist;

        // Torque balance formula: torque = (right - left) * (armLength / 2)
        double thrustDiff = (2 * torque) / armLength;
        dist.rightThrust = (totalThrust + thrustDiff) / 2.0;
        dist.leftThrust  = (totalThrust - thrustDiff) / 2.0;

        // Clamp thrusts to physical limits
        dist.leftThrust = max(0.0, min(dist.leftThrust, maxThrust));
        dist.rightThrust = max(0.0, min(dist.rightThrust, maxThrust));

        return dist;
    }

    // Print thrust distribution
    void printDistribution(double totalThrust, double torque) {
        ThrustDistribution d = calculateDistribution(totalThrust, torque);
        double leftPercent  = (d.leftThrust / totalThrust) * 100;
        double rightPercent = (d.rightThrust / totalThrust) * 100;

        cout << fixed << setprecision(2);
        cout << "Total Thrust: " << totalThrust << " lbs | ";
        cout << "Torque: " << torque << " lb*ft | ";
        cout << "Left: " << d.leftThrust << " lbs (" << leftPercent << "%) | ";
        cout << "Right: " << d.rightThrust << " lbs (" << rightPercent << "%)\n";
    }
};

int main() {
    // Initialize controller: max thrust per prop = 200 lbs, arm length = 2 ft
    FlightController fc(200.0, 2.0);

    // Simulation inputs (can replace with live sensor data later)
    double thrustInputs[] = {100.0, 100.0, 50.0, 50.0};
    double torqueInputs[] = {0.0, 5.0, 0.0, 5.0};

    cout << "=== Thrust & Torque Simulation ===\n";

    for (int i = 0; i < 4; i++) {
        fc.printDistribution(thrustInputs[i], torqueInputs[i]);
        this_thread::sleep_for(chrono::seconds(1));
    }

    cout << "=== Simulation Complete ===\n";
    return 0;
}
