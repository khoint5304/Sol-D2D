#include "../include/EnergyComputation.h"

// Theo mô hình linear, công suất P = beta * w + gamma (W)
// Năng lượng E = P * thời gian
// Thời gian cất cánh = cruiseAlt / takeoffSpeed
// Thời gian bay ngang = dist / cruiseSpeed
// Thời gian hạ cánh = cruiseAlt / landingSpeed
double computeLinearEnergy(double w, double beta, double gamma,
                           double cruiseAlt, double takeoffSpeed,
                           double cruiseSpeed, double landingSpeed,
                           double dist)
{
    double time_takeoff = cruiseAlt / takeoffSpeed;
    double time_cruise = dist / cruiseSpeed;
    double time_landing = cruiseAlt / landingSpeed;

    double P = beta * w + gamma; // W

    double E_takeoff = P * time_takeoff;
    double E_cruise = P * time_cruise;
    double E_landing = P * time_landing;

    double E_total = E_takeoff + E_cruise + E_landing;
    return E_total; // Joule
}
