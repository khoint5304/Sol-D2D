#ifndef READDATA_H
#define READDATA_H

#include <string>
#include <vector>
#include <map>
#include "../lib/json.hpp"

using namespace std;

struct Problem
{
    string problem;
    int droneflighttimelimit, customers_count, trucks_count, drones_count;
    vector<double> x;
    vector<double> y;
    vector<double> demand;
    vector<bool> droneable;
    vector<int> serviceTimeByTruck;
    vector<int> serviceTimeByDrone;
};

struct TruckConfig
{
    double maximum_velocity;
    double capacity;
    vector<double> coefficients;

    static TruckConfig import_data();
};

struct Linear_Drone_Config
{
    double takeoffSpeed;
    double cruiseSpeed;
    double landingSpeed;
    double cruiseAlt;
    double capacity;
    double batteryPower;
    string speed_type;
    string range;
    double beta;
    double gamma;

    // Hàm import_data trả về map<int, Linear_Drone_Config>
    static map<int, Linear_Drone_Config> import_data();
};

// Các hàm đọc dữ liệu
Problem readProblemData(const string &problem_name);

#endif // READDATA_H
