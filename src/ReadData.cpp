#include "../include/ReadData.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

TruckConfig TruckConfig::import_data()
{
    fs::path ROOT = fs::current_path();
    fs::path truck_config_file = ROOT / "D2D Problem" / "config_parameter" / "Truck_config.json";

    ifstream file(truck_config_file);
    if (!file.is_open())
    {
        throw runtime_error("Cannot open file: " + truck_config_file.string());
    }

    nlohmann::json data;
    file >> data;

    TruckConfig config;
    config.maximum_velocity = data["V_max (m/s)"];
    config.capacity = data["M_t (kg)"];
    for (const auto &[key, value] : data["T (hour)"].items())
    {
        config.coefficients.push_back(value);
    }

    return config;
}

map<int, Linear_Drone_Config> Linear_Drone_Config::import_data()
{
    fs::path ROOT = fs::current_path();
    fs::path drone_config_file = ROOT / "D2D Problem" / "config_parameter" / "drone_linear_config.json";

    ifstream file(drone_config_file);
    if (!file.is_open())
    {
        throw runtime_error("Cannot open file: " + drone_config_file.string());
    }

    nlohmann::json data;
    file >> data;

    map<int, Linear_Drone_Config> configs;
    for (const auto &[key, value] : data.items())
    {
        int drone_id = stoi(key);
        Linear_Drone_Config config;
        config.takeoffSpeed = value["takeoffSpeed [m/s]"];
        config.cruiseSpeed = value["cruiseSpeed [m/s]"];
        config.landingSpeed = value["landingSpeed [m/s]"];
        config.cruiseAlt = value["cruiseAlt [m]"];
        config.capacity = value["capacity [kg]"];
        config.batteryPower = value["batteryPower [Joule]"];
        config.speed_type = value["speed_type"];
        config.range = value["range"];
        config.beta = value["beta(w/kg)"];
        config.gamma = value["gamma(w)"];

        configs[drone_id] = config;
    }

    return configs;
}

Problem readProblemData(const string &problem_name)
{
    fs::path ROOT = fs::current_path();
    fs::path problem_file = ROOT / "D2D Problem" / "random_data" / (problem_name + ".txt");

    Problem problem;
    ifstream file(problem_file);
    if (!file.is_open())
    {
        throw runtime_error("Cannot open file: " + problem_file.string());
    }

    string line;
    file >> line >> problem.trucks_count;
    file >> line >> problem.drones_count;
    file >> line >> problem.droneflighttimelimit;
    file >> line >> problem.customers_count;
    while (file >> line)
    {
        if (line == "ServiceTimeByDrone(s)")
        {
            break;
        }
    }
    double x, y, demand;
    bool droneable;
    int TruckServiceTime, DroneServiceTime;
    while (file >> x >> y >> demand >> droneable >> TruckServiceTime >> DroneServiceTime)
    {
        problem.x.push_back(x);
        problem.y.push_back(y);
        problem.demand.push_back(demand);
        problem.droneable.push_back(droneable);
        problem.serviceTimeByTruck.push_back(TruckServiceTime);
        problem.serviceTimeByDrone.push_back(DroneServiceTime);
    }

    return problem;
}
