#pragma GCC optimize("O3")

#include "lib/json.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <limits>
#include <numeric>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <omp.h>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <atomic>
#include <mutex>
#include <csignal>

using namespace std;
namespace fs = std::filesystem;

// struct
struct Problem
{
    string problem;
    int droneflighttimelimit, customers_count, trucks_count, drones_count;
    vector<double> x;
    vector<double> y;
    vector<double> demand;
    vector<bool> droneable;
};

struct Results
{
    vector<vector<int>> truck_paths; // Mỗi vector<int> là tuyến đường của 1 truck
    vector<vector<int>> drone_paths; // Mỗi vector<int> là tuyến đường của 1 drone
    double cost;
};

struct TruckConfig
{
    double maximum_velocity;     // V_max (m/s)
    double capacity;             // M_t (kg)
    vector<double> coefficients; // Coefficients cho từng khung giờ

    // Hàm static để import dữ liệu từ file JSON
    static TruckConfig import_data(const string &filepath)
    {
        // Mở file JSON
        ifstream file(filepath);
        if (!file.is_open())
        {
            throw runtime_error("Không thể mở file " + filepath);
        }

        // Đọc và parse file JSON
        nlohmann::json data;
        file >> data;

        // Lấy thông tin từ JSON
        double max_velocity = data["V_max (m/s)"];
        double capacity = data["M_t (kg)"];

        // Lấy coefficients từ "T (hour)"
        vector<double> coefficients;
        for (const auto &[key, value] : data["T (hour)"].items())
        {
            coefficients.push_back(value);
        }

        // Trả về đối tượng TruckConfig
        return TruckConfig{
            max_velocity,
            capacity,
            coefficients};
    }
};

struct Truck_config
{
    double max_spped;
    double capacity;
    vector<double> coefficients;
};

Truck_config readTruckConfig(const string &TruckConfigName)
{
    Truck_config data;

    // define ROOT
    fs::path ROOT = fs::current_path();

    // problem path
    fs::path truck_config_file = ROOT / "D2D Problem" / "truck_config" / (TruckConfigName + ".jason");

    ifstream infile(truck_config_file);
    if (!infile.is_open())
    {
        throw runtime_error("Không thể mở file " + truck_config_file.string());
    }

    // Đọc và parse file JSON
    nlohmann::json json_data;
    infile >> json_data;

    data.max_spped = json_data["V_max (m/s)"];
    data.capacity = json_data["M_t (kg)"];

    for (const auto &[key, value] : json_data["T (hour)"].items())
    {
        data.coefficients.push_back(value);
    }

    return data;
}

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

    static vector<Linear_Drone_Config> import_data(const string &filepath)
    {
        vector<Linear_Drone_Config> configs;

        // define ROOT
        fs::path ROOT = fs::current_path();

        // problem path
        fs::path drone_config_file = ROOT / "D2D Problem" / "config_parameter" / filepath;

        ifstream infile(drone_config_file);
        if (!infile.is_open())
        {
            throw runtime_error("Không thể mở file " + drone_config_file.string());
        }

        // Đọc và parse file JSON
        nlohmann::json json_data;
        infile >> json_data;

        for (const auto &[key, value] : json_data.items())
        {
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

            configs.push_back(config);
        }

        return configs;
    }
};

Problem readProblemData(const string &problem_name)
{
    Problem data;

    // define ROOT
    fs::path ROOT = fs::current_path();

    // problem path
    fs::path problem_file = ROOT / "D2D Problem" / "random_data" / (problem_name + ".txt");

    ifstream infile(problem_file);
    if (!infile.is_open())
    {
        cerr << "Error: Cannot open file " << problem_file << endl;
        exit(EXIT_FAILURE);
    }

    string line;

    // Đọc dữ liệu metadata từ đầu file
    infile >> line >> data.trucks_count;
    infile >> line >> data.drones_count;
    infile >> line >> data.droneflighttimelimit; // Đọc drone flight time limit
    infile >> line >> data.customers_count;

    // Đọc dữ liệu chi tiết về khách hàng
    double x, y, demand;
    bool droneable;
    while (infile >> x >> y >> demand >> droneable)
    {
        data.x.push_back(x);
        data.y.push_back(y);
        data.demand.push_back(demand);
        data.droneable.push_back(droneable);
    }

    return data;
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        cerr << "Usage: " << argv[0] << " <problem_file> -d <linear|non-linear>" << endl;
        return EXIT_FAILURE;
    }

    string problem_file = argv[1];
    string drone_energy_model;

    // Parse command-line arguments
    for (int i = 2; i < argc; ++i)
    {
        string arg = argv[i];
        if (arg == "-d" && i + 1 < argc)
        {
            drone_energy_model = argv[++i];
        }
    }

    if (drone_energy_model.empty())
    {
        cerr << "Error: Drone energy model (-d) must be specified as 'linear' or 'non-linear'." << endl;
        return EXIT_FAILURE;
    }

    try
    {
        // Read problem data
        Problem problem = readProblemData(problem_file);

        // Handle different drone energy models
        if (drone_energy_model == "linear")
        {
            cout << "Using linear drone energy model." << endl;
            // Implement linear drone model handling here
        }
        else if (drone_energy_model == "non-linear")
        {
            cout << "Using non-linear drone energy model." << endl;
            // Implement non-linear drone model handling here
        }
        else
        {
            cerr << "Error: Invalid drone energy model specified." << endl;
            return EXIT_FAILURE;
        }

        // Continue with computations and results
    }
    catch (const exception &e)
    {
        cerr << "Error: " << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}