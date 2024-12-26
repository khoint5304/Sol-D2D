#include <iostream>
#include <string>
#include <stdexcept>
#include "include/ReadData.h"
#include "include/Result.h"

using namespace std;

extern void solveGA(const Problem &problem, const TruckConfig &truckConfig, const std::vector<Linear_Drone_Config> &droneConfigs,
                    int K, int D, int pop_size, int generations, double mutation_rate);

int main(int argc, char *argv[])
{
    int dronetype = -1;
    string problem_name;

    if (argc < 4)
    {
        cerr << "Usage: " << argv[0] << " -dronetype <1|2|3|4> <problem_file_name>" << endl;
        return EXIT_FAILURE;
    }

    if (string(argv[1]) == "-dronetype")
    {
        dronetype = stoi(argv[2]);
        problem_name = argv[3];
    }
    else
    {
        cerr << "Usage: " << argv[0] << " -dronetype <1|2|3|4> <problem_file_name>" << endl;
        return EXIT_FAILURE;
    }
    try
    {
        TruckConfig truckConfig = TruckConfig::import_data();
        auto linearDroneConfigs = Linear_Drone_Config::import_data();

        if (linearDroneConfigs.find(dronetype) == linearDroneConfigs.end())
        {
            cerr << "Drone type " << dronetype << " not found in config." << endl;
            return EXIT_FAILURE;
        }
        Linear_Drone_Config chosenDrone = linearDroneConfigs[dronetype];
        Problem problem = readProblemData(problem_name);
        int K = problem.trucks_count;
        int D = problem.drones_count;
        int pop_size = 50;
        int generations = 200;
        double mutation_rate = 0.15;
        cout << "Running GA..." << endl;
        solveGA(problem, truckConfig, {chosenDrone}, K, D, pop_size, generations, mutation_rate);
        cout << "GA finished." << endl;

        if (global_result.best_time >= 1e9)
        {
            cout << "No solution found." << endl;
        }
        else
        {
            cout << "Minimum completion time: " << global_result.best_time << "\n";

            cout << "Truck routes:\n";
            for (size_t i = 0; i < global_result.best_solution[0].size(); i++)
            {
                cout << "Truck " << i + 1 << ": ";
                for (auto c : global_result.best_solution[0][i])
                    cout << c << " ";
                cout << "\n";
            }

            cout << "Drone routes:\n";
            for (size_t i = 0; i < global_result.best_solution[1].size(); i++)
            {
                cout << "Drone " << i + 1 << ": ";
                for (auto c : global_result.best_solution[1][i])
                    cout << c << " ";
                cout << "\n";
            }
        }
    }
    catch (const exception &e)
    {
        cerr << "Error: " << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
