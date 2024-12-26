#ifndef RESULT_H
#define RESULT_H

#include <vector>

struct GA_Result
{
    double best_time;
    // best_solution: [0]: trucks routes (vector<vector<int>>), [1]: drones routes (vector<vector<int>>)
    std::vector<std::vector<std::vector<int>>> best_solution;

    GA_Result() : best_time(1e9) {}
};

extern GA_Result global_result;

#endif
