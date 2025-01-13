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

#include "../include/ReadData.h"
#include "../include/Result.h"

using namespace std;

// Biến toàn cục kết quả
extern GA_Result global_result;

// Biến atomic
static atomic<bool> interrupted(false);
static atomic<bool> population_failure(false);

// signal handler
void signal_handler_ga(int signum)
{
    if (signum == SIGINT)
    {
        interrupted.store(true);
    }
}

// Hàm định dạng thời gian dạng HH:MM:SS
string format_time(double seconds)
{
    int hrs = static_cast<int>(seconds) / 3600;
    int mins = (static_cast<int>(seconds) % 3600) / 60;
    int secs = static_cast<int>(seconds) % 60;
    ostringstream oss;
    oss << setw(2) << setfill('0') << hrs << ":"
        << setw(2) << setfill('0') << mins << ":"
        << setw(2) << setfill('0') << secs;
    return oss.str();
}

// Hàm in progress bar
void print_progress_bar(int current_generation, int total_generations,
                        chrono::steady_clock::time_point start_time)
{
    double progress = (double)current_generation / total_generations;
    if (progress > 1.0)
        progress = 1.0;

    auto current_time = chrono::steady_clock::now();
    chrono::duration<double> elapsed = current_time - start_time;
    double elapsed_seconds = elapsed.count();

    double estimated_total_seconds = (progress > 0.0) ? (elapsed_seconds / progress) : 0.0;
    double remaining_seconds = estimated_total_seconds - elapsed_seconds;

    string elapsed_str = format_time(elapsed_seconds);
    string remaining_str = format_time(max(0.0, remaining_seconds));

    int bar_width = 50;
    int pos = (int)(bar_width * progress);

    ostringstream oss;
    oss << "\r[";
    for (int i = 0; i < bar_width; ++i)
    {
        if (i < pos)
            oss << "#";
        else
            oss << "-";
    }
    oss << "] " << fixed << setprecision(2) << (progress * 100.0) << "% "
        << "Generations: " << current_generation << "/" << total_generations << " "
        << "Elapsed Time: " << elapsed_str << " "
        << "Remaining Time: " << remaining_str << flush;

    cout << oss.str();
}

// ---------------- Cấu trúc Individual ----------------
// Mỗi cá thể gồm 2 phần:
//  - truck_routes: vector<vector<int>> size = K
//  - drone_routes: vector<vector<int>> size = D
// Mỗi phần tử trong truck_routes/drone_routes là một route (các khách hàng)
struct Individual
{
    vector<vector<int>> truck_routes;
    vector<vector<int>> drone_routes;

    double fitness;
    double total_time;

    Individual() : fitness(0.0), total_time(numeric_limits<double>::max()) {}
};

// ---------------- Tính toán chi phí, thời gian, khả thi ----------------

// Hàm tính khoảng cách Manhattan cho truck
double truck_distance_manhattan(const Problem &problem, int from, int to)
{
    double dx = abs(problem.x[from - 1] - problem.x[to - 1]);
    double dy = abs(problem.y[from - 1] - problem.y[to - 1]);
    return dx + dy;
}

// Hàm tính khoảng cách Euclid cho drone
double drone_distance_euclid(const Problem &problem, int from, int to)
{
    double dx = problem.x[from - 1] - problem.x[to - 1];
    double dy = problem.y[from - 1] - problem.y[to - 1];
    return sqrt(dx * dx + dy * dy);
}

// Tính thời gian di chuyển truck
double compute_truck_time(const Problem &problem, const TruckConfig &truckConfig, const vector<int> &route)
{
    double total_time = 0.0;
    int current = 1; // depot
    for (int c : route)
    {
        double dist = truck_distance_manhattan(problem, current, c);
        double travel_time = dist / truckConfig.truck_velocity;
        double service_time = problem.serviceTimeByTruck[c - 1];
        total_time += travel_time + service_time;
        current = c;
    }
    // Quay lại depot
    double dist_back = truck_distance_manhattan(problem, current, 1);
    double travel_time_back = dist_back / truckConfig.truck_velocity;
    total_time += travel_time_back;
    return total_time;
}

// Tính năng lượng drone (linear model)
double compute_drone_energy(const Problem &problem, const Linear_Drone_Config &droneConf,
                            const vector<int> &route, const vector<double> &demands)
{
    double w_total = 0.0;
    for (auto c : route)
        w_total += demands[c - 1];

    double P = droneConf.beta * w_total + droneConf.gamma;

    auto segment_energy = [&](int from, int to)
    {
        double dist = drone_distance_euclid(problem, from, to);
        double time_takeoff = droneConf.cruiseAlt / droneConf.takeoffSpeed;
        double time_cruise = dist / droneConf.cruiseSpeed;
        double time_landing = droneConf.cruiseAlt / droneConf.landingSpeed;
        double E = P * (time_takeoff + time_cruise + time_landing);
        return E;
    };

    int current = 1;
    double total_energy = 0.0;
    for (int c : route)
    {
        total_energy += segment_energy(current, c);
        current = c;
    }
    // Về depot
    total_energy += segment_energy(current, 1);

    return total_energy;
}

// Tính thời gian drone
double compute_drone_time(const Problem &problem, const Linear_Drone_Config &droneConf, const vector<int> &route)
{
    auto segment_time = [&](int from, int to)
    {
        double dist = drone_distance_euclid(problem, from, to);
        double time_takeoff = droneConf.cruiseAlt / droneConf.takeoffSpeed;
        double time_cruise = dist / droneConf.cruiseSpeed;
        double time_landing = droneConf.cruiseAlt / droneConf.landingSpeed;
        return (time_takeoff + time_cruise + time_landing);
    };

    double total_time = 0.0;
    int current = 1;
    for (int c : route)
    {
        double service_time = problem.serviceTimeByDrone[c - 1];
        total_time += segment_time(current, c) + service_time;
        current = c;
    }
    total_time += segment_time(current, 1);

    return total_time;
}

// Kiểm tra tính khả thi drone route (dựa vào demand, battery,...)
bool check_drone_feasibility(const Problem &problem, const Linear_Drone_Config &droneConf, const vector<int> &route)
{
    double w_total = 0.0;
    for (auto c : route)
    {
        // Nếu điểm c không droneable => fail
        if (!problem.droneable[c - 1])
            return false;
        w_total += problem.demand[c - 1];
    }
    if (w_total > droneConf.capacity)
        return false;
    double E = compute_drone_energy(problem, droneConf, route, problem.demand);
    if (E > droneConf.batteryPower)
        return false;
    return true;
}

// Tính tổng thời gian hoàn thành lời giải
double compute_solution_time(const Problem &problem,
                             const TruckConfig &truckConfig,
                             const vector<Linear_Drone_Config> &droneConfigs,
                             const Individual &ind)
{
    double max_time = 0.0;

    // Tính thời gian cho từng truck
    for (auto &troute : ind.truck_routes)
    {
        double t = compute_truck_time(problem, truckConfig, troute);
        if (t > max_time)
            max_time = t;
    }

    // Tính thời gian cho từng drone
    // Ở đây tạm thời ta dùng 1 loại droneConfig => droneConfigs[0]
    // Nếu muốn mỗi drone có config riêng, có thể điều chỉnh logic tương tự
    const auto &dconf = droneConfigs[0];
    for (auto &droute : ind.drone_routes)
    {
        if (!droute.empty())
        {
            double t = compute_drone_time(problem, dconf, droute);
            if (t > max_time)
                max_time = t;
        }
    }

    return max_time;
}

// Kiểm tra tính khả thi chung của toàn cá thể
//  - Mỗi khách hàng được phục vụ đúng 1 lần
//  - Drone routes thoả mãn constraint
bool check_feasibility(const Problem &problem,
                       const TruckConfig &truckConfig,
                       const vector<Linear_Drone_Config> &droneConfigs,
                       const Individual &ind)
{
    int C = problem.customers_count;
    vector<int> count_served(C + 1, 0);

    // Truck routes
    for (auto &troute : ind.truck_routes)
    {
        for (auto c : troute)
        {
            if (c < 1 || c > C)
                return false;
            count_served[c]++;
        }
    }

    // Drone routes
    const auto &dconf = droneConfigs[0];
    for (auto &droute : ind.drone_routes)
    {
        for (auto c : droute)
        {
            if (c < 1 || c > C)
                return false;
            count_served[c]++;
        }
        if (!droute.empty())
        {
            if (!check_drone_feasibility(problem, dconf, droute))
                return false;
        }
    }

    // Mỗi khách hàng phải được phục vụ đúng 1 lần
    for (int c = 1; c <= C; c++)
    {
        if (count_served[c] != 1)
            return false;
    }

    return true;
}

// Tính fitness = 1 / total_time (time càng bé fitness càng lớn)
double compute_fitness(const Individual &ind)
{
    if (ind.total_time <= 0.0)
        return numeric_limits<double>::max();
    return 1.0 / ind.total_time;
}

// Hàm encode solution để kiểm tra uniqueness
// Chuỗi này dùng để phân biệt các cá thể trong quần thể
string encode_solution(const Individual &ind)
{
    ostringstream oss;
    oss << "T:";
    for (auto &r : ind.truck_routes)
    {
        oss << "[";
        for (auto c : r)
            oss << c << ",";
        oss << "]";
    }
    oss << "|D:";
    for (auto &r : ind.drone_routes)
    {
        oss << "[";
        for (auto c : r)
            oss << c << ",";
        oss << "]";
    }
    return oss.str();
}

// --------------------- Tạo cá thể ngẫu nhiên ---------------------
// Tạo 1 cá thể random, trong đó:
//   - Có K route truck (mỗi truck 1 route)
//   - Có D route drone (mỗi drone 1 route)
//   - Mỗi customer (1..C) sẽ được phân vào 1 route truck hoặc 1 route drone
//   - Nếu route drone không khả thi, ta bỏ và random lại
//   - Có giới hạn thời gian time_limit_seconds để tránh lặp vô hạn
Individual create_random_individual(const Problem &problem,
                                    int K,
                                    int D,
                                    const TruckConfig &truckConfig,
                                    const vector<Linear_Drone_Config> &droneConfigs,
                                    unordered_set<string> &population_set,
                                    double time_limit_seconds = 1.0)
{
    auto start = chrono::steady_clock::now();
    int C = problem.customers_count;
    vector<int> customers(C);
    iota(customers.begin(), customers.end(), 1);

    random_device rd;
    mt19937 gen(rd());

    while (true)
    {
        auto now = chrono::steady_clock::now();
        chrono::duration<double> elapsed = now - start;
        if (elapsed.count() > time_limit_seconds)
        {
            // Quá thời gian => trả về cá thể không khả thi
            Individual ind;
            ind.total_time = numeric_limits<double>::max();
            return ind;
        }

        shuffle(customers.begin(), customers.end(), gen);

        Individual candidate;
        candidate.truck_routes.resize(K);
        candidate.drone_routes.resize(D);

        // Gán mỗi customer cho 1 truck hoặc 1 drone ngẫu nhiên
        for (auto c : customers)
        {
            uniform_real_distribution<> dis(0.0, 1.0);
            double r = dis(gen);

            // Xác suất 50% drone, 50% truck (có thể tuỳ chỉnh)
            if (r < 0.5 && D > 0 && problem.droneable[c - 1])
            {
                // gán cho 1 drone ngẫu nhiên
                int d_id = uniform_int_distribution<>(0, D - 1)(gen);
                candidate.drone_routes[d_id].push_back(c);
            }
            else
            {
                // gán cho 1 truck ngẫu nhiên
                int t_id = uniform_int_distribution<>(0, K - 1)(gen);
                candidate.truck_routes[t_id].push_back(c);
            }
        }

        // Tính time, fitness
        candidate.total_time = compute_solution_time(problem, truckConfig, droneConfigs, candidate);
        candidate.fitness = compute_fitness(candidate);

        // Kiểm tra khả thi
        if (check_feasibility(problem, truckConfig, droneConfigs, candidate))
        {
            // Kiểm tra uniqueness
            string key = encode_solution(candidate);
            if (population_set.find(key) == population_set.end())
            {
                population_set.insert(key);
                return candidate;
            }
        }
        // nếu không khả thi thì lặp lại
    }
}

// --------------------- Tournament selection ---------------------
const Individual &selection_tournament(const vector<Individual> &population, int tsize = 5)
{
    static thread_local mt19937 gen(random_device{}());
    uniform_int_distribution<> dis(0, (int)population.size() - 1);
    double best_f = -1.0;
    int best_idx = -1;
    for (int i = 0; i < tsize; i++)
    {
        int idx = dis(gen);
        if (population[idx].fitness > best_f)
        {
            best_f = population[idx].fitness;
            best_idx = idx;
        }
    }
    return population[best_idx];
}

// --------------------- Crossover (lai ghép) ---------------------
// Thay vì cắt 1 truck-route + 1 drone-route như cũ, làm route-based:
//   - For each truck route i, lấy route i từ p1 hay p2 với tỉ lệ 50/50
//   - For each drone route i, lấy route i từ p1 hay p2 với tỉ lệ 50/50
// Sau đó xử lý trùng / thiếu khách hàng
Individual crossover(const Individual &p1,
                     const Individual &p2,
                     const Problem &problem,
                     int K,
                     int D,
                     const TruckConfig &truckConfig,
                     const vector<Linear_Drone_Config> &droneConfigs)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    Individual child;
    child.truck_routes.resize(K);
    child.drone_routes.resize(D);

    // Lấy route truck
    for (int i = 0; i < K; i++)
    {
        double r = dis(gen);
        if (r < 0.5)
            child.truck_routes[i] = p1.truck_routes[i];
        else
            child.truck_routes[i] = p2.truck_routes[i];
    }
    // Lấy route drone
    for (int i = 0; i < D; i++)
    {
        double r = dis(gen);
        if (r < 0.5)
            child.drone_routes[i] = p1.drone_routes[i];
        else
            child.drone_routes[i] = p2.drone_routes[i];
    }

    // ----- Xử lý trùng và thiếu khách hàng -----
    int C = problem.customers_count;
    vector<int> count(C + 1, 0);

    // Đếm xem những customer nào xuất hiện bao nhiêu lần
    for (auto &r : child.truck_routes)
        for (auto c : r)
            count[c]++;
    for (auto &r : child.drone_routes)
        for (auto c : r)
            count[c]++;

    // Nếu trùng (count[c] > 1) => xoá bớt
    // Ta duyệt drone_routes trước rồi truck_routes (hoặc ngược lại), tuỳ ý
    // Miễn là loại bớt cho count[c] = 1
    for (int c = 1; c <= C; c++)
    {
        while (count[c] > 1)
        {
            bool removed = false;
            // Thử xoá ở drone
            for (auto &dr : child.drone_routes)
            {
                auto it = find(dr.begin(), dr.end(), c);
                if (it != dr.end())
                {
                    dr.erase(it);
                    count[c]--;
                    removed = true;
                    break;
                }
            }
            if (!removed)
            {
                // Xoá ở truck
                for (auto &tr : child.truck_routes)
                {
                    auto it = find(tr.begin(), tr.end(), c);
                    if (it != tr.end())
                    {
                        tr.erase(it);
                        count[c]--;
                        break;
                    }
                }
            }
        }
    }

    // Nếu thiếu (count[c] == 0) => chèn vào 1 route
    // ưu tiên drone nếu droneable, else truck
    for (int c = 1; c <= C; c++)
    {
        if (count[c] == 0)
        {
            if (problem.droneable[c - 1] && D > 0)
            {
                // tạm chèn vào drone 0
                child.drone_routes[0].push_back(c);
                count[c]++;
            }
            else
            {
                // chèn vào truck 0
                child.truck_routes[0].push_back(c);
                count[c]++;
            }
        }
    }

    // Tính time, fitness
    child.total_time = compute_solution_time(problem, truckConfig, droneConfigs, child);
    child.fitness = compute_fitness(child);

    // Kiểm tra khả thi, nếu fail => đánh dấu total_time = max
    if (!check_feasibility(problem, truckConfig, droneConfigs, child))
    {
        child.total_time = numeric_limits<double>::max();
        child.fitness = 0.0;
    }

    return child;
}

// --------------------- Mutation (đột biến) ---------------------
// Chọn ngẫu nhiên 1 route (truck hoặc drone), hoán đổi 2 khách hàng
void mutate(Individual &ind,
            const Problem &problem,
            const TruckConfig &truckConfig,
            const vector<Linear_Drone_Config> &droneConfigs,
            double mutation_rate)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    double r = dis(gen);
    if (r > mutation_rate)
        return;

    // Chọn ngẫu nhiên truck-route hay drone-route
    bool choose_drone = (!ind.drone_routes.empty()) && (dis(gen) < 0.5);
    if (choose_drone)
    {
        int D = (int)ind.drone_routes.size();
        int d_id = uniform_int_distribution<>(0, D - 1)(gen);
        auto &route = ind.drone_routes[d_id];
        if (route.size() > 1)
        {
            int start = uniform_int_distribution<>(0, (int)route.size() - 1)(gen);
            int end = uniform_int_distribution<>(0, (int)route.size() - 1)(gen);
            if (start > end)
                std::swap(start, end);
            // Hoán đổi 2 phần tử
            std::swap(route[start], route[end]);
        }
    }
    else
    {
        int K = (int)ind.truck_routes.size();
        int t_id = uniform_int_distribution<>(0, K - 1)(gen);
        auto &route = ind.truck_routes[t_id];
        if (route.size() > 1)
        {
            int start = uniform_int_distribution<>(0, (int)route.size() - 1)(gen);
            int end = uniform_int_distribution<>(0, (int)route.size() - 1)(gen);
            if (start > end)
                std::swap(start, end);
            // Hoán đổi 2 phần tử
            std::swap(route[start], route[end]);
        }
    }

    // Tính lại fitness
    ind.total_time = compute_solution_time(problem, truckConfig, droneConfigs, ind);
    ind.fitness = compute_fitness(ind);

    // Nếu không khả thi => set về max
    if (!check_feasibility(problem, truckConfig, droneConfigs, ind))
    {
        ind.total_time = numeric_limits<double>::max();
        ind.fitness = 0.0;
    }
}

void solveGA(const Problem &problem,
             const TruckConfig &truckConfig,
             const vector<Linear_Drone_Config> &droneConfigs,
             int K,
             int D,
             int pop_size,
             int generations,
             double mutation_rate)
{
    signal(SIGINT, signal_handler_ga);
    unordered_set<string> population_set;
    vector<Individual> population;

    // Tạo quần thể ban đầu
    auto start_pop = chrono::steady_clock::now();
    while ((int)population.size() < pop_size)
    {
        auto now = chrono::steady_clock::now();
        chrono::duration<double> elapsed = now - start_pop;

        // Nếu quá 1 giây mà vẫn chưa tạo nổi 1 cá thể => báo lỗi
        if (elapsed.count() > 1.0 && population.empty())
        {
            population_failure.store(true);
            cerr << "Cannot create initial population\n";
            break;
        }

        Individual ind = create_random_individual(problem, K, D, truckConfig, droneConfigs, population_set, 0.5);
        if (ind.total_time < numeric_limits<double>::max())
        {
            population.push_back(ind);
        }
        else
        {
            // Chỉ break nếu đã có ít nhất 1 cá thể
            if (!population.empty())
                break;
        }
    }

    if (population.empty())
    {
        cerr << "Population is empty. GA cannot run.\n";
        return;
    }

    double best_time = numeric_limits<double>::max();
    vector<vector<vector<int>>> best_solution; // [0]->truck_routes, [1]->drone_routes

    int gen_no_improve = 0;
    int elite_count = max(1, pop_size / 20);

    // Lấy thời gian bắt đầu để in progress bar
    auto start_time = chrono::steady_clock::now();

    for (int g = 0; g < generations; g++)
    {
        if (interrupted.load() || population_failure.load())
        {
            cout << "\nTerminated early.\n";
            break;
        }

        // Cập nhật fitness
        for (auto &ind : population)
        {
            ind.fitness = compute_fitness(ind);
        }

        // Cập nhật best solution
        for (auto &ind : population)
        {
            if (ind.total_time < best_time)
            {
                best_time = ind.total_time;
                best_solution.clear();
                best_solution.resize(2);
                best_solution[0] = ind.truck_routes;
                best_solution[1] = ind.drone_routes;
                gen_no_improve = 0;
            }
        }
        gen_no_improve++;

        // Sắp xếp quần thể theo fitness
        sort(population.begin(), population.end(),
             [](const Individual &a, const Individual &b)
             {
                 return a.fitness > b.fitness;
             });

        // Tạo quần thể mới
        vector<Individual> new_pop;
        new_pop.reserve(pop_size);

        // Lấy elitism
        for (int i = 0; i < elite_count && i < (int)population.size(); i++)
        {
            new_pop.push_back(population[i]);
        }

        // Lai ghép
        auto start_gen = chrono::steady_clock::now();
        while ((int)new_pop.size() < pop_size)
        {
            auto now = chrono::steady_clock::now();
            chrono::duration<double> el = now - start_gen;
            if (el.count() > 1.0)
            {
                // Tránh tốn quá nhiều thời gian cho 1 thế hệ
                break;
            }

            const Individual &p1 = selection_tournament(population);
            const Individual &p2 = selection_tournament(population);

            Individual child = crossover(p1, p2, problem, K, D, truckConfig, droneConfigs);
            if (child.total_time < numeric_limits<double>::max())
            {
                // Đột biến
                mutate(child, problem, truckConfig, droneConfigs, mutation_rate);

                // Kiểm tra lần nữa
                if (child.total_time < numeric_limits<double>::max())
                {
                    // Check uniqueness
                    string key = encode_solution(child);
                    if (population_set.find(key) == population_set.end())
                    {
                        population_set.insert(key);
                        new_pop.push_back(child);
                    }
                }
            }
        }

        // Bù cá thể random nếu thiếu
        while ((int)new_pop.size() < pop_size)
        {
            Individual ind = create_random_individual(problem, K, D, truckConfig, droneConfigs, population_set, 0.5);
            if (ind.total_time < numeric_limits<double>::max())
            {
                new_pop.push_back(ind);
            }
            else
            {
                break;
            }
        }

        // Quần thể thế hệ sau
        population = move(new_pop);

        // In progress bar
        print_progress_bar(g + 1, generations, start_time);
    }

    cout << endl; // xuống dòng sau progress bar

    // Lưu kết quả tốt nhất vào global_result
    global_result.best_time = best_time;
    global_result.best_solution = best_solution;
}
