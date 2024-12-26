#ifndef ENERGY_COMPUTATION_H
#define ENERGY_COMPUTATION_H

// Hàm tính năng lượng tiêu hao theo mô hình linear
// w: trọng lượng hàng mà drone mang (kg)
// beta: hệ số năng lượng tiêu hao phụ thuộc trọng lượng (W/kg)
// gamma: hệ số năng lượng tiêu hao cố định (W)
// cruiseAlt: độ cao bay hành trình (m)
// takeoffSpeed: vận tốc cất cánh (m/s)
// cruiseSpeed: vận tốc bay ngang (m/s)
// landingSpeed: vận tốc hạ cánh (m/s)
// dist: khoảng cách ngang giữa hai điểm (m)
// Hàm trả về năng lượng tiêu hao (Joule)
double computeLinearEnergy(double w, double beta, double gamma,
                           double cruiseAlt, double takeoffSpeed,
                           double cruiseSpeed, double landingSpeed,
                           double dist);

#endif // ENERGY_COMPUTATION_H
