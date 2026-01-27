#include "../include/pump_model.h" 
#include <cmath> 

PumpModel::PumpModel(double Ts_in) : Ts(Ts_in) {}
void PumpModel::set_Ts(double Ts_in) { Ts = Ts_in; }

void PumpModel::update_from_hydraulics(double omega_shaft) { 
    omega_p = omega_shaft;                      // 泵角速度（rad/s）
    const double omega_mag = std::max(std::abs(omega_p), omega_floor); // 防止除零
    H = rho * Q * omega_mag * omega_mag;        // 简化扬程模型（m）
    const double P = rho * Q * H * 9.8 / eta_p; // 功率估算（W）
    T_pump = P / omega_mag;                     // 负载扭矩（N·m）
} 
