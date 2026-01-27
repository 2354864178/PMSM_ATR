#include "../include/motor_model.h"
#include <cmath>

MotorModel::MotorModel(double Ts_in) : Ts(Ts_in) {}
void MotorModel::set_Ts(double Ts_in) { Ts = Ts_in; }

void MotorModel::update_electrical(double omega_mech) {
    omega_e = p * omega_mech;  // 电角速度 rad/s
    theta_e += omega_e * Ts;   // 电角度 rad
    
    // Clarke 变换
    u_alpha = (2.0 / 3.0) * (ua - 0.5 * ub - 0.5 * uc); 
    u_beta  = (2.0 / 3.0) * ((std::sqrt(3) / 2.0) * ub - (std::sqrt(3) / 2.0) * uc);

    // Park 变换
    const double cos_theta = std::cos(theta_e);
    const double sin_theta = std::sin(theta_e); 
    ud = u_alpha * cos_theta + u_beta * sin_theta;      
    uq = -u_alpha * sin_theta + u_beta * cos_theta;

    // 电流微分方程与积分
    const double did_dt = (ud - Rs * id + omega_e * Lq * iq) / Ld;
    const double diq_dt = (uq - Rs * iq - omega_e * Ld * id - omega_e * psi_f) / Lq;
    id += did_dt * Ts;
    iq += diq_dt * Ts;
    
    // 逆 Park 变换
    i_alpha = id * cos_theta - iq * sin_theta;
    i_beta  = id * sin_theta + iq * cos_theta;

    // 逆 Clarke 变换
    ia = i_alpha;
    ib = -0.5 * i_alpha + (std::sqrt(3) / 2.0) * i_beta;
    ic = -0.5 * i_alpha - (std::sqrt(3) / 2.0) * i_beta;
    
    // 电磁转矩计算
    T_em = 1.5 * (p / 2.0) * (psi_f * iq + (Ld - Lq) * id * iq);
}
