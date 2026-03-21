#include "../include/pump_model.h" 
#include <cmath> 

PumpModel::PumpModel(double Ts_in) : Ts(Ts_in) {}
void PumpModel::set_Ts(double Ts_in) { Ts = Ts_in; }

void PumpModel::update_from_hydraulics(double omega_shaft) { 
    omega_p = omega_shaft;                      // 泵角速度（rad/s）
    const double omega_mag = std::max(std::abs(omega_p), omega_floor); // 防止除零

    const double p_suc = std::max(p_suction, p_floor);
    const double p_dn = std::max(p_downstream, p_floor);
    p_discharge = std::max(p_discharge, p_floor);

    // 泵本体产生流量（转速项 - 泄漏项）
    Q_pump = std::max(K_q * omega_mag - K_slip * std::max(p_discharge - p_suc, 0.0), 0.0);

    // 向下游流量（流阻模型）
    Q_out = std::max((p_discharge - p_dn) / std::max(R_out, 1.0), 0.0);

    // 容积动态：dp/dt = (beta/V) * (Q_pump - Q_out)
    const double V_safe = std::max(V_out, 1e-6);
    const double dp_dt = (beta_eff / V_safe) * (Q_pump - Q_out);
    p_discharge = std::max(p_floor, p_discharge + dp_dt * Ts);

    // 输出量
    Q = Q_out;
    dp_pump = std::max(p_discharge - p_suc, 0.0);
    H = dp_pump / std::max(rho * 9.8, 1.0);

    // 轴功率与负载扭矩（采用泵产生流量计算水力功）
    const double eta_safe = std::clamp(eta_p, 0.05, 1.0);
    const double P = dp_pump * Q_pump / eta_safe;
    T_pump = P / omega_mag;                     // 负载扭矩（N·m）
} 
