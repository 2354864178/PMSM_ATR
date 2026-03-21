#include "components/shaft_model.h"
#include <algorithm>

// 构造函数：记录步长（当前由系统求解器统一推进）。
ShaftModel::ShaftModel(double Ts_in) : Ts(Ts_in) {}
void ShaftModel::set_Ts(double Ts_in) { Ts = Ts_in; }

// 评估机械状态导数：
// domega = (T_em + T_turb - T_pump - T_fric) / J_total
// dtheta = omega
ShaftModel::MechanicalEval ShaftModel::evaluate_mechanics(double omega_mech_in,
                                                          double T_em,
                                                          double T_turb,
                                                          double T_pump) const {
    MechanicalEval out;
    const double T_fric_total = omega_mech_in * (b_motor + b_turb + b_pump);
    const double J_total = std::max(J_motor + J_turb + J_pump, 1e-9);
    out.domega_mech = (T_em + T_turb - T_pump - T_fric_total) / J_total;    // 计算机械角加速度
    out.dtheta_mech = omega_mech_in;    // 机械角位置的时间导数即为机械角速度
    return out;
}

// 回写机械状态。
void ShaftModel::apply_mechanical_state(double theta_mech_in, double omega_mech_in) {
    theta_mech = theta_mech_in; // 更新机械角位置
    omega_mech = omega_mech_in; // 更新机械角速度
}

// 从配置结构体中加载轴系参数。
void ShaftModel::apply_config(const SimulationConfig::Shaft& cfg) {
    b_motor = cfg.b_motor; 
    b_turb = cfg.b_turb;
    b_pump = cfg.b_pump;
    J_motor = cfg.J_motor;
    J_turb = cfg.J_turb;
    J_pump = cfg.J_pump;
}
