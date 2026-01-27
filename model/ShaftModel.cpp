#include "../include/shaft_model.h"

ShaftModel::ShaftModel(double Ts_in) : Ts(Ts_in) {}
void ShaftModel::set_Ts(double Ts_in) { Ts = Ts_in; }

void ShaftModel::update_mechanics(double T_em, double T_turb, double T_pump) {
    const double T_fric_total = omega_mech * (b_motor + b_turb + b_pump);

    const double domega_mech_dt = (T_em + T_turb - T_pump - T_fric_total) / (J_motor + J_turb + J_pump); // 角加速度

    omega_mech += domega_mech_dt * Ts;  // 更新机械角速度
    theta_mech += omega_mech * Ts;      // 更新机械角位置
}
