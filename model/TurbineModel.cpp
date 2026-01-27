#include "../include/turbine_model.h"
#include <cmath>
#include <algorithm>

TurbineModel::TurbineModel(double Ts_in) : Ts(Ts_in) {}
void TurbineModel::set_Ts(double Ts_in) { Ts = Ts_in; }

void TurbineModel::update_from_gas(double omega_shaft) {
    omega_t = omega_shaft; // 涡轮直接连接到轴系
    const double T_out_ideal = T_in * std::pow(p_out / p_in, (gamma - 1) / gamma);
    const double delta_T_ideal = T_in - T_out_ideal;
    const double delta_T_actual = delta_T_ideal * eta_turb;
    const double power_turbine = m_dot * gas_R / (gamma - 1) * delta_T_actual;

    // 涡轮轴扭矩
    T_turb = power_turbine / std::max(omega_shaft, omega_floor);
}
