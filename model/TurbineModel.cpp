#include "../include/turbine_model.h"
#include <cmath>
#include <algorithm>

TurbineModel::TurbineModel(double Ts_in) : Ts(Ts_in) {}
void TurbineModel::set_Ts(double Ts_in) { Ts = Ts_in; }

void TurbineModel::update_from_gas(double omega_shaft) {
    omega_t = omega_shaft; // 涡轮直接连接到轴系

    const double gamma_safe = std::max(gamma, 1.01);
    const double T_gas = std::max(T_in, 200.0);
    const double p_upstream = std::max(p_in, p_floor);
    const double p_back = std::max(p_out, p_floor);

    // 入口质量流：以 m_dot 为标称量，随供压与腔压差变化
    const double delta_p_in = std::max(p_upstream - p_plenum, 0.0);
    m_dot_in = C_in * m_dot * std::sqrt(delta_p_in / p_upstream);

    // 腔体 -> 背压：可压缩喷嘴流
    // 为防止零压或负压导致的数值问题，加入下限保护
    const double p_plenum_safe = std::max(p_plenum, p_floor);
    const double pressure_ratio = std::clamp(p_back / p_plenum_safe, 0.0, 1.0);
    const double critical_ratio = std::pow(2.0 / (gamma_safe + 1.0), gamma_safe / (gamma_safe - 1.0));

    // 喷嘴流量计算：考虑是否发生声速阻塞
    if (pressure_ratio <= critical_ratio) {
        const double choked_coeff = std::sqrt(gamma_safe / (gas_R * T_gas))
                                  * std::pow(2.0 / (gamma_safe + 1.0), (gamma_safe + 1.0) / (2.0 * (gamma_safe - 1.0)));
        m_dot_turb = Cd_nozzle * A_nozzle * p_plenum_safe * choked_coeff;
    } else {
        const double term = std::pow(pressure_ratio, 2.0 / gamma_safe)
                          - std::pow(pressure_ratio, (gamma_safe + 1.0) / gamma_safe);
        const double subsonic_coeff = std::sqrt(std::max(2.0 * gamma_safe / (gas_R * T_gas * (gamma_safe - 1.0)) * term, 0.0));
        m_dot_turb = Cd_nozzle * A_nozzle * p_plenum_safe * subsonic_coeff;
    }

    // 腔体压力动态：dp/dt = (R*T/V) * (m_in - m_out)
    const double V_safe = std::max(V_plenum, 1e-6);
    const double dp_dt = (gas_R * T_gas / V_safe) * (m_dot_in - m_dot_turb);
    p_plenum = std::max(p_floor, p_plenum + dp_dt * Ts);

    // 基于腔压到背压的膨胀做功
    const double p_turb_in = std::max(p_plenum, p_floor);
    if (p_turb_in <= p_back) {
        T_turb = 0.0;
        return;
    }

    const double cp = gamma_safe * gas_R / (gamma_safe - 1.0);
    const double T_out_ideal = T_gas * std::pow(p_back / p_turb_in, (gamma_safe - 1.0) / gamma_safe);
    const double delta_T_actual = std::max((T_gas - T_out_ideal) * eta_turb, 0.0);
    const double power_turbine = m_dot_turb * cp * delta_T_actual;

    // 涡轮轴扭矩
    T_turb = power_turbine / std::max(std::abs(omega_shaft), omega_floor);
}
