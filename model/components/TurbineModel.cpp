#include "components/turbine_model.h"
#include "utils/numeric_utils.h"
#include <cmath>
#include <algorithm>

// 构造函数：记录仿真步长（当前主要用于保持接口一致性）。
TurbineModel::TurbineModel(double Ts_in) : Ts(Ts_in) {}
void TurbineModel::set_Ts(double Ts_in) { Ts = Ts_in; }

// 双腔涡轮气动评估（纯计算，不修改对象状态）：
TurbineModel::GasEval TurbineModel::evaluate_gas(double p_plenum_in,
                                                 double p_downstream_in,
                                                 double omega_shaft,
                                                 const Command& cmd) const {
    GasEval out;
    const double gamma_safe = numeric_utils::clamp_floor(gamma, 1.01);
    const double T_gas = numeric_utils::clamp_floor(cmd.T_in, 200.0);
    const double p_upstream = numeric_utils::clamp_floor(cmd.p_in, p_floor);
    const double p_back = numeric_utils::clamp_floor(cmd.p_out, p_floor);
    const double p_plenum_safe = numeric_utils::clamp_floor(p_plenum_in, p_floor);
    const double p_downstream_safe = numeric_utils::clamp_floor(p_downstream_in, p_floor);

    // 入口流量
    const double delta_p_in = std::max(p_upstream - p_plenum_safe, 0.0);
    out.m_dot_in = C_in * cmd.m_dot * std::sqrt(delta_p_in / p_upstream);

    const double pressure_ratio = std::clamp(p_downstream_safe / p_plenum_safe, 0.0, 1.0);
    const double critical_ratio = std::pow(2.0 / (gamma_safe + 1.0), gamma_safe / (gamma_safe - 1.0));

    // 喷嘴流量（壅塞/非壅塞）
    if (pressure_ratio <= critical_ratio) {
        const double choked_coeff = std::sqrt(gamma_safe / (gas_R * T_gas))
                                  * std::pow(2.0 / (gamma_safe + 1.0), (gamma_safe + 1.0) / (2.0 * (gamma_safe - 1.0)));
        out.m_dot_turb = Cd_nozzle * A_nozzle * p_plenum_safe * choked_coeff;
    } else {
        const double term = std::pow(pressure_ratio, 2.0 / gamma_safe)
                          - std::pow(pressure_ratio, (gamma_safe + 1.0) / gamma_safe);
        const double subsonic_coeff = std::sqrt(std::max(2.0 * gamma_safe / (gas_R * T_gas * (gamma_safe - 1.0)) * term, 0.0));
        out.m_dot_turb = Cd_nozzle * A_nozzle * p_plenum_safe * subsonic_coeff;
    }

    // 后腔出流（壅塞/非壅塞）
    if (p_downstream_safe <= p_back) {
        out.m_dot_out = 0.0;
    } else {
        const double outlet_ratio = std::clamp(p_back / p_downstream_safe, 0.0, 1.0);
        if (outlet_ratio <= critical_ratio) {
            const double choked_coeff = std::sqrt(gamma_safe / (gas_R * T_gas))
                                      * std::pow(2.0 / (gamma_safe + 1.0), (gamma_safe + 1.0) / (2.0 * (gamma_safe - 1.0)));
            out.m_dot_out = Cd_outlet * A_outlet * p_downstream_safe * choked_coeff;
        } else {
            const double term = std::pow(outlet_ratio, 2.0 / gamma_safe)
                              - std::pow(outlet_ratio, (gamma_safe + 1.0) / gamma_safe);
            const double subsonic_coeff = std::sqrt(std::max(2.0 * gamma_safe / (gas_R * T_gas * (gamma_safe - 1.0)) * term, 0.0));
            out.m_dot_out = Cd_outlet * A_outlet * p_downstream_safe * subsonic_coeff;
        }
    }

    // 双腔压力动态
    const double V_plenum_safe = numeric_utils::clamp_floor(V_plenum, 1e-6);
    const double V_downstream_safe = numeric_utils::clamp_floor(V_downstream, 1e-6);
    out.dp_plenum = (gas_R * T_gas / V_plenum_safe) * (out.m_dot_in - out.m_dot_turb);
    out.dp_downstream = (gas_R * T_gas / V_downstream_safe) * (out.m_dot_turb - out.m_dot_out);

    // 无压降时不做功
    if (p_plenum_safe <= p_downstream_safe) {
        out.T_turb = 0.0;
        return out;
    }

    // 轴功率与扭矩
    const double cp = gamma_safe * gas_R / (gamma_safe - 1.0);
    const double T_out_ideal = T_gas * std::pow(p_downstream_safe / p_plenum_safe, (gamma_safe - 1.0) / gamma_safe);
    const double delta_T_actual = std::max((T_gas - T_out_ideal) * eta_turb, 0.0);
    const double power_turbine = out.m_dot_turb * cp * delta_T_actual;
    out.T_turb = power_turbine / numeric_utils::abs_floor(omega_shaft, omega_floor);
    return out;
}

// 将评估结果回写到对象成员，供日志与外部模块读取。
void TurbineModel::apply_gas_state(double omega_shaft, double p_plenum_in, double p_downstream_in, const GasEval& eval) {
    omega_t = omega_shaft;
    p_plenum = numeric_utils::clamp_floor(p_plenum_in, p_floor);
    p_downstream = numeric_utils::clamp_floor(p_downstream_in, p_floor);
    m_dot_in = eval.m_dot_in;
    m_dot_turb = eval.m_dot_turb;
    m_dot_out = eval.m_dot_out;
    T_turb = eval.T_turb;
}

// 从统一配置结构体加载涡轮参数。
void TurbineModel::apply_config(const SimulationConfig::Turbine& cfg) {
    gamma = cfg.gamma;
    gas_R = cfg.gas_R;
    eta_turb = cfg.eta_turb;
    V_plenum = cfg.V_plenum;
    V_downstream = cfg.V_downstream;
    A_nozzle = cfg.A_nozzle;
    Cd_nozzle = cfg.Cd_nozzle;
    A_outlet = cfg.A_outlet;
    Cd_outlet = cfg.Cd_outlet;
    C_in = cfg.C_in;
}
