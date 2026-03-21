#include "components/turbine_model.h"
#include <cmath>
#include <algorithm>

// 构造函数：记录步长（便于后续拓展显式更新接口）。
TurbineModel::TurbineModel(double Ts_in) : Ts(Ts_in) {}
void TurbineModel::set_Ts(double Ts_in) { Ts = Ts_in; }

// 简化涡轮气动模型：基于入口压力、腔体压力和背压的质量流量计算，以及等熵效率的轴扭矩估计。
TurbineModel::GasEval TurbineModel::evaluate_gas(double p_plenum_in, double omega_shaft) const {
    GasEval out;    // 输出结构体，包含腔体压力变化、进腔质量流量、通过涡轮的质量流量和轴扭矩
    const double gamma_safe = std::max(gamma, 1.01);    // 确保比热比合理，避免数值问题
    const double T_gas = std::max(T_in, 200.0);         // 确保气体温度合理，避免数值问题
    const double p_upstream = std::max(p_in, p_floor);  // 确保供气压力合理，避免数值问题
    const double p_back = std::max(p_out, p_floor);     // 确保背压合理，避免数值问题
    const double p_plenum_safe = std::max(p_plenum_in, p_floor);    // 确保腔体压力合理，避免数值问题

    // 入口流量：用入口压差近似驱动
    const double delta_p_in = std::max(p_upstream - p_plenum_safe, 0.0);    // 入口压力与腔体压力的差值，确保非负
    out.m_dot_in = C_in * m_dot * std::sqrt(delta_p_in / p_upstream);       // 进腔质量流量，基于入口压力差和流量系数计算

    const double pressure_ratio = std::clamp(p_back / p_plenum_safe, 0.0, 1.0); // 背压与腔体压力的比值，限制在0到1之间
    const double critical_ratio = std::pow(2.0 / (gamma_safe + 1.0), gamma_safe / (gamma_safe - 1.0));  // 临界压力比，基于比热比计算

    // 喷嘴流量：按临界压比区分壅塞/非壅塞
    if (pressure_ratio <= critical_ratio) {
        const double choked_coeff = std::sqrt(gamma_safe / (gas_R * T_gas)) // 声速流动，使用声速流动公式计算质量流量
                                  * std::pow(2.0 / (gamma_safe + 1.0), (gamma_safe + 1.0) / (2.0 * (gamma_safe - 1.0)));
        out.m_dot_turb = Cd_nozzle * A_nozzle * p_plenum_safe * choked_coeff;
    } else {
        // 亚声速流动，使用亚声速流动公式计算质量流量
        const double term = std::pow(pressure_ratio, 2.0 / gamma_safe)
                          - std::pow(pressure_ratio, (gamma_safe + 1.0) / gamma_safe);
        const double subsonic_coeff = std::sqrt(std::max(2.0 * gamma_safe / (gas_R * T_gas * (gamma_safe - 1.0)) * term, 0.0));
        out.m_dot_turb = Cd_nozzle * A_nozzle * p_plenum_safe * subsonic_coeff;
    }

    // 前腔压力动态：dp/dt = (R*T/V)*(m_in - m_turb)
    const double V_safe = std::max(V_plenum, 1e-6);
    out.dp_plenum = (gas_R * T_gas / V_safe) * (out.m_dot_in - out.m_dot_turb);

    // 若腔压不高于背压，视作无有效做功。
    if (p_plenum_safe <= p_back) {
        out.T_turb = 0.0;
        return out;
    }

    // 由等熵膨胀功估算轴功率，再换算扭矩。
    const double cp = gamma_safe * gas_R / (gamma_safe - 1.0);
    const double T_out_ideal = T_gas * std::pow(p_back / p_plenum_safe, (gamma_safe - 1.0) / gamma_safe);
    const double delta_T_actual = std::max((T_gas - T_out_ideal) * eta_turb, 0.0);
    const double power_turbine = out.m_dot_turb * cp * delta_T_actual;
    out.T_turb = power_turbine / std::max(std::abs(omega_shaft), omega_floor);
    return out;
}

// 将评估的气动状态应用到对象中，供其他模块读取。
void TurbineModel::apply_gas_state(double omega_shaft, double p_plenum_in, const GasEval& eval) {
    omega_t = omega_shaft;
    p_plenum = std::max(p_plenum_in, p_floor);
    m_dot_in = eval.m_dot_in;
    m_dot_turb = eval.m_dot_turb;
    T_turb = eval.T_turb;
}

// 从配置结构体中应用涡轮参数。
void TurbineModel::apply_config(const SimulationConfig::Turbine& cfg) {
    gamma = cfg.gamma;
    gas_R = cfg.gas_R;
    eta_turb = cfg.eta_turb;
    V_plenum = cfg.V_plenum;
    A_nozzle = cfg.A_nozzle;
    Cd_nozzle = cfg.Cd_nozzle;
    C_in = cfg.C_in;
}
