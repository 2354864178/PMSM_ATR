#include "components/pump_model.h" 
#include "utils/numeric_utils.h"
#include <cmath> 

// 构造函数：记录仿真步长（当前主要用于保持接口一致性）。
PumpModel::PumpModel(double Ts_in) : Ts(Ts_in) {}
void PumpModel::set_Ts(double Ts_in) { Ts = Ts_in; }

// 泵液动评估（纯计算，不修改对象状态）：
PumpModel::HydraulicEval PumpModel::evaluate_hydraulics(double p_discharge_in,
                                                        double omega_shaft,
                                                        const Command& cmd) const {
    HydraulicEval out;
    const double p_suc = numeric_utils::clamp_floor(cmd.p_suction, p_floor);
    const double p_dn = numeric_utils::clamp_floor(cmd.p_downstream, p_floor);
    const double p_discharge_safe = numeric_utils::clamp_floor(p_discharge_in, p_floor);
    const double omega_abs = std::abs(omega_shaft);

    // 流量模型
    out.Q_pump = std::max(K_q * omega_abs - K_slip * std::max(p_discharge_safe - p_suc, 0.0), 0.0);
    out.Q_out = std::max((p_discharge_safe - p_dn) / numeric_utils::clamp_floor(R_out, 1.0), 0.0);

    // 压力动态
    const double V_safe = numeric_utils::clamp_floor(V_out, 1e-6);
    out.dp_discharge = (beta_eff / V_safe) * (out.Q_pump - out.Q_out);

    // 压升与扬程
    out.dp_pump = std::max(p_discharge_safe - p_suc, 0.0);
    out.H = out.dp_pump / numeric_utils::clamp_floor(rho * 9.8, 1.0);

    // 负载扭矩
    const double eta_safe = std::clamp(eta_p, 0.05, 1.0);
    const double P = out.dp_pump * out.Q_pump / eta_safe;
    const double omega_denom = numeric_utils::clamp_floor(omega_abs, 1e-6);
    const double omega_knee = std::max(omega_floor, 1.0);
    const double low_speed_scale = omega_abs / (omega_abs + omega_knee);
    out.T_pump = (P / omega_denom) * low_speed_scale;
    return out;
}

// 将评估结果回写到对象成员，供日志与外部模块读取。
void PumpModel::apply_hydraulic_state(double omega_shaft, double p_discharge_in, const HydraulicEval& eval) {
    omega_p = omega_shaft;
    p_discharge = numeric_utils::clamp_floor(p_discharge_in, p_floor);
    Q_pump = eval.Q_pump;
    Q_out = eval.Q_out;
    Q = eval.Q_out;
    dp_pump = eval.dp_pump;
    H = eval.H;
    T_pump = eval.T_pump;
}

// 从统一配置结构体加载泵参数。
void PumpModel::apply_config(const SimulationConfig::Pump& cfg) {
    rho = cfg.rho;
    eta_p = cfg.eta_p;
    p_suction = cfg.p_suction;
    p_downstream = cfg.p_downstream;
    p_discharge = cfg.p_discharge;
    V_out = cfg.V_out;
    beta_eff = cfg.beta_eff;
    R_out = cfg.R_out;
    K_q = cfg.K_q;
    K_slip = cfg.K_slip;
}
