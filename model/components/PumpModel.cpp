#include "components/pump_model.h" 
#include <cmath> 

// 构造函数：记录步长（当前主要用于接口一致性）。
PumpModel::PumpModel(double Ts_in) : Ts(Ts_in) {}
void PumpModel::set_Ts(double Ts_in) { Ts = Ts_in; }

// 泵液动评估（纯计算）：
// 1) 计算泵输出流量与下游流量
// 2) 计算出口压力导数
// 3) 计算压升、扬程与负载扭矩
PumpModel::HydraulicEval PumpModel::evaluate_hydraulics(double p_discharge_in, double omega_shaft) const {
    HydraulicEval out;
    const double p_suc = std::max(p_suction, p_floor);
    const double p_dn = std::max(p_downstream, p_floor);
    const double p_discharge_safe = std::max(p_discharge_in, p_floor);
    const double omega_mag = std::max(std::abs(omega_shaft), omega_floor);

    // 泵产生流量（速度项 - 泄漏项）
    out.Q_pump = std::max(K_q * omega_mag - K_slip * std::max(p_discharge_safe - p_suc, 0.0), 0.0);
    // 下游阻力流量
    out.Q_out = std::max((p_discharge_safe - p_dn) / std::max(R_out, 1.0), 0.0);

    // 出口容积压力动态
    const double V_safe = std::max(V_out, 1e-6);
    out.dp_discharge = (beta_eff / V_safe) * (out.Q_pump - out.Q_out);

    // 压升与扬程
    out.dp_pump = std::max(p_discharge_safe - p_suc, 0.0);
    out.H = out.dp_pump / std::max(rho * 9.8, 1.0);

    // 轴功率与负载扭矩
    const double eta_safe = std::clamp(eta_p, 0.05, 1.0);
    const double P = out.dp_pump * out.Q_pump / eta_safe;
    out.T_pump = P / omega_mag;
    return out;
}

// 将液动评估结果回写到对象中。
void PumpModel::apply_hydraulic_state(double omega_shaft, double p_discharge_in, const HydraulicEval& eval) {
    omega_p = omega_shaft;
    p_discharge = std::max(p_discharge_in, p_floor);
    Q_pump = eval.Q_pump;
    Q_out = eval.Q_out;
    Q = eval.Q_out;
    dp_pump = eval.dp_pump;
    H = eval.H;
    T_pump = eval.T_pump;
}

// 从配置结构体中加载泵参数。
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
