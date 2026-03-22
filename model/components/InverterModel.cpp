#include "components/inverter_model.h"

#include <algorithm>
#include <cmath>

InverterModel::InverterModel(double Ts_in) : Ts(Ts_in) {}

void InverterModel::set_Ts(double Ts_in) { Ts = Ts_in; }

InverterModel::AcEval InverterModel::evaluate_ac_side(double v_dc_in,
                                                       const SVPWMController::Output& svpwm) const {
    AcEval out;
    const double v_dc_safe = std::max(std::abs(v_dc_in), 1.0);
    out.duty_a = std::clamp(svpwm.gate_u_a, 0.0, 1.0);
    out.duty_b = std::clamp(svpwm.gate_u_b, 0.0, 1.0);
    out.duty_c = std::clamp(svpwm.gate_u_c, 0.0, 1.0);

    // 平均模型：相电压由上桥臂占空比与母线电压线性映射
    out.ua = (out.duty_a - 0.5) * v_dc_safe;
    out.ub = (out.duty_b - 0.5) * v_dc_safe;
    out.uc = (out.duty_c - 0.5) * v_dc_safe;
    return out;
}

double InverterModel::estimate_inverter_dc_current(const AcEval& ac,
                                                   double ia,
                                                   double ib,
                                                   double ic,
                                                   double v_dc_in) const {
    const double p_ac = ac.ua * ia + ac.ub * ib + ac.uc * ic;
    const double v_safe = std::max(std::abs(v_dc_in), 1.0);
    return p_ac / v_safe;
}

InverterModel::DcEval InverterModel::evaluate_dc_bus(double v_dc_in,
                                                      double i_inv_dc_in,
                                                      const Command& cmd) const {
    DcEval out;
    out.v_dc_clamped = std::clamp(v_dc_in, vdc_min, vdc_max);

    out.i_frontend = std::clamp(G_frontend * (vdc_nominal - out.v_dc_clamped),
                                -i_frontend_max,
                                i_frontend_max);

    if (battery_enabled) {
        out.i_batt = std::clamp(cmd.i_batt_cmd, -i_batt_charge_max, i_batt_discharge_max);
    } else {
        out.i_batt = 0.0;
    }

    out.i_inv_dc = i_inv_dc_in;
    out.i_source = out.i_frontend + out.i_batt;
    out.p_frontend = out.v_dc_clamped * out.i_frontend;
    out.p_batt = out.v_dc_clamped * out.i_batt;
    out.p_inv = out.v_dc_clamped * out.i_inv_dc;

    if (fixed_vdc_mode) {
        out.dv_dc = 0.0;
    } else {
        const double C_safe = std::max(C_dc, 1e-9);
        out.dv_dc = (out.i_source - out.i_inv_dc) / C_safe;
    }
    return out;
}

void InverterModel::apply_state(double v_dc_in,
                                const SVPWMController::Output& svpwm,
                                const AcEval& ac,
                                const DcEval& dc_eval) {
    v_dc = fixed_vdc_mode ? vdc_nominal : std::clamp(v_dc_in, vdc_min, vdc_max);
    ua = ac.ua;
    ub = ac.ub;
    uc = ac.uc;
    duty_a = ac.duty_a;
    duty_b = ac.duty_b;
    duty_c = ac.duty_c;
    gate_u_a = svpwm.gate_u_a;
    gate_l_a = svpwm.gate_l_a;
    gate_u_b = svpwm.gate_u_b;
    gate_l_b = svpwm.gate_l_b;
    gate_u_c = svpwm.gate_u_c;
    gate_l_c = svpwm.gate_l_c;

    i_frontend = dc_eval.i_frontend;
    i_batt = dc_eval.i_batt;
    i_source = dc_eval.i_source;
    i_inv_dc = dc_eval.i_inv_dc;
    p_frontend = dc_eval.p_frontend;
    p_batt = dc_eval.p_batt;
    p_inv = dc_eval.p_inv;
}

void InverterModel::apply_config(const SimulationConfig::Inverter& inv_cfg,
                                 const SimulationConfig::BatteryInterface& batt_cfg) {
    fixed_vdc_mode = inv_cfg.fixed_vdc_mode;
    vdc_nominal = inv_cfg.vdc_nominal;
    vdc_min = inv_cfg.vdc_min;
    vdc_max = inv_cfg.vdc_max;
    C_dc = inv_cfg.C_dc;
    G_frontend = inv_cfg.G_frontend;
    i_frontend_max = inv_cfg.i_frontend_max;

    battery_enabled = batt_cfg.enabled;
    i_batt_charge_max = batt_cfg.i_charge_max;
    i_batt_discharge_max = batt_cfg.i_discharge_max;

    v_dc = std::clamp(vdc_nominal, vdc_min, vdc_max);
}
