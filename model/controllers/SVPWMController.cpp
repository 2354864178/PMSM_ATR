#include "controllers/svpwm_controller.h"

#include <algorithm>
#include <cmath>

SVPWMController::SVPWMController(double Ts_in) : Ts(Ts_in) {}

void SVPWMController::set_Ts(double Ts_in) { Ts = Ts_in; }

SVPWMController::Output SVPWMController::evaluate(double v_dc_in,
                                                  double ud_ref,
                                                  double uq_ref,
                                                  double theta_e) const {
    Output out;
    const double v_dc_safe = std::max(std::abs(v_dc_in), 1.0);

    const double cos_theta = std::cos(theta_e);
    const double sin_theta = std::sin(theta_e);
    const double u_alpha = ud_ref * cos_theta - uq_ref * sin_theta;
    const double u_beta = ud_ref * sin_theta + uq_ref * cos_theta;
    const double ua_ref = u_alpha;
    const double ub_ref = -0.5 * u_alpha + (std::sqrt(3.0) / 2.0) * u_beta;
    const double uc_ref = -0.5 * u_alpha - (std::sqrt(3.0) / 2.0) * u_beta;

    const double v_max_ref = std::max({ua_ref, ub_ref, uc_ref});
    const double v_min_ref = std::min({ua_ref, ub_ref, uc_ref});
    const double v_offset = -0.5 * (v_max_ref + v_min_ref);

    double ua_cmd = ua_ref + v_offset;
    double ub_cmd = ub_ref + v_offset;
    double uc_cmd = uc_ref + v_offset;

    const double v_phase_limit = linear_limit * v_dc_safe;
    const double max_abs_cmd = std::max({std::abs(ua_cmd), std::abs(ub_cmd), std::abs(uc_cmd), 1e-9});
    out.modulation_scale = std::min(1.0, v_phase_limit / max_abs_cmd);
    ua_cmd *= out.modulation_scale;
    ub_cmd *= out.modulation_scale;
    uc_cmd *= out.modulation_scale;

    out.duty_a = std::clamp(0.5 + ua_cmd / v_dc_safe, 0.0, 1.0);
    out.duty_b = std::clamp(0.5 + ub_cmd / v_dc_safe, 0.0, 1.0);
    out.duty_c = std::clamp(0.5 + uc_cmd / v_dc_safe, 0.0, 1.0);

    // 六路桥臂开关控制信号（平均模型中用占空比表示）
    out.gate_u_a = out.duty_a;
    out.gate_l_a = 1.0 - out.duty_a;
    out.gate_u_b = out.duty_b;
    out.gate_l_b = 1.0 - out.duty_b;
    out.gate_u_c = out.duty_c;
    out.gate_l_c = 1.0 - out.duty_c;
    return out;
}

void SVPWMController::apply_config(const SimulationConfig::SVPWM& cfg) {
    linear_limit = cfg.linear_limit;
}
