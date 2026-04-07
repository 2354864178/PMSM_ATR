#include "controllers/current_controller.h"
#include "components/motor_model.h"

#include <algorithm>
#include <cassert>
#include <cmath>

DQCurrentController::DQCurrentController(double Ts_in) : Ts(Ts_in) {}

void DQCurrentController::set_Ts(double Ts_in) { Ts = Ts_in; }

void DQCurrentController::reset() {
    int_d = 0.0;    // PI 积分器状态清零
    int_q = 0.0;    // PI 积分器状态清零
}

void DQCurrentController::apply_config(const SimulationConfig::CurrentLoop& cfg) {
    kp_d = cfg.kp_d;
    ki_d = cfg.ki_d;
    kp_q = cfg.kp_q;
    ki_q = cfg.ki_q;
    anti_windup_gain = cfg.anti_windup_gain;
}

DQCurrentController::Output DQCurrentController::evaluate(const Command& cmd) {
    assert(cmd.motor != nullptr && "DQCurrentController::evaluate requires cmd.motor to be valid");

    Output out;
    out.ed = cmd.id_ref - cmd.id_meas;
    out.eq = cmd.iq_ref - cmd.iq_meas;

    const double vd_pi = kp_d * out.ed + int_d;
    const double vq_pi = kp_q * out.eq + int_q;

    const MotorModel* motor = cmd.motor;
    const double Rs = motor->Rs;
    const double Ld = motor->Ld;
    const double Lq = motor->Lq;
    const double psi_f = motor->psi_f;

    const double vd_ff = Rs * cmd.id_meas - cmd.omega_e * Lq * cmd.iq_meas;
    const double vq_ff = Rs * cmd.iq_meas + cmd.omega_e * Ld * cmd.id_meas + cmd.omega_e * psi_f;

    // const double vd_ff = 0; // 电驱动测试：置零前馈项，简化控制器行为
    // const double vq_ff = 0; // 电驱动测试：置零前馈项，简化控制器行为

    const double ud_unsat = vd_pi + vd_ff;  // PI 输出 + 前馈项
    const double uq_unsat = vq_pi + vq_ff;  // PI 输出 + 前馈项

    const double v_phase_limit = std::max(std::abs(cmd.v_dc), 1.0) * std::max(cmd.linear_limit, 0.0);
    const double v_mag = std::sqrt(ud_unsat * ud_unsat + uq_unsat * uq_unsat);
    out.modulation_scale = (v_mag > 1e-9) ? std::min(1.0, v_phase_limit / v_mag) : 1.0; 
    
    // 电驱动测试：当前直接输出未限幅参考，modulation_scale 仅保留用于观测。
    // out.ud_ref = ud_unsat * out.modulation_scale;  // 饱和处理，保持电压矢量方向不变
    // out.uq_ref = uq_unsat * out.modulation_scale;  // 饱和处理，保持电压矢量方向不变

    out.ud_ref = ud_unsat;
    out.uq_ref = uq_unsat;

    const double du_d = out.ud_ref - ud_unsat;  // 饱和引起的反向修正项
    const double du_q = out.uq_ref - uq_unsat;  // 饱和引起的反向修正项

    // const double du_d = 0;  // 电驱动测试：置零饱和修正项，简化控制器行为
    // const double du_q = 0;  // 电驱动测试：置零饱和修正项，简化控制器行为

    int_d += (ki_d * out.ed + anti_windup_gain * du_d) * Ts;    // PI 积分器更新
    int_q += (ki_q * out.eq + anti_windup_gain * du_q) * Ts;    // PI 积分器更新

    return out;
}

DQCurrentController::Output DQCurrentController::evaluate_with_sensor(double id_ref,
                                                                      double iq_ref,
                                                                      const SensorInterface& sensor) {
    const SensorSample sample = sensor.sample();

    Command cmd;
    cmd.id_ref = id_ref;
    cmd.iq_ref = iq_ref;
    cmd.id_meas = sample.id_meas;
    cmd.iq_meas = sample.iq_meas;
    cmd.omega_e = sample.omega_e;
    cmd.v_dc = sample.v_dc;
    cmd.motor = sample.motor;
    cmd.linear_limit = sample.linear_limit;
    return evaluate(cmd);
}
