#include "controllers/speed_controller.h"

#include <algorithm>

SpeedController::SpeedController(double Ts_in) : Ts(Ts_in) {}

void SpeedController::set_Ts(double Ts_in) { Ts = Ts_in; }

void SpeedController::reset() { int_omega = 0.0; }

void SpeedController::apply_config(const SimulationConfig::SpeedLoop& cfg) {
    kp = cfg.kp;
    ki = cfg.ki;
    iq_ref_min = cfg.iq_ref_min;
    iq_ref_max = cfg.iq_ref_max;
    anti_windup_gain = cfg.anti_windup_gain;
}

SpeedController::Output SpeedController::evaluate(double omega_ref_mech, double omega_mech) {
    Output out;
    out.e_omega = omega_ref_mech - omega_mech;

    const double iq_unsat = kp * out.e_omega + int_omega;
    out.iq_ref = std::clamp(iq_unsat, iq_ref_min, iq_ref_max);
    const double diq_aw = out.iq_ref - iq_unsat;

    int_omega += (ki * out.e_omega + anti_windup_gain * diq_aw) * Ts;
    return out;
}
