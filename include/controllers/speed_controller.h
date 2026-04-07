#pragma once

#include "config/sim_config.h"

class SpeedController {
public:
    struct Output {
        double iq_ref = 0.0;
        double e_omega = 0.0;
    };

    SpeedController() = default;
    explicit SpeedController(double Ts_in);

    void set_Ts(double Ts_in);
    void reset();
    void apply_config(const SimulationConfig::SpeedLoop& cfg);
    Output evaluate(double omega_ref_mech, double omega_mech);

private:
    double Ts = 1e-6;
    double kp = 0.0;
    double ki = 0.0;
    double iq_ref_min = -120.0;
    double iq_ref_max = 120.0;
    double anti_windup_gain = 0.0;

    double int_omega = 0.0;
};
