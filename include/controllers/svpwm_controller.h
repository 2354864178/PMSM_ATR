#pragma once

#include <algorithm>

#include "config/sim_config.h"

class SVPWMController {
public:
    struct Output {
        double duty_a = 0.5;
        double duty_b = 0.5;
        double duty_c = 0.5;
        double gate_u_a = 0.5;
        double gate_l_a = 0.5;
        double gate_u_b = 0.5;
        double gate_l_b = 0.5;
        double gate_u_c = 0.5;
        double gate_l_c = 0.5;
        double modulation_scale = 1.0;
    };

    SVPWMController() = default;
    explicit SVPWMController(double Ts_in);
    void set_Ts(double Ts_in);

    double linear_limit = 0.577350269;

    Output evaluate(double v_dc_in,
                    double ud_ref,
                    double uq_ref,
                    double theta_e) const;

    void apply_config(const SimulationConfig::SVPWM& cfg);

private:
    double Ts = 1e-6;
};
