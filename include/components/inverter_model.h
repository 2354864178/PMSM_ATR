#pragma once

#include <algorithm>

#include "controllers/svpwm_controller.h"
#include "config/sim_config.h"

class InverterModel {
public:
    struct AcEval {
        double ua = 0.0;
        double ub = 0.0;
        double uc = 0.0;
        double duty_a = 0.5;
        double duty_b = 0.5;
        double duty_c = 0.5;
    };

    struct DcEval {
        double dv_dc = 0.0;
        double i_frontend = 0.0;
        double i_batt = 0.0;
        double i_source = 0.0;
        double i_inv_dc = 0.0;
        double p_frontend = 0.0;
        double p_batt = 0.0;
        double p_inv = 0.0;
        double v_dc_clamped = 0.0;
    };

    struct Command {
        double i_batt_cmd = 0.0;
    };

    InverterModel() = default;
    explicit InverterModel(double Ts_in);
    void set_Ts(double Ts_in);

    bool fixed_vdc_mode = true;
    double vdc_nominal = 540.0;
    double vdc_min = 100.0;
    double vdc_max = 800.0;
    double C_dc = 5e-3;
    double G_frontend = 40.0;
    double i_frontend_max = 400.0;

    bool battery_enabled = false;
    double i_batt_charge_max = 120.0;
    double i_batt_discharge_max = 120.0;

    double v_dc = 540.0;
    double ua = 0.0;
    double ub = 0.0;
    double uc = 0.0;
    double duty_a = 0.5;
    double duty_b = 0.5;
    double duty_c = 0.5;
    double gate_u_a = 0.5;
    double gate_l_a = 0.5;
    double gate_u_b = 0.5;
    double gate_l_b = 0.5;
    double gate_u_c = 0.5;
    double gate_l_c = 0.5;
    double i_frontend = 0.0;
    double i_batt = 0.0;
    double i_source = 0.0;
    double i_inv_dc = 0.0;
    double p_frontend = 0.0;
    double p_batt = 0.0;
    double p_inv = 0.0;

    AcEval evaluate_ac_side(double v_dc_in, const SVPWMController::Output& svpwm) const;
    double estimate_inverter_dc_current(const AcEval& ac,
                                        double ia,
                                        double ib,
                                        double ic,
                                        double v_dc_in) const;
    DcEval evaluate_dc_bus(double v_dc_in, double i_inv_dc_in, const Command& cmd) const;
    void apply_state(double v_dc_in,
                     const SVPWMController::Output& svpwm,
                     const AcEval& ac,
                     const DcEval& dc_eval);

    void apply_config(const SimulationConfig::Inverter& inv_cfg,
                      const SimulationConfig::BatteryInterface& batt_cfg);

private:
    double Ts = 1e-6;
};
