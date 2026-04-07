#pragma once

#include "config/sim_config.h"

class MotorModel;

class DQCurrentController {
public:
    struct SensorSample {
        double id_meas = 0.0;
        double iq_meas = 0.0;
        double omega_e = 0.0;
        double v_dc = 540.0;
        const MotorModel* motor = nullptr;
        double linear_limit = 0.577350269;
    };

    class SensorInterface {
    public:
        virtual ~SensorInterface() = default;
        virtual SensorSample sample() const = 0;
    };

    struct Command {
        double id_ref = 0.0;
        double iq_ref = 0.0;
        double id_meas = 0.0;
        double iq_meas = 0.0;
        double omega_e = 0.0;
        double v_dc = 540.0;
        const MotorModel* motor = nullptr;
        double linear_limit = 0.577350269;  // SVPWM 线性调制区系数，Vphase_max / Vdc，固定为 sqrt(1/3)
    };

    struct Output {
        double ud_ref = 0.0;
        double uq_ref = 0.0;
        double ed = 0.0;
        double eq = 0.0;
        double modulation_scale = 1.0;  // 实际输出电压矢量与未限幅参考的比值，反映饱和程度，便于观测
    };

    DQCurrentController() = default;
    explicit DQCurrentController(double Ts_in);
    void set_Ts(double Ts_in);
    void reset();
    void apply_config(const SimulationConfig::CurrentLoop& cfg);
    Output evaluate(const Command& cmd);
    Output evaluate_with_sensor(double id_ref, double iq_ref, const SensorInterface& sensor);

private:
    double Ts = 1e-6;
    double kp_d = 0.2;
    double ki_d = 200.0;
    double kp_q = 0.2;
    double ki_q = 200.0;
    double anti_windup_gain = 500.0;

    double int_d = 0.0;
    double int_q = 0.0;
};
