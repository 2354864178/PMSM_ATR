#pragma once

#include "components/motor_model.h"
#include "components/pump_model.h"
#include "components/shaft_model.h"
#include "components/inverter_model.h"
#include "components/turbine_model.h"
#include "controllers/svpwm_controller.h"

// 系统“动态状态”向量：会被 RK4 积分推进的变量。
struct SystemState {
    double id = 0.0;
    double iq = 0.0;
    double theta_mech = 0.0;
    double omega_mech = 0.0;
    double p_plenum = 150000.0;
    double p_downstream_turb = 100000.0;
    double p_discharge = 100000.0;
    double v_dc = 540.0;
};

// 包含系统的代数变量（如电机电流、电压，涡轮扭矩等），用于 RK4 步进计算中间状态评估和模型状态同步
struct SystemAlgebraic {
    MotorModel::ElectricalEval motor;
    TurbineModel::GasEval turbine;
    PumpModel::HydraulicEval pump;
    SVPWMController::Output inverter_svpwm;
    InverterModel::AcEval inverter_ac;
    InverterModel::DcEval inverter_dc;
};

// 包含系统状态的时间导数，用于 RK4 步进计算
struct SystemDeriv {
    double did = 0.0;   // id 的时间导数
    double diq = 0.0;   // iq 的时间导数
    double dtheta_mech = 0.0;   // 机械角位置的时间导数（即机械角速度）
    double domega_mech = 0.0;   // 机械角速度的时间导数（即机械角加速度）
    double dp_plenum = 0.0;     // 腔体压力的时间导数
    double dp_downstream_turb = 0.0; // 涡轮后腔压力时间导数
    double dp_discharge = 0.0;  // 出口压力的时间导数
    double dv_dc = 0.0;         // 直流母线电压时间导数
};

// 全系统 RK4 求解器：
// - 负责耦合顺序（电机->涡轮->泵->轴系）
// - 负责 RK4 子步推进
// - 负责把最终结果回写到各模型对象
class SystemRK4Solver {
public:
    struct ComponentSwitches {
        bool motor_enabled = true;
        bool turbine_enabled = true;
        bool pump_enabled = true;
        bool inverter_enabled = true;
        bool svpwm_enabled = true;
    };

    SystemRK4Solver(MotorModel& motor_in,
                    TurbineModel& turb_in,
                    PumpModel& pump_in,
                    SVPWMController& svpwm_in,
                    InverterModel& inverter_in,
                    ShaftModel& shaft_in,
                    double Ts_in);

    void sync_from_models();
    void set_inverter_command(const InverterModel::Command& cmd);
    void set_turbine_command(const TurbineModel::Command& cmd);
    void set_pump_command(const PumpModel::Command& cmd);
    void set_component_switches(const ComponentSwitches& switches);
    void set_fixed_speed_mode(bool enabled, double omega_mech_fixed);
    // 推进一步：输入当前 dq 电压参考（SVPWM 调制前）。
    void step(double ud_ref, double uq_ref);

private:
    // 单次“评估”结果：导数 + 代数量。
    struct EvalResult {
        SystemDeriv deriv;
        SystemAlgebraic alg;
    };

    // 各组件本次评估原始输出。
    struct ComponentEvals {
        SVPWMController::Output inverter_svpwm;
        InverterModel::AcEval inverter_ac;
        MotorModel::ElectricalEval motor;
        TurbineModel::GasEval turbine;
        PumpModel::HydraulicEval pump;
        InverterModel::DcEval inverter_dc;
        ShaftModel::MechanicalEval shaft;
    };

    // 对某个状态做一次耦合评估。
    EvalResult evaluate(const SystemState& state, double ud_ref, double uq_ref) const;
    // 先分别调用各组件 evaluate_*。
    ComponentEvals evaluate_components(const SystemState& state, double ud_ref, double uq_ref) const;
    SVPWMController::Output eval_svpwm_stage(const SystemState& state,
                                             double ud_ref,
                                             double uq_ref,
                                             double theta_e) const;
    InverterModel::AcEval eval_inverter_ac_stage(const SystemState& state,
                                                 const SVPWMController::Output& svpwm_out) const;
    MotorModel::ElectricalEval eval_motor_stage(const SystemState& state,
                                                const InverterModel::AcEval& inv_ac,
                                                double theta_e) const;
    TurbineModel::GasEval eval_turbine_stage(const SystemState& state) const;
    PumpModel::HydraulicEval eval_pump_stage(const SystemState& state) const;
    InverterModel::DcEval eval_inverter_dc_stage(const SystemState& state,
                                                 const InverterModel::AcEval& inv_ac,
                                                 const MotorModel::ElectricalEval& motor_eval) const;
    ShaftModel::MechanicalEval eval_shaft_stage(const SystemState& state,
                                                const MotorModel::ElectricalEval& motor_eval,
                                                const TurbineModel::GasEval& turb_eval,
                                                const PumpModel::HydraulicEval& pump_eval) const;
    SystemAlgebraic make_algebraic(const ComponentEvals& evals) const;
    SystemDeriv make_derivative(const ComponentEvals& evals) const;
    // 将最终状态和代数量回写到各模型，便于日志与外部读取。
    void sync_to_models(const SystemAlgebraic& alg);
    void apply_fixed_speed_constraint(SystemState& s) const;
    void clamp_state_bounds(SystemState& s) const;
    double electrical_theta(const SystemState& s) const;

    MotorModel& motor;
    TurbineModel& turb;
    PumpModel& pump;
    SVPWMController& svpwm;
    InverterModel& inverter;
    ShaftModel& shaft;
    double Ts = 1e-4;
    SystemState state;
    InverterModel::Command inverter_cmd;
    TurbineModel::Command turbine_cmd;
    PumpModel::Command pump_cmd;
    ComponentSwitches component_switches;
    bool fixed_speed_mode = false;
    double omega_mech_fixed = 0.0;
};
