#pragma once

#include "components/motor_model.h"
#include "components/pump_model.h"
#include "components/shaft_model.h"
#include "components/turbine_model.h"

// 系统“动态状态”向量：会被 RK4 积分推进的变量。
struct SystemState {
    double id = 0.0;
    double iq = 0.0;
    double theta_mech = 0.0;
    double omega_mech = 0.0;
    double p_plenum = 150000.0;
    double p_discharge = 100000.0;
};

// 包含系统的代数变量（如电机电流、电压，涡轮扭矩等），用于 RK4 步进计算中间状态评估和模型状态同步
struct SystemAlgebraic {
    MotorModel::ElectricalEval motor;
    TurbineModel::GasEval turbine;
    PumpModel::HydraulicEval pump;
};

// 包含系统状态的时间导数，用于 RK4 步进计算
struct SystemDeriv {
    double did = 0.0;   // id 的时间导数
    double diq = 0.0;   // iq 的时间导数
    double dtheta_mech = 0.0;   // 机械角位置的时间导数（即机械角速度）
    double domega_mech = 0.0;   // 机械角速度的时间导数（即机械角加速度）
    double dp_plenum = 0.0;     // 腔体压力的时间导数
    double dp_discharge = 0.0;  // 出口压力的时间导数
};

// 全系统 RK4 求解器：
// - 负责耦合顺序（电机->涡轮->泵->轴系）
// - 负责 RK4 子步推进
// - 负责把最终结果回写到各模型对象
class SystemRK4Solver {
public:
    SystemRK4Solver(MotorModel& motor_in,
                    TurbineModel& turb_in,
                    PumpModel& pump_in,
                    ShaftModel& shaft_in,
                    double Ts_in);

    void sync_from_models();
    // 推进一步：输入当前三相电压（来自主循环或控制器）。
    void step(double ua, double ub, double uc);

private:
    // 单次“评估”结果：导数 + 代数量。
    struct EvalResult {
        SystemDeriv deriv;
        SystemAlgebraic alg;
    };

    // 各组件本次评估原始输出。
    struct ComponentEvals {
        MotorModel::ElectricalEval motor;
        TurbineModel::GasEval turbine;
        PumpModel::HydraulicEval pump;
        ShaftModel::MechanicalEval shaft;
    };

    // 对某个状态做一次耦合评估。
    EvalResult evaluate(const SystemState& state, double ua, double ub, double uc) const;
    // 先分别调用各组件 evaluate_*。
    ComponentEvals evaluate_components(const SystemState& state, double ua, double ub, double uc) const;
    // 从组件结果提取状态导数。
    void fill_derivatives(const ComponentEvals& evals, SystemDeriv& deriv) const;
    // 从组件结果提取用于最终回写的代数量。
    void fill_algebraic(const ComponentEvals& evals, SystemAlgebraic& alg) const;
    // 将最终状态和代数量回写到各模型，便于日志与外部读取。
    void sync_to_models(const SystemAlgebraic& alg, double ua, double ub, double uc);

    MotorModel& motor;
    TurbineModel& turb;
    PumpModel& pump;
    ShaftModel& shaft;
    double Ts = 1e-4;
    SystemState state;
};
