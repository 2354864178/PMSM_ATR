#pragma once
#include "config/sim_config.h"

// 轴系模型（刚性同轴简化）：
// - 状态主变量：theta_mech、omega_mech
// - 输入：电机/涡轮/泵三路扭矩
class ShaftModel {
public:
    // 单次机械评估输出。
    struct MechanicalEval {
        double dtheta_mech = 0.0;
        double domega_mech = 0.0;
    };

    ShaftModel() = default;
    explicit ShaftModel(double Ts_in);
    void set_Ts(double Ts_in);

    double J_motor = 0.0001;    // 电机侧转动惯量 kg.m²
    double J_turb  = 0.01;      // 涡轮侧转动惯量 kg.m²
    double J_pump = 0;          // 泵侧转动惯量 kg.m²

    double b_motor = 5e-05;     // 电机侧摩擦系数（N·m·s）
    double b_turb  = 1e-04;     // 涡轮侧摩擦系数（N·m·s）
    double b_pump = 0;          // 泵侧摩擦系数（N·m·s）

    double omega_mech = 0.0;    // 机械角速度 rad/s
    double theta_mech = 0.0;    // 机械角位置 rad

    // 纯计算：给定当前速度与三路扭矩，返回机械导数。
    MechanicalEval evaluate_mechanics(double omega_mech_in, double T_em, double T_turb, double T_pump) const;
    // 回写机械状态。
    void apply_mechanical_state(double theta_mech_in, double omega_mech_in);
    // 参数下发。
    void apply_config(const SimulationConfig::Shaft& cfg);

private:
    double Ts = 0.0001;
};
