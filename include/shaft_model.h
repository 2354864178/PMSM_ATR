#pragma once

// 轴系模型声明
class ShaftModel {
public:
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

    void update_mechanics(double T_em, double T_turb, double T_pump);

private:
    double Ts = 0.0001;
};
