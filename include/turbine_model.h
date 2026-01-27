#pragma once
#include <algorithm>
#include <cmath>

// 简化涡轮气动至轴扭矩映射
class TurbineModel {
public:
    TurbineModel() = default;
    explicit TurbineModel(double Ts_in);
    void set_Ts(double Ts_in);

    // 气动与机械参数
    double gamma = 1.4;             // 比热比
    double gas_R = 287.0;           // 比气体常数 J/(kg·K)
    double eta_turb = 0.9;          // 涡轮等熵效率（0-1）
    double omega_t = 0.0;           // 涡轮角速度 rad/s
    double omega_floor = 0.00001;   // 防止零速除法的下限
    double b_turb = 1e-04;          // 涡轮摩擦系数（N·m·s），用于轴系集中计算（默认估计）
    double J_turb = 0.01;           // 涡轮转动惯量 kg·m²（默认估计）

    double p_in = 101325.0;         // 入口压力 Pa
    double p_out = 100000.0;        // 出口压力 Pa
    double T_in = 600.0;            // 入口温度 K
    double m_dot = 1.0;             // 质量流量 kg/s

    double T_turb = 0.0;            // 轴扭矩 N·m

    void update_from_gas(double omega_shaft);

private:
    double Ts = 0.0001; // 采样时间 s
};
