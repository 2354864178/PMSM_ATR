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
    double omega_floor = 50;        // 防止零速除法的下限
    double b_turb = 1e-04;          // 涡轮摩擦系数（N·m·s），用于轴系集中计算（默认估计）
    double J_turb = 0.01;           // 涡轮转动惯量 kg·m²（默认估计）

    double p_in = 200000.0;         // 供气压力（上游）Pa
    double p_out = 100000.0;        // 下游背压 Pa
    double T_in = 600.0;            // 入口总温 K
    double m_dot = 0.2;             // 进气系数（kg/s 量级）

    // 容积模型参数（简化涡轮前腔体）
    double V_plenum = 5e-3;         // 前腔体积 m^3
    double C_in = 1.0;              // 入口流量系数
    double Cd_nozzle = 0.85;        // 喷嘴流量系数
    double A_nozzle = 1.2e-4;       // 喷嘴等效面积 m^2
    double p_floor = 20000.0;       // 压力下限保护 Pa

    // 容积模型状态/中间量
    double p_plenum = 150000.0;     // 腔体压力状态 Pa
    double m_dot_in = 0.0;          // 进腔质量流量 kg/s
    double m_dot_turb = 0.0;        // 通过涡轮质量流量 kg/s

    double T_turb = 0.0;            // 轴扭矩 N·m

    void update_from_gas(double omega_shaft);

private:
    double Ts = 0.0001; // 采样时间 s
};
