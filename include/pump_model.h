#pragma once
#include <algorithm>
#include <cmath>

// 简化离心泵液动到轴负载扭矩映射
class PumpModel {
public:
    PumpModel() = default;
    explicit PumpModel(double Ts_in);
    void set_Ts(double Ts_in);

    // 参数（可扩展）
    double rho = 1000.0;            // 流体密度 kg/m³
    double Q = 0.0;                 // 系统出口流量 m³/s（输出）
    double omega_p =0.0;            // 泵转速
    double H = 0.0;                 // 扬程 m
    double eta_p = 1;               // 泵效率
    double T_pump = 0.0;            // 负载扭矩（始终为正，抵抗旋转）
    double J_pump = 0;              // 泵转动惯量
    double b_pump = 0;              // 泵摩擦系数
    double omega_floor = 50;      // 零速保护 rad/s

    // 容积动态参数（泵出口腔体/管路等效）
    double p_suction = 100000.0;    // 吸入口压力 Pa
    double p_downstream = 100000.0; // 下游压力 Pa
    double p_discharge = 100000.0;  // 出口压力状态 Pa
    double V_out = 3e-3;            // 出口等效容积 m^3
    double beta_eff = 1.5e9;        // 流体体积弹性模量 Pa
    double R_out = 2e8;             // 下游等效流阻 Pa/(m^3/s)
    double K_q = 2e-5;              // 泵转速到流量系数 m^3/(s·rad)
    double K_slip = 2e-12;          // 压差泄漏系数 (m^3/s)/Pa
    double p_floor = 20000.0;       // 压力下限 Pa

    // 容积动态中间量
    double Q_pump = 0.0;            // 泵产生流量 m^3/s
    double Q_out = 0.0;             // 流向下游流量 m^3/s
    double dp_pump = 0.0;           // 泵压升 Pa

    // 根据轴速更新负载扭矩
    void update_from_hydraulics(double omega_shaft);

private:
    double Ts = 0.0001;             // 采样时间 s
};
