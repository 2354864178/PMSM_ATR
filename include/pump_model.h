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
    double Q = 0.0;                 // 流量 m³/s
    double omega_p =0.0;            // 泵转速
    double H = 0.0;                 // 扬程 m
    double eta_p = 1;               // 泵效率
    double T_pump = 0.0;            // 负载扭矩（始终为正，抵抗旋转）
    double J_pump = 0;              // 泵转动惯量
    double b_pump = 0;              // 泵摩擦系数
    double omega_floor = 1e-6;      // 零速保护 rad/s

    // 根据轴速更新负载扭矩
    void update_from_hydraulics(double omega_shaft);

private:
    double Ts = 0.0001;             // 采样时间 s（当前未用）
};
