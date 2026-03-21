#pragma once

#include <string>

// 仿真配置总结构：
// - 由 main 创建并下发到各部件 apply_config。
// - 当前版本使用代码内默认值，后续可扩展为文件加载。
struct SimulationConfig {
    // 运行时配置：决定仿真时长、步长和日志行为。
    struct Runtime {
        double Ts = 1e-6;           // 采样时间 s
        double total_time = 0.6;    // 仿真总时间 s
        int log_every_n_steps = 10; // 每隔多少步记录一次数据
        std::string log_path = "build/log.xlsx";
    } runtime;

    // 电机参数：PMSM dq 模型所需的电气参数。
    struct Motor {
        double Rs = 0.0051;     // 定子电阻 Ohm
        double Ld = 0.0000346;  // 电机d轴电感 H
        double Lq = 0.0000346;  // 电机q轴电感 H
        double psi_f = 0.026;   // 永磁体磁链 Wb
        int p = 2;              // 极对数
    } motor;

    // 涡轮参数：可压缩流动与容积动态模型参数。
    struct Turbine {
        double gamma = 1.4;         // 比热比
        double gas_R = 287.0;       // 比气体常数 J/(kg·K)
        double eta_turb = 0.9;      // 涡轮等熵效率（0-1），用于计算实际出口温度和轴功率
        double V_plenum = 5e-3;     // 前腔体积 m^3，影响腔体压力动态响应
        double A_nozzle = 1.2e-4;   // 喷嘴等效面积 m^2，影响通过涡轮的质量流量和压力损失
        double Cd_nozzle = 0.85;    // 喷嘴流量系数，考虑喷嘴非理想流动损失，影响通过涡轮的质量流量计算
        double C_in = 1.0;          // 入口流量系数，影响进腔质量流量的计算，考虑入口流动损失和非理想效应
    } turbine;

    // 泵参数：流量-压升关系与出口容积动态参数。
    struct Pump {
        double rho = 1000.0;
        double eta_p = 0.8;
        double p_suction = 100000.0;
        double p_downstream = 120000.0;
        double p_discharge = 100000.0;
        double V_out = 3e-3;
        double beta_eff = 1.5e9;
        double R_out = 2e8;
        double K_q = 2e-5;
        double K_slip = 2e-12;
    } pump;

    // 轴系参数：等效惯量与阻尼，统一在机械方程中使用。
    struct Shaft {
        double b_motor = 0.0009;
        double b_turb = 0.0;
        double b_pump = 0.0;
        double J_motor = 0.000772;
        double J_turb = 0.0;
        double J_pump = 0.0;
    } shaft;

    // 外部输入：当前为开环常值输入，可扩展为时变输入源。
    struct Inputs {
        double ua = 0.0;
        double ub = 0.0;
        double uc = 0.5;
        double turb_p_in = 220000.0;
        double turb_p_out = 100000.0;
        double turb_T_in = 600.0;
        double turb_m_dot = 0.2;
    } inputs;
};
