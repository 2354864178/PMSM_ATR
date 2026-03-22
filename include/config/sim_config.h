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
        double rho = 1000.0;                // 流体密度 kg/m^3
        double eta_p = 0.8;                 // 泵效率（0-1），用于计算实际压升和轴功率
        double p_suction = 100000.0;        // 吸入口压力 Pa
        double p_downstream = 100000.0;     // 出口压力 Pa（当前版本为定值，后续可扩展为动态变量）
        double p_discharge = 100000.0;      // 泵出口压力 Pa（当前版本为定值，后续可扩展为动态变量）
        double V_out = 3e-3;                // 出口容积 m^3，影响出口压力动态响应，当前版本为定值，后续可扩展为动态变量
        double beta_eff = 1.5e9;            // 压升-流量关系二次项系数，影响泵的非线性特性，当前版本为定值，后续可扩展为动态变量
        double R_out = 2e8;                 // 出口容积压力-流量关系线性项系数，影响泵的线性特性，当前版本为定值，后续可扩展为动态变量
        double K_q = 2e-5;                  // 流量-压升关系线性项系数，影响泵的线性特性，当前版本为定值，后续可扩展为动态变量
        double K_slip = 2e-12;              // 滑移流量系数，影响泵在低流量时的压升特性，当前版本为定值，后续可扩展为动态变量
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

    // SVPWM+三相逆变器功率级参数（含DC母线）：
    // - fixed_vdc_mode=true 时母线电压固定在 vdc_nominal（当前默认 540V）
    // - false 时根据 Cdc 动态积分
    struct Inverter {
        bool fixed_vdc_mode = true; 
        double vdc_nominal = 540.0; // 母线标称电压 V
        double vdc_min = 100.0;     // 母线电压范围限制
        double vdc_max = 800.0;     // 母线电压范围限制
        double C_dc = 5e-3;         // 母线电容 F，影响母线电压动态响应
        double G_frontend = 40.0;   // 前端供电导纳 S，影响母线电压调节能力
        double i_frontend_max = 400.0;  // 前端最大电流 A
    } inverter;

    // SVPWM 控制器参数
    struct SVPWM {
        double linear_limit = 0.577350269; // 线性调制区系数（Vphase_max / Vdc）
    } svpwm;

    // 双向电池接口参数（预留）
    // - i_batt > 0: 电池向母线放电
    // - i_batt < 0: 母线给电池充电
    struct BatteryInterface {
        bool enabled = false;
        double i_charge_max = 120.0;    // 电池最大充电电流 A
        double i_discharge_max = 120.0; // 电池最大放电电流 A
    } battery;

    // 外部输入：当前为开环常值输入，可扩展为时变输入源。
    struct Inputs {
        double ud = 0.0; // SVPWM 调制前 d 轴电压参考 V
        double uq = 0.5; // SVPWM 调制前 q 轴电压参考 V
        double turb_p_in = 100000.0;
        double turb_p_out = 100000.0;
        double turb_T_in = 600.0;
        double turb_m_dot = 0.2;
        double i_batt_cmd = 0.0;
    } inputs;
};
