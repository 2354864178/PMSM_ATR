#pragma once
#include <cmath>

// 电机模型声明
class MotorModel {
public:
    MotorModel() = default;
    explicit MotorModel(double Ts_in);
    void set_Ts(double Ts_in);

    double Rs   = 0.0051;           // 定子电阻 Ohm
    double Ld   = 0.0000346;        // 对称电机电感 H
    double Lq   = 0.0000346;        // 对称电机电感 H
    double psi_f = 0.026;           // 永磁体磁链 Wb
    int p       = 2;                // 极对数
    double b_motor = 5e-05;         // 电机摩擦系数（N·m·s），用于轴系集中计算
    double J_motor = 0.0001;        // 转动惯量 kg·m²

    double ua = 0.0, ub = 0.0, uc = 0.0;    // 三相电压 V
    double ia = 0.0, ib = 0.0, ic = 0.0;    // 三相电流 A
    double id = 0.0, iq = 0.0;              // dq 轴电流 A
    double ud = 0.0, uq = 0.0;              // dq 轴电压 V
    double i_alpha = 0.0, i_beta = 0.0;     // Clarke 变换后电流 A
    double u_alpha = 0.0, u_beta = 0.0;     // Clarke 变换后电压 V
    double omega_e = 0.0;                   // 电角速度 rad/s
    double theta_e = 0.0;                   // 电角度 rad
    double T_em = 0.0;                      // 电磁转矩 N·m

    void update_electrical(double omega_mech);

private:
    double Ts = 0.0001;               // 采样时间 s
};
