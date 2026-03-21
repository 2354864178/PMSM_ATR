#pragma once
#include <cmath>
#include "config/sim_config.h"

// 电机模型（PMSM dq）：
// - 状态主变量：id、iq（电流状态）
// - 代数量：ud/uq、ia/ib/ic、T_em 等（由当前状态计算得到）
class MotorModel {
public:
    // 单次电气评估结果：
    // - did/diq 用于积分推进
    // - 其余量用于耦合和回写
    struct ElectricalEval {
        double did = 0.0;
        double diq = 0.0;
        double theta_e = 0.0;
        double omega_e = 0.0;
        double ud = 0.0;
        double uq = 0.0;
        double i_alpha = 0.0;
        double i_beta = 0.0;
        double ia = 0.0;
        double ib = 0.0;
        double ic = 0.0;
        double T_em = 0.0;
    };

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

    // 纯计算接口：不给对象写状态，只返回评估结果。
    ElectricalEval evaluate_electrical(double id_in,
                                       double iq_in,
                                       double theta_mech,
                                       double omega_mech,
                                       double ua_in,
                                       double ub_in,
                                       double uc_in) const;

    // 回写接口：把某次评估结果写入对象成员，便于外部读取/日志记录。
    void apply_electrical_state(double ua_in,
                                double ub_in,
                                double uc_in,
                                double id_in,
                                double iq_in,
                                const ElectricalEval& eval);

    // 参数下发接口：从统一配置结构体加载参数。
    void apply_config(const SimulationConfig::Motor& cfg);

private:
    double Ts = 0.0001;               // 采样时间 s
};
