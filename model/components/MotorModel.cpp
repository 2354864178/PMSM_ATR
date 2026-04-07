#include "components/motor_model.h"
#include <algorithm>
#include <cmath>

// 构造函数：记录仿真步长（当前主要用于保持接口一致性）。
MotorModel::MotorModel(double Ts_in) : Ts(Ts_in) {}
void MotorModel::set_Ts(double Ts_in) { Ts = Ts_in; }

// 电机电气侧评估（纯计算，不修改对象状态）：
MotorModel::ElectricalEval MotorModel::evaluate_electrical(double id_in,
                                                           double iq_in,
                                                           double theta_mech,
                                                           double omega_mech,
                                                           double ua_in,
                                                           double ub_in,
                                                           double uc_in) const {
    ElectricalEval out;
    // 机械到电气映射（极对数 p）
    out.omega_e = p * omega_mech;
    out.theta_e = p * theta_mech;

    // Clarke 变换：abc -> alpha-beta
    const double u_alpha = (2.0 / 3.0) * (ua_in - 0.5 * ub_in - 0.5 * uc_in);
    const double u_beta = (2.0 / 3.0) * ((std::sqrt(3.0) / 2.0) * ub_in - (std::sqrt(3.0) / 2.0) * uc_in);

    // Park 变换：alpha-beta -> dq
    const double cos_theta = std::cos(out.theta_e);
    const double sin_theta = std::sin(out.theta_e);
    out.ud = u_alpha * cos_theta + u_beta * sin_theta;
    out.uq = -u_alpha * sin_theta + u_beta * cos_theta;

    // dq 电流微分方程
    const double Ld_safe = std::max(Ld, 1e-9);
    const double Lq_safe = std::max(Lq, 1e-9);
    out.did = (out.ud - Rs * id_in + out.omega_e * Lq * iq_in) / Ld_safe;
    out.diq = (out.uq - Rs * iq_in - out.omega_e * Ld * id_in - out.omega_e * psi_f) / Lq_safe;

    // 逆 Park + 逆 Clarke：dq -> alpha-beta -> abc（输出观测）
    out.i_alpha = id_in * cos_theta - iq_in * sin_theta;
    out.i_beta = id_in * sin_theta + iq_in * cos_theta;
    out.ia = out.i_alpha;
    out.ib = -0.5 * out.i_alpha + (std::sqrt(3.0) / 2.0) * out.i_beta;
    out.ic = -0.5 * out.i_alpha - (std::sqrt(3.0) / 2.0) * out.i_beta;

    // PMSM 转矩方程
    out.T_em = 1.5 * (p / 2.0) * (psi_f * iq_in + (Ld - Lq) * id_in * iq_in);
    return out;
}

// 将评估结果回写到对象成员，供日志与外部模块读取。
void MotorModel::apply_electrical_state(double ua_in,
                                        double ub_in,
                                        double uc_in,
                                        double id_in,
                                        double iq_in,
                                        const ElectricalEval& eval) {
    ua = ua_in;
    ub = ub_in;
    uc = uc_in;
    id = id_in;
    iq = iq_in;
    theta_e = eval.theta_e;
    omega_e = eval.omega_e;
    ud = eval.ud;
    uq = eval.uq;
    i_alpha = eval.i_alpha;
    i_beta = eval.i_beta;
    ia = eval.ia;
    ib = eval.ib;
    ic = eval.ic;
    T_em = eval.T_em;
}

// 从统一配置结构体加载电机参数。
void MotorModel::apply_config(const SimulationConfig::Motor& cfg) {
    Rs = cfg.Rs;
    Ld = cfg.Ld;
    Lq = cfg.Lq;
    psi_f = cfg.psi_f;
    p = cfg.p;
}
