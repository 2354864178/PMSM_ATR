#pragma once
#include <cmath>

// 三相整流器（主动整流，预留 SVPWM 调制输入）
// 当前实现：基于三相线电压的理想二极管桥瞬时直流电压，
// 通过一阶低通近似滤波获得直流母线电压 v_dc。
class RectifierModel {
public:
    RectifierModel() = default;
    explicit RectifierModel(double Ts_in) : Ts(Ts_in) {}
    void set_Ts(double Ts_in) { Ts = Ts_in; }

    // 电网三相瞬时电压（输入）
    double v_a = 0.0, v_b = 0.0, v_c = 0.0;

    // 预留：SVPWM 调制系数（-1..1），当前未用于计算
    double m_a = 0.0, m_b = 0.0, m_c = 0.0;

    // 直流侧
    double v_dc = 0.0;          // 直流母线电压（滤波后）
    double v_dc_target = 0.0;   // 未滤波的瞬时目标电压
    double tau_dc = 1e-3;       // 直流侧等效滤波时间常数（s）

    // 更新：根据输入三相电压计算直流电压
    void update_from_grid(double v_a_in, double v_b_in, double v_c_in);

private:
    double Ts = 1e-4;           // 采样时间（s）
};
