#include "../include/rectifier_model.h"
#include <algorithm>
#include <cmath>

// 理想三相二极管桥的瞬时直流电压近似：v_dc_inst = max(|v_ab|, |v_bc|, |v_ca|)
// 然后用一阶低通滤波得到 v_dc。
void RectifierModel::update_from_grid(double v_a_in, double v_b_in, double v_c_in) {
    v_a = v_a_in; v_b = v_b_in; v_c = v_c_in;
    const double v_ab = v_a - v_b;
    const double v_bc = v_b - v_c;
    const double v_ca = v_c - v_a;
    v_dc_target = std::max({std::abs(v_ab), std::abs(v_bc), std::abs(v_ca)});

    // 一阶低通：v_dc += (v_dc_target - v_dc) * Ts / tau_dc
    if (tau_dc > 1e-12) {
        v_dc += (v_dc_target - v_dc) * (Ts / tau_dc);
    } else {
        v_dc = v_dc_target;
    }
}
