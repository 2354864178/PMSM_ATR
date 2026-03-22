#include "solver/rk4_utils.h"
#include "solver/system_solver.h"

// 将导数按比例叠加到基础状态，得到某个 RK 子步的“试探状态”。
SystemState rk4_utils::add_scaled(const SystemState& base, const SystemDeriv& deriv, double scale) {
    SystemState out = base;
    // 电机电流状态
    out.id += deriv.did * scale;
    out.iq += deriv.diq * scale;
    // 机械状态
    out.theta_mech += deriv.dtheta_mech * scale;
    out.omega_mech += deriv.domega_mech * scale;
    // 流体压力状态
    out.p_plenum += deriv.dp_plenum * scale;
    out.p_discharge += deriv.dp_discharge * scale;
    // 直流母线状态
    out.v_dc += deriv.dv_dc * scale;
    return out;
}

// 使用标准 RK4 权重把四个子步导数合并到下一步状态。
SystemState rk4_utils::combine_rk4(const SystemState& base,
                                   const SystemDeriv& k1,
                                   const SystemDeriv& k2,
                                   const SystemDeriv& k3,
                                   const SystemDeriv& k4,
                                   double Ts) {
    SystemState out = base;
    out.id += (Ts / 6.0) * (k1.did + 2.0 * k2.did + 2.0 * k3.did + k4.did);
    out.iq += (Ts / 6.0) * (k1.diq + 2.0 * k2.diq + 2.0 * k3.diq + k4.diq);
    out.theta_mech += (Ts / 6.0) * (k1.dtheta_mech + 2.0 * k2.dtheta_mech + 2.0 * k3.dtheta_mech + k4.dtheta_mech);
    out.omega_mech += (Ts / 6.0) * (k1.domega_mech + 2.0 * k2.domega_mech + 2.0 * k3.domega_mech + k4.domega_mech);
    out.p_plenum += (Ts / 6.0) * (k1.dp_plenum + 2.0 * k2.dp_plenum + 2.0 * k3.dp_plenum + k4.dp_plenum);
    out.p_discharge += (Ts / 6.0) * (k1.dp_discharge + 2.0 * k2.dp_discharge + 2.0 * k3.dp_discharge + k4.dp_discharge);
    out.v_dc += (Ts / 6.0) * (k1.dv_dc + 2.0 * k2.dv_dc + 2.0 * k3.dv_dc + k4.dv_dc);
    return out;
}
