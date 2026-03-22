#include "solver/system_solver.h"
#include "solver/rk4_utils.h"

#include <algorithm>
#include <cmath>

// 构造时立即从模型对象抓取一次初始状态，保证 solver 与模型一致。
SystemRK4Solver::SystemRK4Solver(MotorModel& motor_in,
                                 TurbineModel& turb_in,
                                 PumpModel& pump_in,
                                 SVPWMController& svpwm_in,
                                 InverterModel& inverter_in,
                                 ShaftModel& shaft_in,
                                 double Ts_in)
    : motor(motor_in), turb(turb_in), pump(pump_in), svpwm(svpwm_in), inverter(inverter_in), shaft(shaft_in), Ts(Ts_in) {
    sync_from_models();
}

// 将各模型中的“当前状态”同步到系统状态向量。
// 通常在初始化后调用一次。
void SystemRK4Solver::sync_from_models() {
    state.id = motor.id;
    state.iq = motor.iq;
    state.theta_mech = shaft.theta_mech;
    state.omega_mech = shaft.omega_mech;
    state.p_plenum = turb.p_plenum;
    state.p_discharge = pump.p_discharge;
    state.v_dc = inverter.v_dc;
}

void SystemRK4Solver::set_inverter_command(const InverterModel::Command& cmd) {
    inverter_cmd = cmd;
}

// 对某个给定状态做一次完整耦合评估：
// 1) 各组件 evaluate
// 2) 组装导数
// 3) 组装代数量
SystemRK4Solver::EvalResult SystemRK4Solver::evaluate(const SystemState& current,
                                                      double ud_ref,
                                                      double uq_ref) const {
    EvalResult out;
    const ComponentEvals evals = evaluate_components(current, ud_ref, uq_ref);
    fill_derivatives(evals, out.deriv);
    fill_algebraic(evals, out.alg);

    return out;
}

// 耦合顺序说明：
// - 先电机，得到 T_em
// - 再涡轮、泵，得到 T_turb/T_pump
// - 最后轴系用三个扭矩计算机械导数
SystemRK4Solver::ComponentEvals SystemRK4Solver::evaluate_components(const SystemState& current,
                                                                     double ud_ref,
                                                                     double uq_ref) const {
    ComponentEvals evals;
    const double theta_e = motor.p * current.theta_mech;
    evals.inverter_svpwm = svpwm.evaluate(current.v_dc, ud_ref, uq_ref, theta_e);
    evals.inverter_ac = inverter.evaluate_ac_side(current.v_dc, evals.inverter_svpwm);
    evals.motor = motor.evaluate_electrical(current.id,
                                            current.iq,
                                            current.theta_mech,
                                            current.omega_mech,
                                            evals.inverter_ac.ua,
                                            evals.inverter_ac.ub,
                                            evals.inverter_ac.uc);
    evals.turbine = turb.evaluate_gas(current.p_plenum, current.omega_mech);
    evals.pump = pump.evaluate_hydraulics(current.p_discharge, current.omega_mech);
    const double i_inv_dc = inverter.estimate_inverter_dc_current(evals.inverter_ac,
                                                                  evals.motor.ia,
                                                                  evals.motor.ib,
                                                                  evals.motor.ic,
                                                                  current.v_dc);
    evals.inverter_dc = inverter.evaluate_dc_bus(current.v_dc, i_inv_dc, inverter_cmd);
    evals.shaft = shaft.evaluate_mechanics(current.omega_mech,
                                           evals.motor.T_em,
                                           evals.turbine.T_turb,
                                           evals.pump.T_pump);
    return evals;
}

// 导数字段专门用于 RK4 积分。
void SystemRK4Solver::fill_derivatives(const ComponentEvals& evals, SystemDeriv& deriv) const {
    deriv.did = evals.motor.did;
    deriv.diq = evals.motor.diq;
    deriv.dp_plenum = evals.turbine.dp_plenum;
    deriv.dp_discharge = evals.pump.dp_discharge;
    deriv.dtheta_mech = evals.shaft.dtheta_mech;
    deriv.domega_mech = evals.shaft.domega_mech;
    deriv.dv_dc = evals.inverter_dc.dv_dc;
}

// 代数量用于最终回写到模型对象（日志/可视化读取）。
void SystemRK4Solver::fill_algebraic(const ComponentEvals& evals, SystemAlgebraic& alg) const {
    alg.inverter_svpwm = evals.inverter_svpwm;
    alg.inverter_ac = evals.inverter_ac;
    alg.motor = evals.motor;
    alg.turbine = evals.turbine;
    alg.pump = evals.pump;
    alg.inverter_dc = evals.inverter_dc;
}

// 将求解器结果回写到各部件对象。
// 注意：真正的状态推进在 solver.state 里完成，部件对象主要用于对外展示。
void SystemRK4Solver::sync_to_models(const SystemAlgebraic& alg) {
    motor.apply_electrical_state(alg.inverter_ac.ua,
                                 alg.inverter_ac.ub,
                                 alg.inverter_ac.uc,
                                 state.id,
                                 state.iq,
                                 alg.motor);
    shaft.apply_mechanical_state(state.theta_mech, state.omega_mech);
    turb.apply_gas_state(state.omega_mech, state.p_plenum, alg.turbine);
    pump.apply_hydraulic_state(state.omega_mech, state.p_discharge, alg.pump);
    inverter.apply_state(state.v_dc, alg.inverter_svpwm, alg.inverter_ac, alg.inverter_dc);
}

// 单步 RK4 过程：e1/e2/e3/e4 四次评估 + 加权合并 + 边界保护 + 最终回写。
void SystemRK4Solver::step(double ud_ref, double uq_ref) {
    // k1
    const EvalResult e1 = evaluate(state, ud_ref, uq_ref);
    const SystemState s2 = rk4_utils::add_scaled(state, e1.deriv, 0.5 * Ts);

    // k2
    const EvalResult e2 = evaluate(s2, ud_ref, uq_ref);
    const SystemState s3 = rk4_utils::add_scaled(state, e2.deriv, 0.5 * Ts);

    // k3
    const EvalResult e3 = evaluate(s3, ud_ref, uq_ref);
    const SystemState s4 = rk4_utils::add_scaled(state, e3.deriv, Ts);

    // k4
    const EvalResult e4 = evaluate(s4, ud_ref, uq_ref);

    // RK4 合并到下一时刻状态
    state = rk4_utils::combine_rk4(state, e1.deriv, e2.deriv, e3.deriv, e4.deriv, Ts);

    // 物理边界保护：压力不低于各模型允许下限。
    state.p_plenum = std::max(state.p_plenum, turb.p_floor);
    state.p_discharge = std::max(state.p_discharge, pump.p_floor);
    state.v_dc = std::clamp(state.v_dc, inverter.vdc_min, inverter.vdc_max);

    // 用合并后的新状态再评估一次，得到与“新状态”一致的代数量。
    const EvalResult final_eval = evaluate(state, ud_ref, uq_ref);
    sync_to_models(final_eval.alg);
}
