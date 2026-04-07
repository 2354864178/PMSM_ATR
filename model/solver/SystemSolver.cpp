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
    state.p_downstream_turb = turb.p_downstream;
    state.p_discharge = pump.p_discharge;
    state.v_dc = inverter.v_dc;
    apply_fixed_speed_constraint(state);
}

void SystemRK4Solver::set_inverter_command(const InverterModel::Command& cmd) {
    inverter_cmd = cmd;
}

void SystemRK4Solver::set_turbine_command(const TurbineModel::Command& cmd) {
    turbine_cmd = cmd;
}

void SystemRK4Solver::set_pump_command(const PumpModel::Command& cmd) {
    pump_cmd = cmd;
}

void SystemRK4Solver::set_component_switches(const ComponentSwitches& switches) {
    component_switches = switches;
}

void SystemRK4Solver::set_fixed_speed_mode(bool enabled, double omega_mech_fixed_in) {
    fixed_speed_mode = enabled;
    omega_mech_fixed = omega_mech_fixed_in;
    apply_fixed_speed_constraint(state);
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
    out.deriv = make_derivative(evals);
    out.alg = make_algebraic(evals);

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
    const double theta_e = electrical_theta(current);

    evals.inverter_svpwm = eval_svpwm_stage(current, ud_ref, uq_ref, theta_e);
    evals.inverter_ac = eval_inverter_ac_stage(current, evals.inverter_svpwm);
    evals.motor = eval_motor_stage(current, evals.inverter_ac, theta_e);
    evals.turbine = eval_turbine_stage(current);
    evals.pump = eval_pump_stage(current);
    evals.inverter_dc = eval_inverter_dc_stage(current, evals.inverter_ac, evals.motor);
    evals.shaft = eval_shaft_stage(current, evals.motor, evals.turbine, evals.pump);
    return evals;
}

SVPWMController::Output SystemRK4Solver::eval_svpwm_stage(const SystemState& current,
                                                           double ud_ref,
                                                           double uq_ref,
                                                           double theta_e) const {
    if (component_switches.svpwm_enabled) {
        return svpwm.evaluate(current.v_dc, ud_ref, uq_ref, theta_e);
    }
    return SVPWMController::Output{};
}

InverterModel::AcEval SystemRK4Solver::eval_inverter_ac_stage(
    const SystemState& current,
    const SVPWMController::Output& svpwm_out) const {
    if (component_switches.inverter_enabled) {
        return inverter.evaluate_ac_side(current.v_dc, svpwm_out);
    }
    return InverterModel::AcEval{};
}

// 电机评估需要输入当前电压（由 SVPWM 输出）和电气角（由机械角映射）。如果电机被禁用，则直接返回一个“空”评估，仅包含电气角信息，保持后续计算接口一致。
MotorModel::ElectricalEval SystemRK4Solver::eval_motor_stage(const SystemState& current,
                                                              const InverterModel::AcEval& inv_ac,
                                                              double theta_e) const {
    if (component_switches.motor_enabled) {
        return motor.evaluate_electrical(current.id,
                                         current.iq,
                                         current.theta_mech,
                                         current.omega_mech,
                                         inv_ac.ua,
                                         inv_ac.ub,
                                         inv_ac.uc);
    }

    MotorModel::ElectricalEval out;
    out.theta_e = theta_e;
    out.omega_e = motor.p * current.omega_mech;
    return out;
}

TurbineModel::GasEval SystemRK4Solver::eval_turbine_stage(const SystemState& current) const {
    if (component_switches.turbine_enabled) {
        return turb.evaluate_gas(current.p_plenum,
                                 current.p_downstream_turb,
                                 current.omega_mech,
                                 turbine_cmd);
    }
    return TurbineModel::GasEval{};
}

PumpModel::HydraulicEval SystemRK4Solver::eval_pump_stage(const SystemState& current) const {
    if (component_switches.pump_enabled) {
        return pump.evaluate_hydraulics(current.p_discharge,
                                        current.omega_mech,
                                        pump_cmd);
    }
    return PumpModel::HydraulicEval{};
}

InverterModel::DcEval SystemRK4Solver::eval_inverter_dc_stage(
    const SystemState& current,
    const InverterModel::AcEval& inv_ac,
    const MotorModel::ElectricalEval& motor_eval) const {
    const double i_inv_dc = (component_switches.inverter_enabled && component_switches.motor_enabled)
                                ? inverter.estimate_inverter_dc_current(inv_ac,
                                                                        motor_eval.ia,
                                                                        motor_eval.ib,
                                                                        motor_eval.ic,
                                                                        current.v_dc)
                                : 0.0;
    if (component_switches.inverter_enabled) {
        return inverter.evaluate_dc_bus(current.v_dc, i_inv_dc, inverter_cmd);
    }

    InverterModel::DcEval out;
    out.v_dc_clamped = std::clamp(current.v_dc, inverter.vdc_min, inverter.vdc_max);
    return out;
}

ShaftModel::MechanicalEval SystemRK4Solver::eval_shaft_stage(
    const SystemState& current,
    const MotorModel::ElectricalEval& motor_eval,
    const TurbineModel::GasEval& turb_eval,
    const PumpModel::HydraulicEval& pump_eval) const {
    return shaft.evaluate_mechanics(current.omega_mech,
                                    motor_eval.T_em,
                                    turb_eval.T_turb,
                                    pump_eval.T_pump);
}

SystemAlgebraic SystemRK4Solver::make_algebraic(const ComponentEvals& evals) const {
    SystemAlgebraic alg;
    alg.inverter_svpwm = evals.inverter_svpwm;
    alg.inverter_ac = evals.inverter_ac;
    alg.motor = evals.motor;
    alg.turbine = evals.turbine;
    alg.pump = evals.pump;
    alg.inverter_dc = evals.inverter_dc;
    return alg;
}

SystemDeriv SystemRK4Solver::make_derivative(const ComponentEvals& evals) const {
    SystemDeriv deriv;
    deriv.did = evals.motor.did;
    deriv.diq = evals.motor.diq;
    deriv.dp_plenum = evals.turbine.dp_plenum;
    deriv.dp_downstream_turb = evals.turbine.dp_downstream;
    deriv.dp_discharge = evals.pump.dp_discharge;
    deriv.dtheta_mech = fixed_speed_mode ? omega_mech_fixed : evals.shaft.dtheta_mech;
    deriv.domega_mech = fixed_speed_mode ? 0.0 : evals.shaft.domega_mech;
    deriv.dv_dc = evals.inverter_dc.dv_dc;
    return deriv;
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
    turb.apply_gas_state(state.omega_mech, state.p_plenum, state.p_downstream_turb, alg.turbine);
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
    apply_fixed_speed_constraint(state);

    // 物理边界保护：压力不低于各模型允许下限。
    clamp_state_bounds(state);

    // 用合并后的新状态再评估一次，得到与“新状态”一致的代数量。
    const EvalResult final_eval = evaluate(state, ud_ref, uq_ref);
    sync_to_models(final_eval.alg);
}

void SystemRK4Solver::apply_fixed_speed_constraint(SystemState& s) const {
    if (fixed_speed_mode) {
        s.omega_mech = omega_mech_fixed;
    }
}

void SystemRK4Solver::clamp_state_bounds(SystemState& s) const {
    s.p_plenum = std::max(s.p_plenum, turb.p_floor);
    s.p_downstream_turb = std::max(s.p_downstream_turb, turb.p_floor);
    s.p_discharge = std::max(s.p_discharge, pump.p_floor);
    s.v_dc = std::clamp(s.v_dc, inverter.vdc_min, inverter.vdc_max);
}

double SystemRK4Solver::electrical_theta(const SystemState& s) const {
    return motor.p * s.theta_mech;
}
