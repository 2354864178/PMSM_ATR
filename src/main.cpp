#include <algorithm> 
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "components/motor_model.h"             // 电机模型
#include "components/turbine_model.h"           // 涡轮模型
#include "components/shaft_model.h"             // 轴系模型
#include "components/pump_model.h"              // 离心泵模型（作为负载）
#include "components/inverter_model.h"          // 三相逆变器功率级
#include "controllers/current_controller.h"     // dq 电流环控制器
#include "controllers/speed_controller.h"       // 机械转速外环控制器
#include "controllers/svpwm_controller.h"       // SVPWM 控制器
#include "config/sim_config.h"                  // 仿真配置
#include "simulation_logger.h"                  // 日志输出模块
#include "solver/system_solver.h"               // 全系统RK4求解器

namespace {
SystemRK4Solver::ComponentSwitches build_component_switches(const SimulationConfig::Components& cfg_components) {
    SystemRK4Solver::ComponentSwitches switches;
    switches.motor_enabled = cfg_components.motor_enabled;
    switches.turbine_enabled = cfg_components.turbine_enabled;
    switches.pump_enabled = cfg_components.pump_enabled;
    switches.inverter_enabled = cfg_components.inverter_enabled;
    switches.svpwm_enabled = cfg_components.svpwm_enabled;

    if (!switches.motor_enabled) {
        switches.inverter_enabled = false;
        switches.svpwm_enabled = false;
    }
    return switches;
}

void apply_all_configs(const SimulationConfig& cfg,
                       MotorModel& motor,
                       TurbineModel& turb,
                       PumpModel& pump,
                       SpeedController& speed_loop,
                       DQCurrentController& current_loop,
                       SVPWMController& svpwm,
                       InverterModel& inverter,
                       ShaftModel& shaft,
                       SystemRK4Solver& solver,
                       double control_dt) {
    motor.apply_config(cfg.motor);
    turb.apply_config(cfg.turbine);
    pump.apply_config(cfg.pump);
    speed_loop.apply_config(cfg.speed_loop);
    current_loop.apply_config(cfg.current_loop);
    svpwm.apply_config(cfg.svpwm);
    inverter.apply_config(cfg.inverter, cfg.battery);
    shaft.apply_config(cfg.shaft);
    speed_loop.set_Ts(control_dt);
    current_loop.set_Ts(control_dt);
    solver.set_fixed_speed_mode(cfg.runtime.fixed_speed_mode, cfg.runtime.omega_mech_fixed);
}

// 当前版本输入为常值，循环前注入一次
void inject_constant_external_commands(const SimulationConfig::Inputs& inputs, SystemRK4Solver& solver) {
    TurbineModel::Command turb_cmd;
    turb_cmd.p_in = inputs.turb_p_in;
    turb_cmd.p_out = inputs.turb_p_out;
    turb_cmd.T_in = inputs.turb_T_in;
    turb_cmd.m_dot = inputs.turb_m_dot;
    solver.set_turbine_command(turb_cmd);

    PumpModel::Command pump_cmd;
    pump_cmd.p_suction = inputs.pump_p_suction;
    pump_cmd.p_downstream = inputs.pump_p_downstream;
    solver.set_pump_command(pump_cmd);

    InverterModel::Command inv_cmd;
    inv_cmd.i_batt_cmd = inputs.i_batt_cmd;
    solver.set_inverter_command(inv_cmd);
}

// 根据当前时间和控制周期更新控制输出（电压参考 ud_ref、uq_ref 和 speed_loop 计算的 iq_ref_cmd）。
void update_control_outputs(bool current_loop_enabled,
                            const SimulationConfig& cfg,
                            double t,
                            double control_dt,
                            double& next_control_t,
                            double& iq_ref_cmd,
                            double& ud_ref,
                            double& uq_ref,
                            const ShaftModel& shaft,
                            SpeedController& speed_loop,
                            DQCurrentController& current_loop,
                            const DQCurrentController::SensorInterface& current_sensor) {
    if (current_loop_enabled && t + 1e-12 >= next_control_t) {
        const double id_ref_cmd = cfg.current_loop.id_ref;
        if (cfg.speed_loop.enabled) {
            const double omega_ref = cfg.speed_loop.omega_ref_1;
            const SpeedController::Output speed_out = speed_loop.evaluate(omega_ref, shaft.omega_mech);
            iq_ref_cmd = speed_out.iq_ref;
        } else {
            iq_ref_cmd = cfg.current_loop.iq_ref;
        }

        const DQCurrentController::Output i_out = current_loop.evaluate_with_sensor(
            id_ref_cmd,
            iq_ref_cmd,
            current_sensor);
        ud_ref = i_out.ud_ref;
        uq_ref = i_out.uq_ref;
        next_control_t += control_dt;
    } else if (!current_loop_enabled) {
        ud_ref = 0.0;
        uq_ref = 0.0;
    }
}
}  // namespace

class CurrentLoopPlantSensor final : public DQCurrentController::SensorInterface {
public:
    CurrentLoopPlantSensor(const MotorModel& motor_in,
                           const ShaftModel& shaft_in,
                           const InverterModel& inverter_in,
                           const SVPWMController& svpwm_in)
        : motor(motor_in), shaft(shaft_in), inverter(inverter_in), svpwm(svpwm_in) {}

    DQCurrentController::SensorSample sample() const override {
        DQCurrentController::SensorSample s;
        s.id_meas = motor.id;
        s.iq_meas = motor.iq;
        s.omega_e = static_cast<double>(motor.p) * shaft.omega_mech;
        s.v_dc = inverter.v_dc;
        s.motor = &motor;
        s.linear_limit = svpwm.linear_limit;
        return s;
    }

private:
    const MotorModel& motor;
    const ShaftModel& shaft;
    const InverterModel& inverter;
    const SVPWMController& svpwm;
};

// 程序入口（开环示例）：
// 1) 创建默认配置
// 2) 构建各部件与系统求解器
// 3) 循环注入输入并调用 solver.step
// 4) 记录日志供 Notebook 绘图
int main() {
    // 使用代码内默认配置（sim_config.h）。
    SimulationConfig cfg;
    const SystemRK4Solver::ComponentSwitches comp_switches = build_component_switches(cfg.components);
    const bool current_loop_enabled = cfg.components.current_loop_enabled && comp_switches.motor_enabled;

    const double model_dt = cfg.runtime.model_dt;
    const double control_dt = cfg.runtime.control_dt;

    // 按统一步长构造各组件。
    MotorModel motor(model_dt);
    TurbineModel turb(model_dt);
    PumpModel pump(model_dt);
    SpeedController speed_loop(control_dt);
    DQCurrentController current_loop(control_dt);
    SVPWMController svpwm(model_dt);
    InverterModel inverter(model_dt);
    ShaftModel shaft(model_dt);
    SystemRK4Solver solver(motor, turb, pump, svpwm, inverter, shaft, model_dt);
    solver.set_component_switches(comp_switches);

    // 将配置参数下发到各组件。
    apply_all_configs(cfg, motor, turb, pump, speed_loop, current_loop, svpwm, inverter, shaft, solver, control_dt);

    // 当前版本输入为常值，循环前注入一次。
    inject_constant_external_commands(cfg.inputs, solver);
    // 初始状态从组件同步到求解器内部状态向量。
    solver.sync_from_models();
    CurrentLoopPlantSensor current_sensor(motor, shaft, inverter, svpwm);

    SimulationLogger logger(cfg.runtime.log_path);
    if (!logger.is_open()) {
        std::cerr << "Failed to open log file: " << cfg.runtime.log_path << std::endl;
        return 1;
    }
    logger.write_header();
    std::cout << std::fixed << std::setprecision(6);

    const double time = cfg.runtime.total_time;            // 总模拟时间 s
    const int steps = static_cast<int>(time / model_dt);        // 总步数
    const int log_stride = std::max(1, cfg.runtime.log_every_n_steps);
    double next_control_t = 0.0;    // 下一个控制更新的时间点 s
    double ud_ref = 0.0;
    double uq_ref = 0.0;
    double iq_ref_cmd = cfg.current_loop.iq_ref;
    for (int k = 0; k < steps; ++k) {
        const double t = k * model_dt;
        update_control_outputs(current_loop_enabled,
                               cfg,
                               t,
                               control_dt,
                               next_control_t,
                               iq_ref_cmd,
                               ud_ref,
                               uq_ref,
                               shaft,
                               speed_loop,
                               current_loop,
                               current_sensor);

        // 全系统统一 RK4 步进（一步内包含 4 个子步评估）。
        solver.step(ud_ref, uq_ref);

        if (k % log_stride == 0) {
            logger.log_snapshot(t, ud_ref, uq_ref, motor, turb, pump, shaft, inverter);
        }
    }

    std::cout << "Using in-code parameters" << std::endl;
    std::cout << "Log written to " << cfg.runtime.log_path << std::endl;
    return 0;
}
