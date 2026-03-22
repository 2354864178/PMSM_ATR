#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

#include "components/motor_model.h"    // 电机模型
#include "components/turbine_model.h"  // 涡轮模型
#include "components/shaft_model.h"    // 轴系模型
#include "components/pump_model.h"     // 离心泵模型（作为负载）
#include "components/inverter_model.h" // 三相逆变器功率级
#include "controllers/svpwm_controller.h" // SVPWM 控制器
#include "config/sim_config.h"         // 仿真配置
#include "simulation_logger.h"         // 日志输出模块
#include "solver/system_solver.h"      // 全系统RK4求解器

// 程序入口（开环示例）：
// 1) 创建默认配置
// 2) 构建各部件与系统求解器
// 3) 循环注入输入并调用 solver.step
// 4) 记录日志供 Notebook 绘图
int main() {
    // 当前版本使用代码内默认配置（sim_config.h）。
    SimulationConfig cfg;

    const double custom_Ts = cfg.runtime.Ts;

    // 按统一步长构造各组件。
    MotorModel motor(custom_Ts);
    TurbineModel turb(custom_Ts);
    PumpModel pump(custom_Ts);
    SVPWMController svpwm(custom_Ts);
    InverterModel inverter(custom_Ts);
    ShaftModel shaft(custom_Ts);
    SystemRK4Solver solver(motor, turb, pump, svpwm, inverter, shaft, custom_Ts);

    // 将配置参数下发到各组件。
    motor.apply_config(cfg.motor);
    turb.apply_config(cfg.turbine);
    pump.apply_config(cfg.pump);
    svpwm.apply_config(cfg.svpwm);
    inverter.apply_config(cfg.inverter, cfg.battery);
    shaft.apply_config(cfg.shaft);

    // 初始状态从组件同步到求解器内部状态向量。
    solver.sync_from_models();

    SimulationLogger logger(cfg.runtime.log_path);
    if (!logger.is_open()) {
        std::cerr << "Failed to open log file: " << cfg.runtime.log_path << std::endl;
        return 1;
    }
    logger.write_header();
    std::cout << std::fixed << std::setprecision(6);

    const double time = cfg.runtime.total_time;            // 总模拟时间 s
    const int steps = static_cast<int>(time / custom_Ts);        // 总步数
    const int log_stride = std::max(1, cfg.runtime.log_every_n_steps);
    for (int k = 0; k < steps; ++k) {
        double t = k * custom_Ts;

        // 输入注入：SVPWM调制前的 dq 电压参考（开环示例）。
        const double ud_ref = cfg.inputs.ud;
        const double uq_ref = cfg.inputs.uq;

        // 输入注入：涡轮入口边界条件。
        turb.p_in = cfg.inputs.turb_p_in;
        turb.p_out = cfg.inputs.turb_p_out;
        turb.T_in = cfg.inputs.turb_T_in;
        turb.m_dot = cfg.inputs.turb_m_dot;
        InverterModel::Command inv_cmd;
        inv_cmd.i_batt_cmd = cfg.inputs.i_batt_cmd;
        solver.set_inverter_command(inv_cmd);
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
