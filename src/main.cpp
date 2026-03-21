#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "components/motor_model.h"    // 电机模型
#include "components/turbine_model.h"  // 涡轮模型
#include "components/shaft_model.h"    // 轴系模型
#include "components/pump_model.h"     // 离心泵模型（作为负载）
#include "config/sim_config.h"         // 仿真配置
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
    ShaftModel shaft(custom_Ts);
    SystemRK4Solver solver(motor, turb, pump, shaft, custom_Ts);

    // 将配置参数下发到各组件。
    motor.apply_config(cfg.motor);
    turb.apply_config(cfg.turbine);
    pump.apply_config(cfg.pump);
    shaft.apply_config(cfg.shaft);

    // 初始状态从组件同步到求解器内部状态向量。
    solver.sync_from_models();

    // 同时写文件并在 stdout 回显每行，便于运行时检查
    std::ofstream ofs(cfg.runtime.log_path);
    ofs << "t,ua,ub,uc,omega_mech,theta_mech,theta_e,ia,ib,ic,id,iq,T_em,T_turb,T_pump,p_plenum,m_dot_in,m_dot_turb,p_discharge,Q_pump,Q_out" << '\n';
    ofs << std::fixed << std::setprecision(6);
    std::cout << std::fixed << std::setprecision(6);

    const double time = cfg.runtime.total_time;            // 总模拟时间 s
    const int steps = static_cast<int>(time / custom_Ts);        // 总步数
    const int log_stride = std::max(1, cfg.runtime.log_every_n_steps);
    for (int k = 0; k < steps; ++k) {
        double t = k * custom_Ts;

        // 输入注入：电机三相电压（当前为常值输入，可改成时变函数）。
        motor.ua = cfg.inputs.ua;
        motor.ub = cfg.inputs.ub;
        motor.uc = cfg.inputs.uc;

        // 输入注入：涡轮入口边界条件。
        turb.p_in = cfg.inputs.turb_p_in;
        turb.p_out = cfg.inputs.turb_p_out;
        turb.T_in = cfg.inputs.turb_T_in;
        turb.m_dot = cfg.inputs.turb_m_dot;
        // 全系统统一 RK4 步进（一步内包含 4 个子步评估）。
        solver.step(motor.ua, motor.ub, motor.uc);

        if (k % log_stride == 0) {
            // 采样每 10 步输出一行
            std::ostringstream line;
            line << t << ','
                 << motor.ua << ',' << motor.ub << ',' << motor.uc << ','
                 << shaft.omega_mech << ',' << shaft.theta_mech << ','
                 << motor.theta_e << ','
                 << motor.ia << ',' << motor.ib << ',' << motor.ic << ','
                 << motor.id << ',' << motor.iq << ','
                 << motor.T_em << ',' << turb.T_turb << ',' << pump.T_pump << ','
                 << turb.p_plenum << ',' << turb.m_dot_in << ',' << turb.m_dot_turb << ','
                 << pump.p_discharge << ',' << pump.Q_pump << ',' << pump.Q_out;
            const std::string row = line.str();
            ofs << row << '\n';
            // std::cout << row << '\n';   // 开启终端输出
        }
    }

    std::cout << "Using in-code parameters" << std::endl;
    std::cout << "Log written to " << cfg.runtime.log_path << std::endl;
    return 0;
}
