#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "motor_model.h"    // 电机模型
#include "turbine_model.h"  // 涡轮模型
#include "shaft_model.h"    // 轴系模型
#include "pump_model.h"     // 离心泵模型（作为负载）

// 开环：三相正弦 + 欧拉积分，记录 CSV 供 Notebook 绘图
int main(int argc, char* argv[]) {
    double custom_Ts = 0.000001; // 默认采样时间

    MotorModel motor(custom_Ts);
    TurbineModel turb(custom_Ts);
    PumpModel pump(custom_Ts);
    ShaftModel shaft(custom_Ts);

    // 电机参数
    motor.Rs = 0.0051;                  // 设置电机定子电阻
    motor.Ld = 0.0000346;               // 设置电机 d 轴电感
    motor.Lq = 0.0000346;               // 设置电机 q 轴电感
    motor.psi_f = 0.026;                // 设置永磁体磁链
    motor.p = 2;                        // 设置极对数

    // 涡轮参数
    turb.gamma = 1.4;                   // 设置比热比
    turb.gas_R = 287.0;                 // 设置比气体常数
    turb.eta_turb = 0.9;                // 设置涡轮等熵效率

    // 泵参数
    pump.rho = 1;                       // 设置流体密度
    pump.Q = 0;                         // 设置流体流量
    pump.eta_p = 1;                     // 设置泵效率

    // 轴系参数
    shaft.b_motor = 0.0009;             // 设置轴系电机侧摩擦系数
    shaft.b_turb = 0;                   // 设置轴系涡轮侧摩擦系数
    shaft.b_pump = 0;                   // 设置轴系泵侧摩擦系数
    shaft.J_motor = 0.000772;           // 设置轴系电机侧转动惯量
    shaft.J_turb = 0.1000;              // 设置轴系涡轮侧转动惯量
    shaft.J_pump = 0.0000;              // 设置轴系泵侧转动惯量

    // 同时写文件并在 stdout 回显每行，便于运行时检查
    std::ofstream ofs("build/log.csv");
    ofs << "t,ua,ub,uc,omega_mech,theta_mech,theta_e,ia,ib,ic,id,iq,T_em,T_turb,T_pump" << '\n';
    ofs << std::fixed << std::setprecision(6);
    std::cout << std::fixed << std::setprecision(6);

    const double time = 10;            // 总模拟时间 s
    const int steps = static_cast<int>(time / custom_Ts);        // 总步数
    for (int k = 0; k < steps; ++k) {
        double t = k * custom_Ts;

        // 三相正弦电压
        motor.ua = 0.0;
        motor.ub = 0.0;
        motor.uc = 0.5;
        motor.update_electrical(shaft.omega_mech);      // 更新电机电气状态

        // 涡轮入口条件
        turb.p_in = 100000.0;
        turb.p_out = 100000.0;
        turb.T_in = 600.0;
        turb.m_dot = 1.0;
        turb.update_from_gas(shaft.omega_mech);         // 更新涡轮状态

        // 泵流量
        pump.Q = 0;
        pump.update_from_hydraulics(shaft.omega_mech);  // 更新泵负载（基于轴速与压差）

        // 将泵作为负载扭矩传入轴系，不修改轴系文件
        shaft.update_mechanics(motor.T_em, turb.T_turb, pump.T_pump);

        if (k % 10 == 0) {
            // 采样每 10 步输出一行
            std::ostringstream line;
            line << t << ','
                 << motor.ua << ',' << motor.ub << ',' << motor.uc << ','
                 << shaft.omega_mech << ',' << shaft.theta_mech << ','
                 << motor.theta_e << ','
                 << motor.ia << ',' << motor.ib << ',' << motor.ic << ','
                 << motor.id << ',' << motor.iq << ','
                 << motor.T_em << ',' << turb.T_turb << ',' << pump.T_pump;
            const std::string row = line.str();
            ofs << row << '\n';
            // std::cout << row << '\n';   // 开启终端输出
        }
    }

    std::cout << "Log written to build/log.csv" << std::endl;
    return 0;
}
