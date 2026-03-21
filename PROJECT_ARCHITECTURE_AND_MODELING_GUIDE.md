# PMSM_ATR 项目架构、建模原理与开发操作指南（详细版）

本文档面向开发者，详细说明：
- 项目整体架构
- 建模的物理原理与数值求解思路
- 部件模型接口设计
- 系统封装与调用链
- 使用方式
- 新增部件接入流程（可直接照做）

> 说明：文档中“代码讲解”部分均直接粘贴当前项目代码原文。

---

## 1. 项目总体架构

### 1.1 目录职责划分

- `include/components/`：部件模型接口（电机、涡轮、泵、轴系）
- `model/components/`：部件模型实现
- `include/solver/`：系统求解器接口、RK4工具接口
- `model/solver/`：系统求解器实现、RK4工具实现
- `include/config/`：配置结构定义
- `model/config/`：配置解析实现
- `config/`：运行配置文件
- `src/main.cpp`：程序入口、装配部件、加载配置、执行仿真、输出日志

### 1.2 架构设计原则

1. **部件级建模**：每个部件维护自己的参数、状态和方程。
2. **系统级统一求解**：通过 `SystemRK4Solver` 统一做 RK4 子步积分。
3. **无副作用计算接口**：`evaluate_*` 只计算，不改对象状态。
4. **集中回写接口**：`apply_*` 专门写状态和输出。
5. **配置驱动**：参数初始化由配置文件下沉，`main` 不再硬编码具体参数值。

---

## 2. 程序入口与系统装配

下面是当前入口程序代码原文（`src/main.cpp`）：

```cpp
#include <cmath>
#include <cstdlib>
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

// 开环：三相正弦 + 欧拉积分，记录 CSV 供 Notebook 绘图
int main(int argc, char* argv[]) {
    const std::string config_path = (argc > 1) ? argv[1] : "config/simulation.cfg";
    SimulationConfig cfg;
    std::string config_error;
    if (!load_simulation_config(config_path, cfg, config_error)) {
        std::cerr << "Failed to load config: " << config_error << std::endl;
        return 1;
    }

    const double custom_Ts = cfg.runtime.Ts;

    MotorModel motor(custom_Ts);
    TurbineModel turb(custom_Ts);
    PumpModel pump(custom_Ts);
    ShaftModel shaft(custom_Ts);
    SystemRK4Solver solver(motor, turb, pump, shaft, custom_Ts);

    motor.apply_config(cfg.motor);
    turb.apply_config(cfg.turbine);
    pump.apply_config(cfg.pump);
    shaft.apply_config(cfg.shaft);

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

        // 三相正弦电压
        motor.ua = cfg.inputs.ua;
        motor.ub = cfg.inputs.ub;
        motor.uc = cfg.inputs.uc;

        // 涡轮入口条件
        turb.p_in = cfg.inputs.turb_p_in;
        turb.p_out = cfg.inputs.turb_p_out;
        turb.T_in = cfg.inputs.turb_T_in;
        turb.m_dot = cfg.inputs.turb_m_dot;
        solver.step(motor.ua, motor.ub, motor.uc);      // 全系统统一RK4步进

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

    std::cout << "Config loaded: " << config_path << std::endl;
    std::cout << "Log written to " << cfg.runtime.log_path << std::endl;
    return 0;
}
```

### 2.1 入口层设计解读

- 配置加载失败直接退出，防止“半配置运行”。
- `apply_config` 统一部件参数初始化，主函数清晰。
- 每步只做输入注入 + `solver.step(...)`，职责简明。

---

## 3. 建模原理（物理方程）

### 3.1 电机（PMSM dq）

核心微分方程：

$$
\dot i_d = \frac{u_d - R_s i_d + \omega_e L_q i_q}{L_d},\quad
\dot i_q = \frac{u_q - R_s i_q - \omega_e L_d i_d - \omega_e\psi_f}{L_q}
$$

电磁转矩：

$$
T_{em} = 1.5\,(p/2)\left(\psi_f i_q + (L_d-L_q)i_d i_q\right)
$$

在代码中体现为 `MotorModel::evaluate_electrical(...)`。

### 3.2 涡轮（容积+喷嘴流）

腔体压力动态：

$$
\dot p_{plenum}=\frac{R T}{V}(\dot m_{in}-\dot m_{turb})
$$

流量采用入口压差与喷嘴可压缩流动（壅塞/非壅塞）模型，最终得到涡轮扭矩 `T_turb`。

在代码中体现为 `TurbineModel::evaluate_gas(...)`。

### 3.3 泵（容积+下游流阻）

核心关系：

$$
Q_{pump}=\max(K_q\omega-K_{slip}\Delta p,0),\quad
Q_{out}=\max((p_{discharge}-p_{downstream})/R_{out},0)
$$

$$
\dot p_{discharge}=\frac{\beta_{eff}}{V_{out}}(Q_{pump}-Q_{out})
$$

由压升与效率得到负载扭矩 `T_pump`。

在代码中体现为 `PumpModel::evaluate_hydraulics(...)`。

### 3.4 轴系（刚性同轴）

$$
\dot\omega=\frac{T_{em}+T_{turb}-T_{pump}-T_{fric}}{J_{total}},\quad
\dot\theta=\omega
$$

在代码中体现为 `ShaftModel::evaluate_mechanics(...)`。

---

## 4. 部件模型接口（代码原文）

### 4.1 电机接口（`include/components/motor_model.h`）

```cpp
#pragma once
#include <cmath>
#include "config/sim_config.h"

// 电机模型声明
class MotorModel {
public:
    struct ElectricalEval {
        double did = 0.0;
        double diq = 0.0;
        double theta_e = 0.0;
        double omega_e = 0.0;
        double ud = 0.0;
        double uq = 0.0;
        double i_alpha = 0.0;
        double i_beta = 0.0;
        double ia = 0.0;
        double ib = 0.0;
        double ic = 0.0;
        double T_em = 0.0;
    };

    MotorModel() = default;
    explicit MotorModel(double Ts_in);
    void set_Ts(double Ts_in);

    double Rs   = 0.0051;
    double Ld   = 0.0000346;
    double Lq   = 0.0000346;
    double psi_f = 0.026;
    int p       = 2;
    double b_motor = 5e-05;
    double J_motor = 0.0001;

    double ua = 0.0, ub = 0.0, uc = 0.0;
    double ia = 0.0, ib = 0.0, ic = 0.0;
    double id = 0.0, iq = 0.0;
    double ud = 0.0, uq = 0.0;
    double i_alpha = 0.0, i_beta = 0.0;
    double u_alpha = 0.0, u_beta = 0.0;
    double omega_e = 0.0;
    double theta_e = 0.0;
    double T_em = 0.0;

    ElectricalEval evaluate_electrical(double id_in,
                                       double iq_in,
                                       double theta_mech,
                                       double omega_mech,
                                       double ua_in,
                                       double ub_in,
                                       double uc_in) const;
    void apply_electrical_state(double ua_in,
                                double ub_in,
                                double uc_in,
                                double id_in,
                                double iq_in,
                                const ElectricalEval& eval);
    void apply_config(const SimulationConfig::Motor& cfg);
    void update_electrical(double omega_mech);

private:
    double Ts = 0.0001;
};
```

### 4.2 涡轮接口（`include/components/turbine_model.h`）

```cpp
#pragma once
#include <algorithm>
#include <cmath>
#include "config/sim_config.h"

// 简化涡轮气动至轴扭矩映射
class TurbineModel {
public:
    struct GasEval {
        double dp_plenum = 0.0;
        double m_dot_in = 0.0;
        double m_dot_turb = 0.0;
        double T_turb = 0.0;
    };

    TurbineModel() = default;
    explicit TurbineModel(double Ts_in);
    void set_Ts(double Ts_in);

    double gamma = 1.4;
    double gas_R = 287.0;
    double eta_turb = 0.9;
    double omega_t = 0.0;
    double omega_floor = 50;
    double b_turb = 1e-04;
    double J_turb = 0.01;

    double p_in = 200000.0;
    double p_out = 100000.0;
    double T_in = 600.0;
    double m_dot = 0.2;

    double V_plenum = 5e-3;
    double C_in = 1.0;
    double Cd_nozzle = 0.85;
    double A_nozzle = 1.2e-4;
    double p_floor = 20000.0;

    double p_plenum = 150000.0;
    double m_dot_in = 0.0;
    double m_dot_turb = 0.0;

    double T_turb = 0.0;

    GasEval evaluate_gas(double p_plenum_in, double omega_shaft) const;
    void apply_gas_state(double omega_shaft, double p_plenum_in, const GasEval& eval);
    void apply_config(const SimulationConfig::Turbine& cfg);
    void update_from_gas(double omega_shaft);

private:
    double Ts = 0.0001;
};
```

### 4.3 泵接口（`include/components/pump_model.h`）

```cpp
#pragma once
#include <algorithm>
#include <cmath>
#include "config/sim_config.h"

// 简化离心泵液动到轴负载扭矩映射
class PumpModel {
public:
    struct HydraulicEval {
        double dp_discharge = 0.0;
        double Q_pump = 0.0;
        double Q_out = 0.0;
        double dp_pump = 0.0;
        double H = 0.0;
        double T_pump = 0.0;
    };

    PumpModel() = default;
    explicit PumpModel(double Ts_in);
    void set_Ts(double Ts_in);

    double rho = 1000.0;
    double Q = 0.0;
    double omega_p =0.0;
    double H = 0.0;
    double eta_p = 1;
    double T_pump = 0.0;
    double J_pump = 0;
    double b_pump = 0;
    double omega_floor = 50;

    double p_suction = 100000.0;
    double p_downstream = 100000.0;
    double p_discharge = 100000.0;
    double V_out = 3e-3;
    double beta_eff = 1.5e9;
    double R_out = 2e8;
    double K_q = 2e-5;
    double K_slip = 2e-12;
    double p_floor = 20000.0;

    double Q_pump = 0.0;
    double Q_out = 0.0;
    double dp_pump = 0.0;

    HydraulicEval evaluate_hydraulics(double p_discharge_in, double omega_shaft) const;
    void apply_hydraulic_state(double omega_shaft, double p_discharge_in, const HydraulicEval& eval);
    void apply_config(const SimulationConfig::Pump& cfg);
    void update_from_hydraulics(double omega_shaft);

private:
    double Ts = 0.0001;
};
```

### 4.4 轴系接口（`include/components/shaft_model.h`）

```cpp
#pragma once
#include "config/sim_config.h"

// 轴系模型声明
class ShaftModel {
public:
    struct MechanicalEval {
        double dtheta_mech = 0.0;
        double domega_mech = 0.0;
    };

    ShaftModel() = default;
    explicit ShaftModel(double Ts_in);
    void set_Ts(double Ts_in);

    double J_motor = 0.0001;
    double J_turb  = 0.01;
    double J_pump = 0;

    double b_motor = 5e-05;
    double b_turb  = 1e-04;
    double b_pump = 0;

    double omega_mech = 0.0;
    double theta_mech = 0.0;

    MechanicalEval evaluate_mechanics(double omega_mech_in, double T_em, double T_turb, double T_pump) const;
    void apply_mechanical_state(double theta_mech_in, double omega_mech_in);
    void apply_config(const SimulationConfig::Shaft& cfg);
    void update_mechanics(double T_em, double T_turb, double T_pump);

private:
    double Ts = 0.0001;
};
```

---

## 5. 系统封装（求解器）

### 5.1 求解器接口原文（`include/solver/system_solver.h`）

```cpp
#pragma once

#include "components/motor_model.h"
#include "components/pump_model.h"
#include "components/shaft_model.h"
#include "components/turbine_model.h"

struct SystemState {
    double id = 0.0;
    double iq = 0.0;
    double theta_mech = 0.0;
    double omega_mech = 0.0;
    double p_plenum = 150000.0;
    double p_discharge = 100000.0;
};

struct SystemAlgebraic {
    MotorModel::ElectricalEval motor;
    TurbineModel::GasEval turbine;
    PumpModel::HydraulicEval pump;
};

struct SystemDeriv {
    double did = 0.0;
    double diq = 0.0;
    double dtheta_mech = 0.0;
    double domega_mech = 0.0;
    double dp_plenum = 0.0;
    double dp_discharge = 0.0;
};

class SystemRK4Solver {
public:
    SystemRK4Solver(MotorModel& motor_in,
                    TurbineModel& turb_in,
                    PumpModel& pump_in,
                    ShaftModel& shaft_in,
                    double Ts_in);

    void sync_from_models();
    void step(double ua, double ub, double uc);

private:
    struct EvalResult {
        SystemDeriv deriv;
        SystemAlgebraic alg;
    };

    struct ComponentEvals {
        MotorModel::ElectricalEval motor;
        TurbineModel::GasEval turbine;
        PumpModel::HydraulicEval pump;
        ShaftModel::MechanicalEval shaft;
    };

    EvalResult evaluate(const SystemState& state, double ua, double ub, double uc) const;
    ComponentEvals evaluate_components(const SystemState& state, double ua, double ub, double uc) const;
    void fill_derivatives(const ComponentEvals& evals, SystemDeriv& deriv) const;
    void fill_algebraic(const ComponentEvals& evals, SystemAlgebraic& alg) const;
    void sync_to_models(const SystemAlgebraic& alg, double ua, double ub, double uc);

    MotorModel& motor;
    TurbineModel& turb;
    PumpModel& pump;
    ShaftModel& shaft;
    double Ts = 1e-4;
    SystemState state;
};
```

### 5.2 求解器实现原文（`model/solver/SystemSolver.cpp`）

```cpp
#include "solver/system_solver.h"
#include "solver/rk4_utils.h"

#include <algorithm>

SystemRK4Solver::SystemRK4Solver(MotorModel& motor_in,
                                 TurbineModel& turb_in,
                                 PumpModel& pump_in,
                                 ShaftModel& shaft_in,
                                 double Ts_in)
    : motor(motor_in), turb(turb_in), pump(pump_in), shaft(shaft_in), Ts(Ts_in) {
    sync_from_models();
}

void SystemRK4Solver::sync_from_models() {
    state.id = motor.id;
    state.iq = motor.iq;
    state.theta_mech = shaft.theta_mech;
    state.omega_mech = shaft.omega_mech;
    state.p_plenum = turb.p_plenum;
    state.p_discharge = pump.p_discharge;
}

SystemRK4Solver::EvalResult SystemRK4Solver::evaluate(const SystemState& current,
                                                      double ua,
                                                      double ub,
                                                      double uc) const {
    EvalResult out;
    const ComponentEvals evals = evaluate_components(current, ua, ub, uc);
    fill_derivatives(evals, out.deriv);
    fill_algebraic(evals, out.alg);

    return out;
}

SystemRK4Solver::ComponentEvals SystemRK4Solver::evaluate_components(const SystemState& current,
                                                                     double ua,
                                                                     double ub,
                                                                     double uc) const {
    ComponentEvals evals;
    evals.motor = motor.evaluate_electrical(current.id, current.iq, current.theta_mech, current.omega_mech, ua, ub, uc);
    evals.turbine = turb.evaluate_gas(current.p_plenum, current.omega_mech);
    evals.pump = pump.evaluate_hydraulics(current.p_discharge, current.omega_mech);
    evals.shaft = shaft.evaluate_mechanics(current.omega_mech,
                                           evals.motor.T_em,
                                           evals.turbine.T_turb,
                                           evals.pump.T_pump);
    return evals;
}

void SystemRK4Solver::fill_derivatives(const ComponentEvals& evals, SystemDeriv& deriv) const {
    deriv.did = evals.motor.did;
    deriv.diq = evals.motor.diq;
    deriv.dp_plenum = evals.turbine.dp_plenum;
    deriv.dp_discharge = evals.pump.dp_discharge;
    deriv.dtheta_mech = evals.shaft.dtheta_mech;
    deriv.domega_mech = evals.shaft.domega_mech;
}

void SystemRK4Solver::fill_algebraic(const ComponentEvals& evals, SystemAlgebraic& alg) const {
    alg.motor = evals.motor;
    alg.turbine = evals.turbine;
    alg.pump = evals.pump;
}

void SystemRK4Solver::sync_to_models(const SystemAlgebraic& alg, double ua, double ub, double uc) {
    motor.apply_electrical_state(ua, ub, uc, state.id, state.iq, alg.motor);
    shaft.apply_mechanical_state(state.theta_mech, state.omega_mech);
    turb.apply_gas_state(state.omega_mech, state.p_plenum, alg.turbine);
    pump.apply_hydraulic_state(state.omega_mech, state.p_discharge, alg.pump);
}

void SystemRK4Solver::step(double ua, double ub, double uc) {
    const EvalResult e1 = evaluate(state, ua, ub, uc);
    const SystemState s2 = rk4_utils::add_scaled(state, e1.deriv, 0.5 * Ts);

    const EvalResult e2 = evaluate(s2, ua, ub, uc);
    const SystemState s3 = rk4_utils::add_scaled(state, e2.deriv, 0.5 * Ts);

    const EvalResult e3 = evaluate(s3, ua, ub, uc);
    const SystemState s4 = rk4_utils::add_scaled(state, e3.deriv, Ts);

    const EvalResult e4 = evaluate(s4, ua, ub, uc);

    state = rk4_utils::combine_rk4(state, e1.deriv, e2.deriv, e3.deriv, e4.deriv, Ts);
    state.p_plenum = std::max(state.p_plenum, turb.p_floor);
    state.p_discharge = std::max(state.p_discharge, pump.p_floor);

    const EvalResult final_eval = evaluate(state, ua, ub, uc);
    sync_to_models(final_eval.alg, ua, ub, uc);
}
```

### 5.3 RK4工具原文

`include/solver/rk4_utils.h`：

```cpp
#pragma once

struct SystemState;
struct SystemDeriv;

namespace rk4_utils {
SystemState add_scaled(const SystemState& state, const SystemDeriv& deriv, double scale);
SystemState combine_rk4(const SystemState& state,
                        const SystemDeriv& k1,
                        const SystemDeriv& k2,
                        const SystemDeriv& k3,
                        const SystemDeriv& k4,
                        double Ts);
}
```

`model/solver/RK4Utils.cpp`：

```cpp
#include "solver/rk4_utils.h"
#include "solver/system_solver.h"

SystemState rk4_utils::add_scaled(const SystemState& base, const SystemDeriv& deriv, double scale) {
    SystemState out = base;
    out.id += deriv.did * scale;
    out.iq += deriv.diq * scale;
    out.theta_mech += deriv.dtheta_mech * scale;
    out.omega_mech += deriv.domega_mech * scale;
    out.p_plenum += deriv.dp_plenum * scale;
    out.p_discharge += deriv.dp_discharge * scale;
    return out;
}

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
    return out;
}
```

---

## 6. 配置系统（参数初始化下沉）

### 6.1 配置结构原文（`include/config/sim_config.h`）

```cpp
#pragma once

#include <string>

struct SimulationConfig {
    struct Runtime {
        double Ts = 1e-6;
        double total_time = 0.6;
        int log_every_n_steps = 10;
        std::string log_path = "build/log.xlsx";
    } runtime;

    struct Motor {
        double Rs = 0.0051;
        double Ld = 0.0000346;
        double Lq = 0.0000346;
        double psi_f = 0.026;
        int p = 2;
    } motor;

    struct Turbine {
        double gamma = 1.4;
        double gas_R = 287.0;
        double eta_turb = 0.9;
        double V_plenum = 5e-3;
        double A_nozzle = 1.2e-4;
        double Cd_nozzle = 0.85;
        double C_in = 1.0;
    } turbine;

    struct Pump {
        double rho = 1000.0;
        double eta_p = 0.8;
        double p_suction = 100000.0;
        double p_downstream = 120000.0;
        double p_discharge = 100000.0;
        double V_out = 3e-3;
        double beta_eff = 1.5e9;
        double R_out = 2e8;
        double K_q = 2e-5;
        double K_slip = 2e-12;
    } pump;

    struct Shaft {
        double b_motor = 0.0009;
        double b_turb = 0.0;
        double b_pump = 0.0;
        double J_motor = 0.000772;
        double J_turb = 0.0;
        double J_pump = 0.0;
    } shaft;

    struct Inputs {
        double ua = 0.0;
        double ub = 0.0;
        double uc = 0.5;
        double turb_p_in = 220000.0;
        double turb_p_out = 100000.0;
        double turb_T_in = 600.0;
        double turb_m_dot = 0.2;
    } inputs;
};

bool load_simulation_config(const std::string& file_path, SimulationConfig& config, std::string& error_message);
```

### 6.2 配置文件格式原文（`config/simulation.cfg`）

```properties
# Simulation runtime
runtime.Ts = 1e-6
runtime.total_time = 0.6
runtime.log_every_n_steps = 10
runtime.log_path = build/log.xlsx

# Motor
motor.Rs = 0.0051
motor.Ld = 0.0000346
motor.Lq = 0.0000346
motor.psi_f = 0.026
motor.p = 2

# Turbine model parameters
turbine.gamma = 1.4
turbine.gas_R = 287.0
turbine.eta_turb = 0.9
turbine.V_plenum = 5e-3
turbine.A_nozzle = 1.2e-4
turbine.Cd_nozzle = 0.85
turbine.C_in = 1.0

# Pump model parameters
pump.rho = 1000
pump.eta_p = 0.8
pump.p_suction = 100000.0
pump.p_downstream = 120000.0
pump.p_discharge = 100000.0
pump.V_out = 3e-3
pump.beta_eff = 1.5e9
pump.R_out = 2e8
pump.K_q = 2e-5
pump.K_slip = 2e-12

# Shaft model parameters
shaft.b_motor = 0.0009
shaft.b_turb = 0
shaft.b_pump = 0
shaft.J_motor = 0.000772
shaft.J_turb = 0
shaft.J_pump = 0

# Step inputs
inputs.ua = 0.0
inputs.ub = 0.0
inputs.uc = 0.5
inputs.turb_p_in = 220000.0
inputs.turb_p_out = 100000.0
inputs.turb_T_in = 600.0
inputs.turb_m_dot = 0.2
```

---

## 7. 使用说明

### 7.1 构建与运行

```bash
make
make run
```

或显式传配置文件：

```bash
./build/main config/simulation.cfg
```

### 7.2 输出

- 日志文件路径由 `runtime.log_path` 控制（默认 `build/log.xlsx`）
- 日志包含：电压、电流、转速、扭矩、涡轮/泵容积动态变量

### 7.3 构建脚本原文（`makefile`）

```makefile
## Simple build for PMSM demo

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Iinclude
LDFLAGS :=

SRCS := src/main.cpp model/components/MotorModel.cpp model/components/TurbineModel.cpp model/components/ShaftModel.cpp model/components/PumpModel.cpp model/solver/SystemSolver.cpp model/solver/RK4Utils.cpp
OBJS := $(SRCS:%.cpp=build/%.o)
BIN := build/main

.PHONY: all run clean

all: $(BIN)

run: $(BIN)
	./$(BIN)

$(BIN): $(OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build
```

---

## 8. 新增部件操作说明（详细流程）

以下流程可直接执行。

### 步骤 1：新增部件头/源文件

在：
- `include/components/new_component_model.h`
- `model/components/NewComponentModel.cpp`

建议接口模式：

```cpp
struct NewEval {
    double d_state = 0.0;
    double output_1 = 0.0;
};

class NewComponentModel {
public:
    NewEval evaluate_new(double state_in, double coupling_inputs...) const;
    void apply_new_state(double state_in, const NewEval& eval);
    void apply_config(const SimulationConfig::NewComponent& cfg);
};
```

### 步骤 2：扩展配置结构

在 `include/config/sim_config.h` 中新增：

```cpp
struct NewComponent {
    double param_a = ...;
    double param_b = ...;
} new_component;
```

并在 `model/config/SimConfig.cpp` 增加对应 key 解析。

### 步骤 3：扩展系统状态与导数

在 `include/solver/system_solver.h`：
- `SystemState` 增加新状态
- `SystemDeriv` 增加新导数
- `SystemAlgebraic` 增加该部件 `Eval`
- `ComponentEvals` 增加该部件 `Eval`

### 步骤 4：接入求解器耦合

在 `model/solver/SystemSolver.cpp`：
1. `evaluate_components(...)` 调用 `new_component.evaluate_new(...)`
2. `fill_derivatives(...)` 写入导数
3. `fill_algebraic(...)` 写入代数量
4. `sync_to_models(...)` 调用 `new_component.apply_new_state(...)`
5. `sync_from_models()` 初始化同步

### 步骤 5：扩展 RK4 工具

在 `model/solver/RK4Utils.cpp`：
- `add_scaled(...)` 添加新状态缩放项
- `combine_rk4(...)` 添加新状态组合项

### 步骤 6：主程序接入

在 `src/main.cpp`：
1. 实例化新部件
2. 传给求解器（若求解器构造函数变更）
3. `apply_config(cfg.new_component)`
4. 注入输入与日志输出扩展

### 步骤 7：构建系统接入

在 `makefile` 的 `SRCS` 增加：
- `model/components/NewComponentModel.cpp`

### 步骤 8：验证清单

1. `make clean && make` 通过
2. `./build/main` 运行成功
3. 日志中新增字段与数据列一一对应
4. 无 NaN/Inf，关键量量纲与数量级合理

---

## 9. 常见改造误区

1. **只改 `SystemState`，忘改 `RK4Utils`**
   - 现象：状态似乎“没生效”或错误积分。
2. **`evaluate_*` 中直接改成员变量**
   - 破坏 RK4 子步独立性，导致数值错误。
3. **日志头和日志行字段不同步**
   - 后处理脚本解析会错位。
4. **配置 key 新增但未在解析器注册**
   - 运行时报 unknown key。

---

## 10. 推荐后续优化路线

1. 轴系从单惯量升级到双/三惯量扭振模型
2. 为每个 `evaluate_*` 增加单元测试（边界工况、极端参数）
3. 增加多工况批处理脚本（批量读不同 cfg）
4. 统一日志模块（支持分组字段和版本化头）

---

本文档可作为当前项目的“架构主文档 + 开发手册”。后续如代码接口调整，建议优先同步本文件中的“原文代码片段”和“新增部件流程”。
