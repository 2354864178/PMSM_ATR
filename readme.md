# 动力系统公式梳理

## make 命令示例
- 构建：
```bash
make
```
- 运行示例（使用代码内参数）：
```bash
make run
```
- 清理：
```bash
make clean
```
构建产物位于 build/，主程序为 build/main。

当前主程序输出数据文件为 build/log.xlsx，绘图文件为 notebooks/plot.ipynb

## 代码结构（重构后）
- 部件模型头文件：`include/components/`
- 控制器头文件：`include/controllers/`
- 求解器头文件：`include/solver/`
- 部件模型实现：`model/components/`
- 控制器实现：`model/controllers/`
- 求解器实现：`model/solver/`
- 主程序入口：`src/main.cpp`

说明：
- 控制器（SVPWM）负责 dq->abc 调制计算。
- 逆变器功率级负责 DC 母线与功率流（前端供电/电池接口/电容动态）。
- 各部件（电机/涡轮/泵/轴系）负责各自物理方程（`evaluate_*`）与状态回写（`apply_*`）。
- 全系统耦合与积分在求解器层统一完成。
- 参数初始化在 `src/main.cpp` 中统一设置，再通过 `apply_config` 下发到部件。

## 坐标变换
- Clarke：$\alpha = \tfrac{2}{3}(a - \tfrac{1}{2}b - \tfrac{1}{2}c)$，$\beta = \tfrac{2}{3}(\tfrac{\sqrt{3}}{2}b - \tfrac{\sqrt{3}}{2}c)$。
- Park：$d = \alpha\cos\theta_e + \beta\sin\theta_e$，$q = -\alpha\sin\theta_e + \beta\cos\theta_e$。
- 逆 Park：$\alpha = d\cos\theta_e - q\sin\theta_e$，$\beta = d\sin\theta_e + q\cos\theta_e$。

## PMSM 电气侧（dq 模型）
- 电角速度：$\omega_e = p\,\omega_{mech}$。
- 电流微分：
  - $\dot i_d = \dfrac{u_d - R_s i_d + \omega_e L_q i_q}{L_d}$
  - $\dot i_q = \dfrac{u_q - R_s i_q - \omega_e L_d i_d - \omega_e\psi_f}{L_q}$
- 电磁转矩：$T_{em} = 1.5\,(p/2)\,(\psi_f i_q + (L_d - L_q) i_d i_q)$。
- 定子铜耗：$P_{cu} = 1.5\,R_s\,(i_d^2 + i_q^2)$。
- 气隙功率：$P_{ag} = T_{em}\,\omega_{shaft}$。

## SVPWM 三相逆变器与 DC 母线模型
- 参考电压输入改为 dq 坐标：$u_d^{*},u_q^{*}$。
- 逆 Park + 逆 Clarke 变换得到三相参考：
  $$u_\alpha^{*}=u_d^{*}\cos\theta_e-u_q^{*}\sin\theta_e,\quad u_\beta^{*}=u_d^{*}\sin\theta_e+u_q^{*}\cos\theta_e$$
  $$u_a^{*}=u_\alpha^{*},\quad u_b^{*}=-\frac{1}{2}u_\alpha^{*}+\frac{\sqrt{3}}{2}u_\beta^{*},\quad u_c^{*}=-\frac{1}{2}u_\alpha^{*}-\frac{\sqrt{3}}{2}u_\beta^{*}$$
- SVPWM 零序注入：
  $$u_0=-\frac{\max(u_a^{*},u_b^{*},u_c^{*})+\min(u_a^{*},u_b^{*},u_c^{*})}{2}$$
  $$\tilde u_a=u_a^{*}+u_0,\quad \tilde u_b=u_b^{*}+u_0,\quad \tilde u_c=u_c^{*}+u_0$$
- 线性调制区限幅（当前实现）：
  $$|\tilde u_{phase}|\le k_{lin}V_{dc},\quad k_{lin}=0.577350269$$
- 占空比映射：
  $$d_a=\mathrm{clip}\left(0.5+\frac{u_a}{V_{dc}},0,1\right),\ d_b=\mathrm{clip}\left(0.5+\frac{u_b}{V_{dc}},0,1\right),\ d_c=\mathrm{clip}\left(0.5+\frac{u_c}{V_{dc}},0,1\right)$$
- 逆变器直流侧电流（功率等效）：
  $$P_{ac}=u_a i_a+u_b i_b+u_c i_c,\quad i_{inv,dc}=\frac{P_{ac}}{\max(|V_{dc}|,1)}$$
- 母线电容动态：
  $$\dot V_{dc}=\frac{i_{source}-i_{inv,dc}}{C_{dc}}$$
  其中 $i_{source}=i_{frontend}+i_{batt}$。
- 前端供电（简化导纳模型）：
  $$i_{frontend}=\mathrm{clip}(G_{frontend}(V_{dc,nom}-V_{dc}),-I_{frontend,max},I_{frontend,max})$$
- 双向电池接口（预留）：
  $$i_{batt}=\mathrm{clip}(i_{batt}^{cmd},-I_{chg,max},I_{dis,max})$$
  约定 $i_{batt}>0$ 为电池向母线放电，$i_{batt}<0$ 为母线给电池充电。

## 机械侧（刚性同轴，当前实现）
- 轴上总摩擦（线性阻尼）：$T_{fric} = \omega_{shaft}\,(b_{motor} + b_{turb} + b_{pump})$。
- 角加速度：$\dot\omega = \dfrac{T_{em} + T_{turb} - T_{pump} - T_{fric}}{J_{motor} + J_{turb} + J_{pump}}$。
- 机械角位置积分：$\theta \leftarrow \theta + \omega\,T_s$（代码未对 $2\pi$ 取模）。
- 电角度在电机模型中单独计算：$\theta_e = p\,\theta$（随后用于 Clarke/Park 变换）。

## 全系统求解器（统一 RK4）
- 状态向量：$x=[i_d,i_q,\theta_{mech},\omega_{mech},p_{plenum},p_{discharge},V_{dc}]$。
- 每个子步（$k_1\sim k_4$）都执行同样的部件耦合顺序：
  - 逆变器：根据 $u_d^{*},u_q^{*},\theta_e$ 和 $V_{dc}$ 计算 SVPWM 后的 $u_a,u_b,u_c$。
  - 电机：基于逆变器输出电压计算电磁转矩 $T_{em}$ 与电流导数。
  - 逆变器 DC 侧：根据 $P_{ac}$ 估算 $i_{inv,dc}$ 并计算 $\dot V_{dc}$。
  - 涡轮：计算 $\dot p_{plenum}$ 与涡轮扭矩 $T_{turb}$。
  - 泵：计算 $\dot p_{discharge}$ 与负载扭矩 $T_{pump}$。
  - 轴系：用 $T_{em},T_{turb},T_{pump}$ 计算 $\dot\theta_{mech},\dot\omega_{mech}$。
- RK4 数值工具函数位于 `model/solver/RK4Utils.cpp`。

## 涡轮气动→扭矩（容积动态模型）
- 新增涡轮前腔体压力状态 $p_{plenum}$，通过质量守恒更新：
  $$\dot p_{plenum} = \frac{R\,T_{in}}{V_{plenum}}\,(\dot m_{in} - \dot m_{turb})$$
- 入口流量采用压差驱动近似：$\dot m_{in} = C_{in}\,\dot m\,\sqrt{\max(p_{in}-p_{plenum},0)/p_{in}}$。
- 腔体到背压的流量采用可压缩喷嘴关系（按临界压比区分壅塞/非壅塞）。
- 涡轮做功由腔压到背压的等熵膨胀计算：
  - $T_{out,ideal} = T_{in}\,(p_{out}/p_{plenum})^{(\gamma-1)/\gamma}$
  - $P_{turb} = \dot m_{turb}\,c_p\,\eta_{turb}\,(T_{in}-T_{out,ideal})$，其中 $c_p=\gamma R/(\gamma-1)$
- 轴扭矩：$T_{turb} = \dfrac{P_{turb}}{\max(|\omega_{shaft}|,\,\omega_{floor})}$。


## 离心泵（容积动态实现）
 输入：轴速 $\omega$、吸入压力 $p_{suction}$、下游压力 $p_{downstream}$，以及出口等效容积 $V_{out}$、体积弹性模量 $\beta_{eff}$、流阻 $R_{out}$。
 泵产生流量（简化）:
  $$Q_{pump} = \max\left(K_q\,\omega_{abs} - K_{slip}\,\max(p_{discharge}-p_{suction},0),\,0\right)$$
 下游流量（流阻模型）:
  $$Q_{out} = \max\left(\frac{p_{discharge}-p_{downstream}}{R_{out}},\,0\right)$$
 出口压力动态（容积模型）:
  $$\dot p_{discharge} = \frac{\beta_{eff}}{V_{out}}\,(Q_{pump}-Q_{out})$$
 压升与扬程：$\Delta p = \max(p_{discharge}-p_{suction},0)$，$H = \Delta p/(\rho g)$。
 轴功率与负载扭矩：$P_{shaft}=\Delta p\,Q_{pump}/\eta_{pump}$，$T_{pump}=P_{shaft}/\omega_{abs}$。
