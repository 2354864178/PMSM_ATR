# 动力系统公式梳理

## make 命令示例
- 构建：
```bash
make
```
- 运行示例（生成 build/log.csv）：
```bash
make run
```
- 清理：
```bash
make clean
```
构建产物位于 build/，主程序为 build/main。

数据文件为 build/log.csv，绘图文件为 notebooks/plot.ipynb

## 坐标变换
- Clarke：$\alpha = \tfrac{2}{3}(a - \tfrac{1}{2}b - \tfrac{1}{2}c)$，$\beta = \tfrac{2}{3}(\tfrac{\sqrt{3}}{2}b - \tfrac{\sqrt{3}}{2}c)$。
- Park：$d = \alpha\cos\theta_e + \beta\sin\theta_e$，$q = -\alpha\sin\theta_e + \beta\cos\theta_e$。
- 逆 Park：$\alpha = d\cos\theta_e - q\sin\theta_e$，$\beta = d\sin\theta_e + q\cos\theta_e$。

## PMSM 电气侧（dq 模型）
 输入：三相电压 $u_a,u_b,u_c$、机械角速度 $\omega_{mech}$、电机参数 $R_s,L_d,L_q,\psi_f,p$、采样时间 $T_s$。
- 电角速度：$\omega_e = p\,\omega_{mech}$。
- 电流微分：
  - $\dot i_d = \dfrac{u_d - R_s i_d + \omega_e L_q i_q}{L_d}$
  - $\dot i_q = \dfrac{u_q - R_s i_q - \omega_e L_d i_d - \omega_e\psi_f}{L_q}$
- 电磁转矩：$T_{em} = 1.5\,(p/2)\,(\psi_f i_q + (L_d - L_q) i_d i_q)$。
- 定子铜耗：$P_{cu} = 1.5\,R_s\,(i_d^2 + i_q^2)$。
- 气隙功率：$P_{ag} = T_{em}\,\omega_{shaft}$。

## 机械侧（刚性同轴，当前实现）
 输入：电机电磁转矩 $T_{em}$、涡轮扭矩 $T_{turb}$、泵负载扭矩 $T_{pump}$、参数 $J_{motor},J_{turb},J_{pump},b_{motor},b_{turb},b_{pump}$、采样时间 $T_s$。
- 轴上总摩擦（线性阻尼）：$T_{fric} = \omega_{shaft}\,(b_{motor} + b_{turb} + b_{pump})$。
- 角加速度：$\dot\omega = \dfrac{T_{em} + T_{turb} - T_{pump} - T_{fric}}{J_{motor} + J_{turb} + J_{pump}}$。
- 机械角位置积分：$\theta \leftarrow \theta + \omega\,T_s$（代码未对 $2\pi$ 取模）。
- 电角度在电机模型中单独计算：$\theta_e = p\,\theta$（随后用于 Clarke/Park 变换）。

## 涡轮气动→扭矩（当前实现）
 输入：入口压力 $p_{in}$、出口压力 $p_{out}$、入口温度 $T_{in}$、质量流量 $\dot m$、比热比 $\gamma$、气体常数 $R$、效率 $\eta_{turb}$、轴速 $\omega_{shaft}$、零速保护下限 $\omega_{floor}$。
- 等熵出口温度：$T_{out,ideal} = T_{in}\,(p_{out}/p_{in})^{(\gamma-1)/\gamma}$。
- 理想温降：$\Delta T_{ideal} = T_{in} - T_{out,ideal}$。
- 实际温降：$\Delta T_{actual} = \eta_{turb}\,\Delta T_{ideal}$。
- 轴功率：$P_{turb} = \dot m\,\dfrac{R}{\gamma-1}\,\Delta T_{actual}$。
- 轴扭矩：$T_{turb} = \dfrac{P_{turb}}{\max(\omega_{shaft},\,\omega_{floor})}$（摩擦阻尼集中在轴系模型中处理）。

## 离心泵（当前实现）
 输入：外部给定流量 $Q$、轴速 $\omega$（作为泵速）、流体密度 $\rho$、效率 $\eta_{pump}$、零速保护下限 $\omega_{floor}$。
 - 扬程：$H = \rho\,Q\,\omega_{abs}^2$（简化关系），
 - 除零保护：$\omega_{abs} = \max(|\omega|,\,\omega_{floor})$。
 - 压升：$\Delta p = \rho g H$。
 - 液压功率：$P_{hyd} = \Delta p\,Q = \rho\,Q\,H\,g$。
 - 轴功率：$P_{shaft} = \dfrac{P_{hyd}}{\eta_{pump}}$。
 - 负载扭矩：$T_{pump} = \dfrac{P_{shaft}}{\omega_{abs}}$（零速保护防止除零，尽量保证不为零）。
