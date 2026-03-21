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
- 电角速度：$\omega_e = p\,\omega_{mech}$。
- 电流微分：
  - $\dot i_d = \dfrac{u_d - R_s i_d + \omega_e L_q i_q}{L_d}$
  - $\dot i_q = \dfrac{u_q - R_s i_q - \omega_e L_d i_d - \omega_e\psi_f}{L_q}$
- 电磁转矩：$T_{em} = 1.5\,(p/2)\,(\psi_f i_q + (L_d - L_q) i_d i_q)$。
- 定子铜耗：$P_{cu} = 1.5\,R_s\,(i_d^2 + i_q^2)$。
- 气隙功率：$P_{ag} = T_{em}\,\omega_{shaft}$。

## 机械侧（刚性同轴，当前实现）
- 轴上总摩擦（线性阻尼）：$T_{fric} = \omega_{shaft}\,(b_{motor} + b_{turb} + b_{pump})$。
- 角加速度：$\dot\omega = \dfrac{T_{em} + T_{turb} - T_{pump} - T_{fric}}{J_{motor} + J_{turb} + J_{pump}}$。
- 机械角位置积分：$\theta \leftarrow \theta + \omega\,T_s$（代码未对 $2\pi$ 取模）。
- 电角度在电机模型中单独计算：$\theta_e = p\,\theta$（随后用于 Clarke/Park 变换）。

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
