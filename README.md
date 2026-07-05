# MODEL

`MODEL` 是一个独立交付目录，把同一套飞机动力学模型的 MATLAB 版本和 C++ 版本放在一起，方便其他智能体或开发者继续验证、接入控制律、编译和版本管理。

本目录已经初始化为独立 git 仓库。

## 来源

```text
matlab_model/  复制自 D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\GJ\model
cpp_model/     复制自 D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\GJ2\cpp
```

`matlab_model` 是原 MATLAB compact aircraft model。`cpp_model` 是按该 MATLAB 模型移植的 C++ 实现，文件名已整理为 `aircraft.cpp` 和 `aircraft.hpp`。

## 目录结构

```text
MODEL/
  README.md
  VERIFICATION.md
  verify_matlab_model.m
  verify_propulsion_power_balance.m
  .gitignore

  docs/
    PROPULSION_MODEL_UPDATE_GUIDE.md

  matlab_model/
    init_param_zx.m
    tandem_zx_dynamics.m
    tandem_aero_fm.m
    tandem_rotor_fm.m
    tandem_rotor_thrust.m
    tandem_addprop_fm.m
    zx_ground_contact_force.m
    Runge_Kutta4.m
    sat.m
    aerodata1_tianshizhiyi_CFDslip.xlsx
    aerodata2_tianshizhiyi.xlsx

  cpp_model/
    aircraft.hpp
    aircraft.cpp
    validate_closedloop.cpp
    README.md
    data/
      aerodata1_tianshizhiyi_CFDslip.xlsx
      aerodata2_tianshizhiyi.xlsx

  propulsion_data/
    ST建模.xlsx
```

`cpp_model/build/` 是编译验证产生的目录，已被 `.gitignore` 忽略。

## 模型接口

MATLAB 和 C++ 均保持 compact model 接口：

```text
state x[13]:
  x(1:3)    position_NED [m]
  x(4:6)    velocity_NED [m/s]
  x(7:10)   quaternion_body_to_earth [qw qx qy qz]
  x(11:13)  body angular rate [rad/s]

control u[12]:
  u(1:8)    dt1..dt8 main rotor throttle
  u(9:10)   dt9, dt10 wingtip auxiliary prop throttle
  u(11:12)  daeL, daeR elevon command [rad]
```

C++ 控制律输出的 `gj2::Control` 与 C++ 模型 `gj2::tandemZxDynamics` 直接兼容。命名空间仍为 `gj2`，只是文件名改为 `aircraft.cpp/.hpp`。

## MATLAB 模型

核心入口：

```matlab
param = init_param_zx();
dx = tandem_zx_dynamics(t, x, u, param);
[t, z, zdot] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), tspan, x0);
```

主要模块：

```text
tandem_zx_dynamics       主 6DOF 动力学
tandem_aero_fm           气动力和气动力矩
tandem_rotor_fm          旋翼力和力矩
tandem_rotor_thrust      旋翼拉力
tandem_addprop_fm        前后推进桨力和力矩
zx_ground_contact_force  地面接触力
Runge_Kutta4             四阶 Runge-Kutta 积分
init_param_zx            参数、气动表、全迎角补全
```

主旋翼推力模型 `tandem_rotor_thrust.m` 已更新为功率平衡模型：

```text
1. delta_t -> P_motor(delta_t)
2. C_P(J) 与来流 V0 建立功率平衡
3. 二分法求 n_rps
4. J = V0 / (n_rps * D)
5. T = C_T * rho * n_rps^2 * D^4
6. M = prop_spin * C_M * rho * n_rps^2 * D^5
```

使用系数：

```text
P_motor = 16.501776 * delta_t^2 + 160.89067 * delta_t - 54.531088
CT(J) = -0.44128219 * J^2 + 0.067079209 * J + 0.095811585
CM(J) = -0.033493272 * J^2 + 0.020157983 * J + 0.0072201342
CP(J) = -0.21044444 * J^2 + 0.12665634 * J + 0.045365441
D = 0.2032 m
```

## C++ 模型

核心入口见 `cpp_model/aircraft.hpp`：

```cpp
gj2::Param param = gj2::initParamZxFwCtrl();
gj2::StateDerivative dx = gj2::tandemZxDynamics(t, x, u, param);
gj2::State x_next = gj2::rk4Step(t, dt, x, u, param);
```

主要 API：

```text
gj2::tandemZxDynamics
gj2::rk4Step
gj2::tandemAeroFm
gj2::tandemRotorFm
gj2::tandemRotorThrust
gj2::tandemAddpropFm
gj2::zxGroundContactForce
gj2::FixedWingController
```

C++ 运行时不直接读取 `.xlsx`。气动数据已经按 `matlab_model/init_param_zx.m` 移植并内嵌到 `aircraft.cpp` 中。`cpp_model/data/` 下的 Excel 文件用于数据来源追溯和人工核对。

主旋翼推力模型 `gj2::tandemRotorThrust` 已与 MATLAB `tandem_rotor_thrust.m` 对齐，使用同一套功率平衡系数和二分法转速求解。辅助桨 `gj2::tandemAddpropFm` 仍保留原 0.127 m 辅助桨模型，未套用 8 英寸主旋翼系数。

## 验证 MATLAB 模型

在 MATLAB 中运行：

```matlab
run('D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\MODEL\verify_matlab_model.m')
```

该脚本验证：

```text
1. init_param_zx 参数初始化
2. tandem_zx_dynamics 输出 13x1 状态导数
3. Runge_Kutta4 小步长积分
4. 输出维度和有限值
```

最近一次验证结果记录在 `VERIFICATION.md`。

## 验证主旋翼功率平衡模型

在 MATLAB 中运行：

```matlab
run('D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\MODEL\verify_propulsion_power_balance.m')
```

该脚本使用 `propulsion_data/ST建模.xlsx` 中的车载实测数据，比较旧油门-RPM 多项式和新功率平衡模型的 RPM 预测误差，并输出：

```text
propulsion_power_balance_validation.csv
```

该 CSV 是验证产物，已由 `.gitignore` 忽略。

## 验证 C++ 模型

先打开 Visual Studio 开发者 PowerShell，确保 `cl` 可用，然后运行：

```cmd
cd /d D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\MODEL\cpp_model
if not exist build mkdir build
cl /EHsc /O2 /std:c++17 /I. /Fo:build\ /Fe:build\validate_closedloop.exe aircraft.cpp validate_closedloop.cpp
build\validate_closedloop.exe build\cpp_closedloop.csv
```

该验证会：

```text
1. 编译 aircraft.cpp / aircraft.hpp
2. 运行 validate_closedloop.exe
3. 输出 build/cpp_closedloop.csv
```

## 等价范围

当前 C++ 模型的目标是与 `matlab_model` 中的 MATLAB compact aircraft model 对齐，而不是直接声明完整复刻 Simulink 模型中所有子系统细节。

已对齐内容：

```text
1. 状态接口 x[13]
2. 控制输入 u[12]
3. 主动力学
4. 气动力/气动力矩
5. 旋翼力/力矩
6. 推进桨力/力矩
7. 地面接触力
8. 参数和气动表
9. CL/CD 全迎角补全逻辑
10. 固定翼控制律 C++ 接口
```

动导数使用说明：

```text
CY_dyn = CYbeta * beta + CYp(alpha_dyn) * p_hat + CYr(alpha_dyn) * r_hat
Cl_dyn = Clbeta(alpha_dyn) * beta + Clp(alpha_dyn) * p_hat + Clr(alpha_dyn) * r_hat
Cn_dyn = Cnbeta(alpha_dyn) * beta + Cnp(alpha_dyn) * p_hat + Cnr(alpha_dyn) * r_hat
CL_dyn = CLq * q_hat + CLalpdot * alpha_dot_hat
Cm_dyn = Cmq * q_hat + Cmalpdot * alpha_dot_hat

p_hat = p * b / (2 Va)
q_hat = q * MAC / (2 Va)
r_hat = r * b / (2 Va)
alpha_dyn = clamp(alpha, -0.5, 0.5)
```

`alpha_dot_hat` 当前默认值为 0，与 MATLAB `tandem_aero_fm.m` 中缺省行为一致；如果后续外部模型提供迎角变化率，可通过 C++ `Param::alpha_dot_hat` 接入。

如需证明更大工况范围内完全一致，应继续增加测试用例，例如：

```text
hover
forward flight
transition
large angle of attack
ground contact
actuator saturation
different altitude and airspeed
```

## 给后续智能体的注意事项

```text
1. 不要把 cpp_model/build/ 提交进 git。
2. 不要把 C++ 改成运行时读取 xlsx，除非明确引入并验证 Excel 解析库。
3. 修改 MATLAB 模型后，应同步检查 aircraft.cpp 中的参数和算法。
4. 修改 C++ 模型后，应至少重新跑 MATLAB smoke test 和 C++ compile/run test。
5. 文件名 aircraft.cpp/.hpp 是当前交付名，不要再改回 gj2.cpp/.hpp。
6. C++ 命名空间仍为 gj2，这是为了减少已有代码接口变动。
```
