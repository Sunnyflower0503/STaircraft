# 螺旋桨/电机动力学模型更新说明

本文档用于指导其它工程目录中的智能体，把旧版 `tandem_rotor_thrust.m`、`tandem_rotor_fm.m` 等动力学模型文件更新为当前的功率平衡模型。修改时应尽量保持原文件名、函数名、输入输出接口不变，避免影响上层仿真模型调用。

## 1. 必须更新的核心问题

旧模型中常见做法是直接用油门多项式得到转速：

```matlab
RPM = polyval([-3424, 12810, -4], delta_t);
```

这种写法只考虑了油门 `delta_t`，没有考虑来流速度 `V0` 对螺旋桨吸收功率和转速的影响，因此在有空速的车载实验/飞行条件下误差较大。

新版模型必须改为：

1. 油门 `delta_t` 先映射为电机可输出轴功率 `P_motor`。
2. 由螺旋桨功率系数 `C_P(J)` 和来流速度 `V0` 建立功率平衡。
3. 求解螺旋桨转速 `n`，单位为 r/s。
4. 根据 `J` 计算 `C_T(J)`、`C_M(J)`。
5. 输出推力、反扭矩和转速。

## 2. 数据来源和拟合系数

当前模型系数来自 `ST建模.xlsx` 中的实测数据。注意：

- `C_T`、`C_M`、`C_P` 是用有来流速度的实测数据拟合的。
- `C_P` 必须由轴功率 `P_shaft` 计算，不能用总电功率 `P_total` 直接计算。
- 轴功率计算公式为：

```math
P_{shaft}=M\omega=2\pi nM
```

其中 `M` 为实测转矩，`n=RPM/60` 为转速，单位 r/s。

当前使用的拟合系数如下：

```matlab
% P_motor = p2*delta_t^2 + p1*delta_t + p0, W
P_motor_coef = [16.501776, 160.89067, -54.531088];

% C_X = a*J^2 + b*J + c
CT_coef = [-0.44128219, 0.067079209, 0.095811585];
CM_coef = [-0.033493272, 0.020157983, 0.0072201342];
CP_coef = [-0.21044444, 0.12665634, 0.045365441];
```

螺旋桨直径：

```matlab
D = 0.2032;   % 8 inch, m
```

默认空气密度：

```matlab
rho = 1.225;
```

如果 `param.rho` 或 `param.air_density` 存在，应优先使用参数中的空气密度。

## 3. 功率平衡推导

推进比定义为：

```math
J=\frac{V_0}{nD}
```

螺旋桨功率系数定义为：

```math
C_P=\frac{P_{shaft}}{\rho n^3D^5}
```

由于：

```math
C_P(J)=a_PJ^2+b_PJ+c_P
```

代入 `J=V0/(nD)` 后，螺旋桨吸收功率为：

```math
P_{shaft}
= C_P(J)\rho n^3D^5
= \rho\left(c_PD^5n^3+b_PD^4V_0n^2+a_PD^3V_0^2n\right)
```

因此新版模型中转速不是直接拟合得到，而是求解：

```math
\rho\left(c_PD^5n^3+b_PD^4V_0n^2+a_PD^3V_0^2n\right)-P_{motor}(\delta_t)=0
```

其中 `n` 单位为 r/s，最后：

```matlab
RPM = 60*n;
```

## 4. `tandem_rotor_thrust.m` 的推荐改法

如果其它目录里仍然叫 `tandem_rotor_thrust.m`，建议保留原函数名和原输入输出，把内部实现替换为当前功率平衡流程。

参考接口：

```matlab
function [Thrust, M_prop, n_rps, CT] = tandem_rotor_thrust(delta_t, alpha, beta, TAS, prop_spin, param, prop_angle)
```

如果原文件函数名不是这个，以原文件函数名为准，只替换内部模型逻辑。

核心实现应包含以下步骤：

```matlab
if nargin < 7
    prop_angle = param.prop_angle(1);
end

D = 0.2032;
rho = get_air_density(param);
delta_t = min(max(delta_t, 0), 1);

ca = cos(prop_angle);
sa = sin(prop_angle);
V_axial = TAS*cos(alpha)*cos(beta)*ca - TAS*sin(alpha)*cos(beta)*sa;

coef = default_propulsion_coefficients();
P_motor = max(polyval(coef.P_motor, delta_t), 0);
n_rps = solve_rps_from_power(P_motor, V_axial, rho, D, coef.CP);
```

然后计算气动系数和输出：

```matlab
if n_rps <= 0
    Thrust = 0;
    M_prop = 0;
    CT = 0;
    return;
end

J = V_axial/(n_rps*D);
CT = polyval(coef.CT, J);
CM = polyval(coef.CM, J);

if CT <= 0
    Thrust = 0;
    M_prop = 0;
    return;
end

CM = max(CM, 0);
Thrust = CT*rho*n_rps^2*D^4;
M_prop = prop_spin*CM*rho*n_rps^2*D^5;
```

需要一并加入三个局部函数：`default_propulsion_coefficients`、`get_air_density`、`solve_rps_from_power`。

`solve_rps_from_power` 推荐用二分法，避免依赖 Optimization Toolbox：

```matlab
function n_rps = solve_rps_from_power(P_motor, V0, rho, D, CP_coef)
    if P_motor <= 0
        n_rps = 0;
        return;
    end

    aP = CP_coef(1);
    bP = CP_coef(2);
    cP = CP_coef(3);

    f = @(n) rho*(cP*D^5*n.^3 + bP*D^4*V0*n.^2 + aP*D^3*V0^2*n) - P_motor;

    lo = 0;
    hi = 10;
    while f(hi) < 0 && hi < 20000
        hi = 2*hi;
    end

    if f(hi) < 0
        n_rps = 0;
        return;
    end

    for k = 1:80
        mid = 0.5*(lo + hi);
        if f(mid) >= 0
            hi = mid;
        else
            lo = mid;
        end
    end

    n_rps = 0.5*(lo + hi);
end
```

## 5. `tandem_rotor_fm.m` 的推荐改法

`tandem_rotor_fm.m` 通常是力/力矩封装文件。如果该文件内部调用旧的 `tandem_rotor_thrust.m`，则优先保持外部接口不变，只确认它调用的推力模型已经换成新版功率平衡模型。

检查重点：

1. 不应在 `tandem_rotor_fm.m` 中再次使用旧的 `RPM=f(delta_t)` 多项式。
2. 如果该文件直接计算推力/力矩，应把直接转速映射替换成功率平衡求解。
3. 如果该文件只是组合多个旋翼的力和力矩，则只需要保证每个旋翼调用新版推力函数。
4. `prop_spin` 的符号应继续用于反扭矩方向：

```matlab
M_prop = prop_spin*CM*rho*n_rps^2*D^5;
```

不要把 `prop_spin` 混入推力方向；推力方向应由机体系安装角、旋翼位置和上层力矩计算逻辑决定。

## 6. 必须删除或替换的旧逻辑

其它目录中如果发现以下逻辑，应视为旧模型并替换：

```matlab
RPM = polyval([-3424, 12810, -4], delta_t);
n = RPM/60;
```

或任何形式的：

```matlab
n = f(delta_t);
RPM = f(delta_t);
```

除非只是用于和旧模型对比验证，否则不应再作为正式动力学模型使用。

## 7. 螺旋桨效率计算

如果需要输出或绘制螺旋桨效率，使用：

```math
\eta_{prop}=\frac{TV_0}{P_{shaft}}=\frac{JC_T}{C_P}
```

对应 MATLAB：

```matlab
eta_prop = J*CT/CP;
```

注意：

- `CP` 来自 `C_P(J)` 或由 `P_shaft/(rho*n^3*D^5)` 计算。
- 不能用总电功率 `P_total` 代替 `P_shaft` 计算 `C_P`。
- 当 `V0 <= 0` 时，推进效率按定义可能为 0 或不可用于评价；零空速下更适合看推力、转速和功率匹配。

## 8. 更新后的验证方法

更新完成后，必须用实测数据验证模型。当前工程中的参考验证脚本为：

```matlab
result = validate_tandem_rotor_power_balance("ST建模.xlsx");
```

其它目录可以直接复制验证思路：

1. 从实验 Excel 中读取油门 `delta_t`、来流速度 `V0`、实测 `RPM`。
2. 旧模型预测：

```matlab
old_rpm = max(polyval([-3424, 12810, -4], delta_t), 0);
```

3. 新模型预测：

```matlab
[~, ~, n_rps, ~] = tandem_rotor_thrust(delta_t, 0, 0, V0, 1, param, 0);
new_rpm = 60*n_rps;
```

4. 计算 RMSE、MAE、最大相对误差、R2：

```matlab
err = rpm_pred - rpm_measured;
RMSE = sqrt(mean(err.^2));
MAE = mean(abs(err));
MaxAbsRelError = max(abs(err./rpm_measured))*100;
R2 = 1 - sum(err.^2)/sum((rpm_measured - mean(rpm_measured)).^2);
```

当前模型在 `ST建模.xlsx` 有空速实测数据上的参考结果：

| 模型 | RMSE/RPM | MAE/RPM | 最大相对误差 | R2 |
|---|---:|---:|---:|---:|
| 旧油门-RPM 直接多项式 | 733.21 | 728.63 | 13.08% | 0.8068 |
| 新功率平衡模型 | 109.61 | 86.02 | 3.70% | 0.9957 |

如果其它目录数据一致，更新后结果应接近上述数值；如果不一致，应检查：

1. `delta_t` 是否为 0 到 1，而不是 0 到 100。
2. `TAS/V0` 单位是否为 m/s。
3. `RPM` 是否转换为 `n=RPM/60`。
4. `P_shaft` 是否使用 `M*2*pi*n`。
5. `C_P` 是否来自轴功率，而不是总电功率。
6. `prop_angle`、`alpha`、`beta` 的单位是否为弧度。

## 9. 修改完成后的最低检查清单

修改完成后，至少确认以下项目：

- 原文件名和函数接口没有破坏上层调用。
- 模型内部不再使用 `RPM=f(delta_t)` 作为正式转速模型。
- `P_motor(delta_t)`、`C_T(J)`、`C_M(J)`、`C_P(J)` 系数与本文档一致。
- 转速通过功率平衡方程求解。
- 有来流速度时，同一油门下 `V0` 改变会导致预测转速改变。
- 推力计算为 `T=C_T*rho*n^2*D^4`。
- 反扭矩计算为 `M=C_M*rho*n^2*D^5`，并保留 `prop_spin` 符号。
- 用实测数据完成 RPM 校验，并输出 RMSE、MAE、最大相对误差和 R2。

