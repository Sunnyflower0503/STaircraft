# Verification

验证时间：2026-07-03

## MATLAB Model

验证命令：

```matlab
run('D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\MODEL\verify_matlab_model.m')
```

验证结果：

```text
MATLAB model verification passed.
dx norm: 12.0384568178
final state norm: 32.3356991999
```

验证内容：

```text
1. init_param_zx 参数初始化成功
2. tandem_zx_dynamics 输出 13x1 状态导数
3. Runge_Kutta4 小步长积分正常
4. 输出均为有限值
```

## C++ Model

验证命令：

```cmd
cd /d D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\MODEL\cpp_model
cl /EHsc /O2 /std:c++17 /I. /Fo:build\ /Fe:build\validate_closedloop.exe aircraft.cpp validate_closedloop.cpp
build\validate_closedloop.exe build\cpp_closedloop.csv
```

验证结果：

```text
aircraft.cpp
validate_closedloop.cpp
正在生成代码...
Saved build\cpp_closedloop.csv
```

验证内容：

```text
1. aircraft.cpp / aircraft.hpp 编译成功
2. validate_closedloop.exe 运行成功
3. build/cpp_closedloop.csv 输出成功
```

## Dynamic Derivative Check

复查时间：2026-07-03

结论：

```text
1. MATLAB 和 C++ 均使用相同的 alpha_dyn = clamp(alpha, -0.5, 0.5) 作为动导数插值自变量。
2. p_hat = p * b / (2 Va)，q_hat = q * MAC / (2 Va)，r_hat = r * b / (2 Va) 两边一致。
3. CYp/CYr、Clbeta/Clp/Clr、Cnbeta/Cnp/Cnr 的插值和组合公式两边一致。
4. C++ 已补齐 CLalpdot * alpha_dot_hat 和 Cmalpdot * alpha_dot_hat。
5. alpha_dot_hat 默认值为 0，与 MATLAB 缺省行为一致。
```

复查后重新运行：

```text
MATLAB model verification passed.
C++ aircraft.cpp / validate_closedloop.cpp 编译成功。
Saved build\cpp_closedloop.csv
```
