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
