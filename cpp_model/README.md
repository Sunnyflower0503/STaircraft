# C++ 独立模型说明

这个目录用于独立编译和运行 C++ 版飞机动力学模型与固定翼控制律。目录里不放批处理脚本，使用 Visual Studio 的 `cl` 编译器直接编译即可。

## 目录内容

```text
aircraft.hpp                C++ 对外接口声明
aircraft.cpp                飞机模型、气动力、旋翼/螺旋桨、地面接触力、控制律实现
validate_closedloop.cpp     C++ 闭环验证入口
data/
  aerodata1_tianshizhiyi_CFDslip.xlsx
  aerodata2_tianshizhiyi.xlsx
```

## 编译

先打开 Visual Studio 开发者 PowerShell，确保 `cl` 命令可用。

然后编译：

```cmd
cd /d D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\MODEL\cpp_model
if not exist build mkdir build
cl /EHsc /O2 /std:c++17 /I. /Fo:build\ /Fe:build\validate_closedloop.exe aircraft.cpp validate_closedloop.cpp
```

运行验证程序：

```cmd
build\validate_closedloop.exe build\cpp_closedloop.csv
```

## 模型接口

C++ 模型保留 MATLAB compact model 的状态量和控制量接口：

```text
x[13]:
  position_NED[3], velocity_NED[3], quaternion_body_to_earth[4], omega_body[3]

u[12]:
  dt1..dt8, dt9, dt10, daeL, daeR
```

固定翼控制律输出的 `u[12]` 可以直接接入模型。

## 主要 C++ API

`aircraft.hpp` 中暴露了模型需要接入控制律时常用的函数：

```text
tandemZxDynamics       对应 MATLAB 主动力学模型
rk4Step                四阶 Runge-Kutta 单步积分
tandemAeroFm           对应气动力/气动力矩
tandemRotorFm          对应旋翼力/力矩
tandemRotorThrust      对应旋翼拉力计算
tandemAddpropFm        对应前后推进桨力/力矩
zxGroundContactForce   对应地面接触力
FixedWingController    固定翼控制律
```

这些接口的目的是让 C++ 版本像 `GJ/model` 中的 MATLAB `.m` 文件一样，可以作为一个模型模块被外部控制律调用。

## XLSX 数据说明

`data/` 中的两个 Excel 文件已经复制进来，用于说明和追溯原始气动数据来源：

```text
data/aerodata1_tianshizhiyi_CFDslip.xlsx
data/aerodata2_tianshizhiyi.xlsx
```

当前 C++ 运行时不直接读取 `.xlsx`。气动数据已经从 `GJ/model/init_param_zx.m` 移植并内嵌在 `aircraft.cpp::initParamZx()` 中，这样 `MODEL/cpp_model` 不需要额外 Excel 解析库也能独立运行。

其中 `CL/CD` 的全迎角数据不是简单夹紧表格，而是按 MATLAB 中相同的平板/失速后补全逻辑在 C++ 中生成。
