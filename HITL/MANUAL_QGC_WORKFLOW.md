# 串列翼 HITL 手动操作与 QGroundControl 配置流程

更新日期：2026-09-14
适用仓库：`STaircraft` 模型与 `PX4-WR-ST` 固件
适用场景：执行器已物理断开，仅使用 MATLAB 植物模型、PX4 飞控和 QGroundControl（QGC）进行半物理仿真。

## 1. 使用边界

- 每次开始前重新确认所有实体执行器已断开；不能仅依赖上一次检查结果。
- MATLAB 独占 TELEM2 对应的 FTDI 串口，QGC 独占飞控 USB 串口。不要让 QGC、Python 和 MATLAB 争用同一个端口。
- `runtime_control.txt` 中的 `force_enable=0` 表示冻结模型动力学，`force_enable=1` 才开始积分。任何异常先改回 `0`，再在 QGC 中上锁。
- 本流程记录的是当前已经使用过的配置。串口号会随 Windows 枚举变化，端口身份应以设备管理器中的 VID/PID 和实际连接为准。
- 当前完整参数快照是 JSON 证据文件，不是 QGC 可直接导入的 `.params` 文件。需要在 QGC 参数页逐项核对。

## 2. 连接拓扑

| 链路 | 当前枚举示例 | 配置 | 用途 |
|---|---|---|---|
| MATLAB → FTDI → PX4 TELEM2 | `COM9`，`VID_0403&PID_6001` | 115200 baud | MATLAB 发送 `HIL_STATE_QUATERNION`，接收 `SERVO_OUTPUT_RAW` |
| QGC → PX4 USB | `COM5`，`VID_3163&PID_004C` | USB CDC；手动串口项可填 115200 | 模式、解锁、任务、参数和遥测监视 |
| PX4 Bootloader | 历史上为 `COM3` | 只用于刷写 | 正常 HITL 运行时不要连接 |

端口占用规则：

1. MATLAB 只打开 TELEM2/FTDI 端口。
2. QGC 只打开 PX4 USB 端口。
3. `HITL/tests/*.py` 自动化脚本也使用 PX4 USB；运行 QGC 时不要同时启动这些脚本。
4. `run_hitl_stand_takeoff.m` 的部分旧提示文字仍写着 `COM4`，实际端口以 [`user_hitl_config.m`](user_hitl_config.m) 为准，当前是 `COM9`。

## 3. 手动运行会用到的文件

| 文件 | 何时使用 | 手动需要关注的内容 |
|---|---|---|
| [`user_hitl_config.m`](user_hitl_config.m) | 每次运行前 | TELEM2 串口、波特率、经纬度、海拔、航向、气动开关和执行器动态 |
| [`runtime_control.txt`](runtime_control.txt) | 运行中 | `force_enable=0/1`，冻结或释放模型 |
| [`run_hitl_stand_takeoff.m`](run_hitl_stand_takeoff.m) | 跃升起飞、巡航、转换、定点 | 通用全流程模型运行器；QGC 负责 USB 侧操作 |
| [`run_hitl_direct_landing.m`](run_hitl_direct_landing.m) | 不飞航线，直接验证着陆 | 以机头朝上、离地 8 m 的冻结状态启动 |
| [`tests/run_mission_stand_takeoff.py`](tests/run_mission_stand_takeoff.py) | 自动流程或核对航点逻辑 | 与 QGC 二选一，不要同时占用 PX4 USB |
| [`tests/run_direct_landing.py`](tests/run_direct_landing.py) | 自动直接着陆 | 当前直接着陆的已验证 USB 侧控制顺序，可作为手动步骤基准 |

当前 `user_hitl_config.m` 的关键值：

- TELEM2：`COM9 @ 115200`。
- 原点：纬度 `34.021511°`、经度 `108.757100°`、AMSL `500 m`、航向 `0°`。
- 滑流、滑流速度修正、机身/机翼气动力均开启。
- 电机纯延迟 `0 s`，一阶时间常数 `0.03 s`；舵面延迟和一阶时间常数均为 `0 s`。
- 普通初值覆盖关闭；空中旋翼初始姿态为物理俯仰 `90°`。

## 4. QGC 连接和输入设置

### 4.1 建立连接

1. 先关闭所有会占用 PX4 USB 的 Python 脚本和串口工具。
2. 打开 QGC，优先使用自动连接 Pixhawk/USB。
3. 如需手动连接，在“应用设置/通讯链路（Comm Links）”中新建 Serial 链路，选择当前的 PX4 USB 端口；不要选择 TELEM2 的 `COM9`。
4. 等待 QGC 显示飞控、姿态和定位状态，再进行模式选择或解锁。

### 4.2 遥控器或虚拟摇杆

- 使用实体遥控器：`COM_RC_IN_MODE=0`。
- 没有实体遥控器、使用 QGC 虚拟摇杆：`COM_RC_IN_MODE=1`，并在 QGC“应用设置/常规”中启用 Virtual Joystick。
- `COM_RC_IN_MODE=2` 是另一种虚拟 RC 输入方式，不是当前验证基线，不建议作为默认设置。
- 试验结束后把 `COM_RC_IN_MODE` 恢复为当前板上基线值 `0`。

## 5. QGC 参数核对表

进入 Vehicle Setup → Parameters，通过参数名搜索。修改后等待飞控回读，必要时重启飞控；不要用“重置全部参数”。

### 5.1 机型、HITL 和 TELEM2

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `SYS_AUTOSTART` | 13020 | 当前串列翼机型 |
| `SYS_HITL` | 1 | 开启 HITL |
| `VT_TYPE` | 0 | 尾座式 VTOL |
| `MAV_TYPE` | 20 | VTOL 类型上报 |
| `MAV_1_CONFIG` | 102 | MAVLink 实例 1 使用 TELEM2 |
| `MAV_1_MODE` | 1 | 当前 MAVLink 模式 |
| `MAV_1_RATE` | 0 | 自动选择发送速率 |
| `SER_TEL2_BAUD` | 115200 | 与 MATLAB 一致 |

### 5.2 安全状态和一次性试验开关

| 参数 | 正常起始值 | 说明 |
|---|---:|---|
| `CBRK_VTOLARMING` | 0 | 不绕过 VTOL 解锁检查 |
| `COM_RCL_EXCEPT` | 0 | 不额外豁免遥控链路检查 |
| `TD_FW_TKO_EN` | 0 | 跃升释放钥匙，任务开始后才置 1 |
| `TD_BTR_DBG_FAST` | 0 | 不绕过后转换门槛 |
| `TD_MC_DIRECT_EN` | 0 | 不使用直接混控调试通路 |
| `COM_DISARM_LAND` | 0.5 s | 完全落地后自动上锁延时 |

### 5.3 航线和后转换

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `TD_FW_NAV_DIR` | 0 | 使用标准 L1 航段制导 |
| `TD_FW_WP_ACC` | 45 m | 固定翼航点接受半径上限 |
| `FW_L1_METHOD` | 0 | 当前 L1 方法 |
| `FW_L1_PERIOD` | 20 s | L1 响应周期 |
| `FW_L1_DAMPING` | 0.8 | L1 阻尼 |
| `FW_L1_R_SLEW_MAX` | 8 deg/s | 滚转设定变化率限制 |
| `FW_R_LIM` | 25 deg | 固定翼滚转限制 |
| `FW_AIRSPD_TRIM` | 13 m/s | 固定翼基准空速 |
| `TD_BTR_ARSP` | 13.5 m/s | 后转换最大准入速度 |
| `TD_BTR_ROLL` | 10 deg | 后转换滚转门槛 |
| `TD_BTR_PITCH` | 15 deg | 后转换俯仰门槛 |
| `TD_BTR_GATE_T` | 0.5 s | 三项条件连续满足时间 |
| `TD_BTR_THR` | 0.40 | 等待准入时的最大油门 |

### 5.4 六点着陆

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `TD_LAND_CTL_EN` | 1 | 在 HITL 旋翼模式使用六点接触判定 |
| `TD_LAND_CNT_T` | 0.10 s | 六点全部接触确认时间 |
| `TD_TIP_GND_EN` | 1 | 开启尾部三点触地保护 |
| `TD_TIP_GND_T` | 0.10 s | 尾部三点连续接触确认时间 |
| `TD_TIP_GND_PWM` | 1900 us | 尾部触地后 MAIN7/8 保持约 90% |
| `TD_LAND_M_PWM` | 1400 us | 尾部触地后 MAIN1～4 的统一输出 |

全部新增参数的作用、范围和板上当前值见 [`PX4-WR-ST/docs/shengtai/secondary_development_parameters_after_20260710.md`](../../PX4-WR-ST/docs/shengtai/secondary_development_parameters_after_20260710.md)。

## 6. 通用启动顺序

1. 物理确认全部执行器断开。
2. 确认 QGC/Python 未占用 `COM9`，且没有 Python 脚本与 QGC 同时占用 PX4 USB。
3. 把 [`runtime_control.txt`](runtime_control.txt) 设为 `force_enable=0`。
4. 在 QGC 核对飞控处于上锁状态，且 `TD_FW_TKO_EN=0`、`TD_BTR_DBG_FAST=0`、`TD_MC_DIRECT_EN=0`。
5. 在 MATLAB 中运行：

   ```matlab
   run('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/run_hitl_stand_takeoff.m')
   ```

6. 等待 MATLAB 显示实际串口已打开、持续收到 `SERVO_OUTPUT_RAW`，并确认冻结姿态稳定。
7. 再打开 QGC 的 PX4 USB 链路，确认姿态、位置、模式和告警正常。
8. 只有在所需模式已接受、解锁状态和输出均正常后，才把 `force_enable` 改为 `1`。

## 7. 手动旋翼模式检查

1. 保持 `force_enable=0`，在上锁状态选择 Multi-Rotor，再选 Stabilized。
2. 在 MAVLink Inspector 中确认 `EXTENDED_SYS_STATE.vtol_state=3`。
3. 解锁，先观察 MAIN1～4、MAIN7～8 输出是否合理；无异常后设 `force_enable=1`。
4. 按 Stabilized → Altitude → Position 的顺序逐级验证，每一级只给小量输入。
5. Position 定点时物理俯仰不一定是 `90°`；当前日志中的 `82°～83°` 是位置环为修正水平速度/位置而主动给出的倾角，不能据此判定姿态环失效。

## 8. 用 QGC 手动执行整条航线

### 8.1 在 Plan 中建立任务

QGC 中以原点 `34.021511°N, 108.757100°E, AMSL 500 m` 为 Home。下表高度均按相对 Home 高度填写；坐标来自最近一次完整流程的[航点导出](../../report/experiments/20260913-214727_picture-route-fullflow-landing/data/picture_route_waypoints.csv)。

| 序号 | QGC 任务项 | 纬度 | 经度 | 相对高度 | 关键设置 |
|---:|---|---:|---:|---:|---|
| 0 | Takeoff（`NAV_TAKEOFF`） | 34.0228584668 | 108.7571000000 | 30 m | 最小俯仰 10°；航向 0° |
| 1 | Change Speed | — | — | — | Airspeed，13 m/s，Throttle -1 |
| 2 | Waypoint | 34.0235771157 | 108.7571000000 | 30 m | 接受半径 45 m |
| 3 | Waypoint | 34.0246550891 | 108.7554742496 | 30 m | 接受半径 45 m |
| 4 | Waypoint | 34.0246550891 | 108.7511389152 | 30 m | 接受半径 45 m |
| 5 | Waypoint | 34.0322907341 | 108.7511389152 | 30 m | 接受半径 45 m |
| 6 | Waypoint | 34.0322907341 | 108.7571000000 | 30 m | 接受半径 45 m |
| 7 | Change Speed | — | — | — | Airspeed，11.5 m/s，Throttle -1 |
| 8 | Waypoint（下滑段） | 34.0260025559 | 108.7571000000 | 15 m | 接受半径 20 m |
| 9 | Waypoint（拉平/稳定） | 34.0242059335 | 108.7571000000 | 18 m | Hold 300 s；接受半径 20 m |
| 10 | Change Speed | — | — | — | Ground speed，1.5 m/s，Throttle -1 |
| 11 | Loiter Unlimited（转换屏障） | 34.0242059335 | 108.7571000000 | 18 m | 在此人工判断后转换条件 |
| 12 | Waypoint（旋翼平移） | 34.0236669468 | 108.7571000000 | 15 m | Hold 2 s；接受半径 10 m |
| 13 | Loiter Time（定点） | 34.0236669468 | 108.7571000000 | 15 m | 5 s |
| 14 | Land | 34.0236669468 | 108.7571000000 | 0 m | 垂直着陆 |

注意：自动脚本会在序号 11 主动请求 FW→MC，并在确认 `vtol_state=3` 后把当前任务项推进到序号 12。纯 QGC 操作不会自动完成这两步。

### 8.2 起飞与巡航

1. 保持模型冻结，上传任务并确认序号和坐标。
2. 在上锁状态选择 Fixed-Wing；当前图片航线的已验证自动顺序是在 Mission 模式下解锁。手动 QGC 若解锁被拒绝，先读取 Preflight/STATUSTEXT 原因，不要关闭安全检查硬闯。
3. 启动任务，确认当前任务项为 Takeoff。
4. 在 QGC 参数页把 `TD_FW_TKO_EN` 从 `0` 改为 `1`，等待至少 2 s。
5. 把 `force_enable` 改为 `1`。`TD_FW_TKO_EN` 在本次起飞中只操作一次，不要来回切换。
6. 监视高度、空速、滚转和横航迹误差，确认飞机按序号 2～9 飞行。

### 8.3 后转换、定点与着陆

1. 到达序号 11 后保持 Loiter Unlimited，确认空速不高于 13.5 m/s、`|roll|≤10°`、`|pitch|≤15°` 并连续稳定至少 0.5 s。
2. 在 QGC Fly 页执行 VTOL Transition/Transition to Multi-Rotor。
3. 确认 `EXTENDED_SYS_STATE.vtol_state=3`，且姿态、速度和输出稳定。
4. 在 QGC 中把当前任务项推进到序号 12。不同 QGC 版本入口可能显示为“Set current mission item/设为当前航点”。
5. 观察旋翼平移、5 s 定点和 Land。尾部三点接触后 MAIN7/8 应约为 1900 us，六点全部接触并保持 0.10 s 后飞控应确认落地，随后约 0.5 s 自动上锁。

如果 QGC 版本支持在任务中加入 `DO_VTOL_TRANSITION`，理论上可以替代人工转换和推进任务项；该方案尚未作为当前完整流程验证，不属于本流程的已验证路径。

## 9. 不飞航线、直接验证着陆

直接着陆的模型入口是：

```matlab
run('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/run_hitl_direct_landing.m')
```

模型会先用地面高度建立本地原点，约 3 s 后显示冻结的机头朝上、离地 8 m 状态。当前已验证的 USB 侧顺序来自 `tests/run_direct_landing.py`：

1. 等待本地位置有效，保持模型冻结并确认上锁。
2. 选择 Multi-Rotor → Altitude，解锁。
3. 短暂进入 Stabilized，使已在空中的 HITL 初态进入飞行状态；随后回到 Altitude。
4. 等待主桨输出稳定，再选择 Land/AUTO.LAND。
5. 确认 Land 已接受后，把 `force_enable` 改为 `1`。
6. 监视尾部三点、翼尖桨 1900 us、六点接触、落地状态和自动上锁。

这一直接着陆顺序的自动脚本使用 HITL 专用强制解锁命令。纯 QGC 的普通解锁如果被 Preflight 拒绝，不要通过修改安全参数绕过；应保持模型冻结并改用已验证的自动脚本或先排除拒绝原因。

## 10. QGC 中应重点监视的消息

打开 Analyze Tools → MAVLink Inspector，重点查看：

- `HEARTBEAT.base_mode`：是否已解锁，以及当前模式。
- `EXTENDED_SYS_STATE.vtol_state`：`1` 前转换、`2` 后转换、`3` 旋翼、`4` 固定翼。
- `EXTENDED_SYS_STATE.landed_state`：六点接触后是否进入 On Ground。
- `ATTITUDE`、`ATTITUDE_TARGET`：实际姿态和目标姿态是否一致。
- `LOCAL_POSITION_NED`、`GLOBAL_POSITION_INT`、`VFR_HUD`：位置、高度、速度和空速。
- `NAV_CONTROLLER_OUTPUT`：`nav_roll`、`nav_bearing`、`target_bearing`、`xtrack_error`。
- `SERVO_OUTPUT_RAW`：MAIN1～8 输出，特别是触地后的 MAIN1～4 与 MAIN7/8。
- `NAMED_VALUE_FLOAT` 中的 `TD_CNTCT`：模型接触位掩码；尾部三点为 `0x38`，六点全部为 `0x3F`。
- `STATUSTEXT`：解锁拒绝、位置失效、数据过期和 failsafe 原因。

## 11. 异常中止和收尾

发生姿态发散、速度越界、输出异常、串口丢失或 QGC failsafe 时：

1. 立即把 `runtime_control.txt` 改回 `force_enable=0`，冻结模型。
2. 在 QGC 中上锁；若普通上锁无响应，再使用经过验证的 HITL 强制上锁工具。
3. 把 `TD_FW_TKO_EN=0`、`TD_BTR_DBG_FAST=0`、`TD_MC_DIRECT_EN=0`。
4. 如使用了虚拟摇杆，把 `COM_RC_IN_MODE` 恢复为 `0`。
5. 停止本次 MATLAB 运行器，关闭 QGC/脚本并释放串口。
6. 最后从 `HEARTBEAT` 和输出通道共同确认飞控已上锁。

## 12. Position 定点时物理俯仰约 82°～83°的论证

现有 `TD_MC_HOV_P=90°` 只在尾座式**手动 Stabilized 旋翼模式**生成中立姿态目标；Position 模式的姿态目标来自位置/速度误差形成的推力矢量，因此这个参数不会把 Position 定点姿态改成 90°。

最近一次完整流程在约 340～342 s 的定点窗口内，模型物理俯仰约为 `82.13°～83.36°`，平均约 `82.79°`；PX4 适配坐标系内的实际俯仰和目标俯仰都约为 `-7°`，两者跟踪接近。这说明 80 多度主要是外层位置控制器主动要求的水平推力，而不是内层姿态环跟不上。

技术上可以新增参数，但不建议增加“Position 模式强制物理俯仰为 90°”的参数，因为它会消除维持位置所需的水平推力并可能造成漂移。更合理的候选是一个默认值为 `0°`、仅在纯旋翼 Position/Velocity 模式生效的小范围中立俯仰修正量，例如 `TD_MC_POS_P_TRIM`，建议范围 `-10°～+10°`；它应叠加在位置环给出的动态倾角上，而不是覆盖位置环。

在增加该参数前，应先记录 10～20 s 稳态 Position 数据，对比 `ATTITUDE_TARGET`、`ATTITUDE`、水平位置/速度误差及主桨输出。只有在位置和速度误差接近零时仍长期存在固定目标偏置，才值得参数化；否则应优先核对模型重心、气动力、推力轴和配平，而不是用 90° 强制值掩盖平衡需求。本文件只做方案论证，未修改控制代码或参数。
