# TandemTailSitter HITL 智能体交接说明

更新日期：2026-07-20

## 0. 2026-07-20 最新基线

固定翼航线跟踪与鲁棒参数已完成长四边复验，最新证据见：

`D:\D_zx\26WORK\ShengTai\0710HITL_ST\report\2026-07-20_固定翼起飞门控与长四边鲁棒性验证.md`

- 固定翼默认使用 `TD_FW_NAV_DIR=0` 的上一航点到当前航点线段跟踪，不使用直奔目标点模式。
- Mission TAKEOFF 位于北向约 `150 m`，首个普通航线点约 `230 m`，两者间距由 `250 m` 缩短为 `80 m`。
- 默认长四边为闭合 `600 x 650 m`；四个主要转角一次性横迹峰值约 `71.5-73.7 m`，约 `21-22 s` 捕获，捕获后峰值不超过 `11.4 m`，没有重复穿线。
- 固定翼鲁棒参数基线为 `FW_RR_P=0.02`、`FW_RR_I=0`、`FW_RR_FF=0.05`、`FW_RR_IMAX=0.05`、`FW_R_TC=1.5`、`FW_R_RMAX=15`、`FW_R_LIM=15`、`FW_L1_PERIOD=50`、`FW_L1_DAMPING=1`、`FW_L1_R_SLEW_MAX=15`。
- 不要恢复历史建议的 `FW_RR_P=0.008`：当前模型实测会导致实际滚转约 `+78 deg`、横迹超过 `120 m`。
- `TD_FW_TKO_EN` 只控制 AUTO Mission TAKEOFF；参数置 `1` 后连续保持约 `2 s` 才释放跃升输出。本轮固件实测为 `2.015 s`。
- 最新镜像 SHA256：`D7565C206B4287A910989BD3F25AF1C9C7E4A494F561A4E431DE7AE93FD767BC`。
- 长四边、正常门控 FW->MC、旋翼 Position hold 和 MATLAB 回归均通过；垂直着陆仍不在本轮验收范围。

## 1. 接手后先读

先阅读工作区总报告：

`D:\D_zx\26WORK\ShengTai\0710HITL_ST\report\0718工作进度.md`

再阅读 7 月 19 日详细变更：

`D:\D_zx\26WORK\ShengTai\0710HITL_ST\report\0719工作进度.md`

两份报告包含当前架构、飞控实读参数、文件级修改、验证结果、实飞边界和剩余问题。本文件只提供下一位智能体立即开始工作的操作入口。

## 2. 当前任务状态

已通过：

- 旋翼 Stabilized 三轴自稳和打杆恢复；
- 旋翼 Position/Altitude 控制；
- 固定翼 Stabilized；
- Mission 跃升起飞两秒许可门控；
- 五边形和四边航线；
- 固定翼低速拉起后 FW→MC；
- 纯 `HIL_STATE_QUATERNION` 下旋翼位置有效性；
- 后三点接地消息和翼尖桨 1500 PWM 保护；
- MATLAB HITL 全部单元测试。

尚未通过：

- 接地终止逻辑修改后的完整闭环垂直着陆复验。

不要把当前 `--approach-only` 当作有效的纯着陆测试。它仍从固定翼支架跃升开始，并会在爬升阶段提前掉高，尚未到达 FW→MC。

推荐下一步：

1. 建立真正独立的“空中旋翼初始状态→Position/Altitude→缓降→后三点接地”测试入口；或
2. 使用已验证能稳定到达 FW→MC 的完整长航线，只验收最后着陆段。

着陆验收判据见总报告第 9 节。

## 3. 仓库与分支

| 仓库 | 路径 | 当前工作分支 |
| --- | --- | --- |
| PX4 | `D:\D_zx\26WORK\ShengTai\0710HITL_ST\PX4-WR-ST` | `STHITL` |
| MATLAB 模型 | `D:\D_zx\26WORK\ShengTai\0710HITL_ST\STaircraft` | `STHITL` |

根目录 `report` 不属于上述两个 Git 仓库。提交时分别进入两个仓库，不要在根目录初始化新仓库。

提交时不要包含：

- `__pycache__` 和 `.pyc`；
- 临时 PNG；
- 运行生成的 MAT/ULog/控制台日志；
- 与当前任务无关的旧日志删除；
- PX4 嵌套依赖仅因文件时间或工作树状态产生的变化。

## 4. 硬件与端口识别

当前连接：

- FTDI → TELEM2：COM9，115200 baud，供 MATLAB HITL 使用；
- CUAV USB：COM5，供 Python/QGC/NSH 使用。

COM 号会随插拔变化。按设备身份识别：

- `VID_0403:PID_6001`：FTDI/TELEM2；
- `VID_3163:PID_004C`：飞控 USB。

```powershell
Get-PnpDevice -PresentOnly |
  Where-Object FriendlyName -Match 'COM|Serial' |
  Select-Object FriendlyName, InstanceId
```

如果只看到一条链路，不要让 MATLAB 和 Python 同时打开同一个 COM。

## 5. 安全边界

开始前：

- 真实螺旋桨已拆除、动力已断开，或明确处于纯模型环境；
- QGC 在自动测试时关闭；
- `runtime_control.txt` 写成 `force_enable=0`；
- 飞控保持未解锁；
- `TD_FW_TKO_EN=0`。

异常时立即：

1. 写 `force_enable=0`；
2. 停止任务脚本；
3. 通过飞控 USB 发送 force-disarm；
4. 恢复 `TD_FW_TKO_EN=0` 和 `COM_RC_IN_MODE=0`；
5. 停止 MATLAB runner。

新修改 PX4 固件前先向用户说明修改点并申请许可。用户批准后可连续完成修改、编译、刷写和验证，不必在每一步重复申请。

## 6. 正确 HITL 数据链

MATLAB 只发送 `HIL_STATE_QUATERNION`。不要重新组合 `HIL_SENSOR + HIL_GPS`，除非更换为足够带宽的链路并重新验证 EKF。

接地时同一编码链附加：

`NAMED_VALUE_FLOAT(name="TD_REAR", value=1)`

PX4 通过 `SERVO_OUTPUT_RAW` 返回执行机构输出。

冻结位姿必须发送 1 g 静止比力；否则 QGC 中飞机会漂移或被解释为自由落体。

如果 MATLAB 报：

`未识别类 py.pymavlink_bridge.MavlinkBridge 的方法 encode_hil_sensor_and_state`

说明 MATLAB 仍缓存旧 Python 类。当前正确路径不再调用该 bundle 方法；确认使用最新代码后，关闭全部 MATLAB 进程并重新启动，使 Python 模块重新加载。

## 7. MATLAB 启动

确认 `STaircraft\HITL\user_hitl_config.m` 中串口与 FTDI 实际 COM 一致。

```matlab
cd('D:\D_zx\26WORK\ShengTai\0710HITL_ST\STaircraft\HITL');
run_hitl_stand_takeoff
```

必须看到：

- `Serial opened: COMx @ 115200`
- `HITL serial link is alive`
- 初始 `force_enable=0`
- 支架状态 `STAND_HOLD`

当前执行机构设置：

- motor delay = 0；
- elevon delay = 0；
- motor collective tau = 0.03 s；
- elevon tau = 0。

## 8. 自动任务入口

```powershell
python -u "D:\D_zx\26WORK\ShengTai\0710HITL_ST\STaircraft\HITL\tests\run_mission_stand_takeoff.py" --port COM5 ...
```

任务脚本负责：

- 参数临时设置与读回；
- 固定翼/旋翼模式选择；
- Mission 上传和启动；
- `TD_FW_TKO_EN` 两秒起飞门控；
- FW→MC 的高度、空速、下降率、姿态和稳定时间门控；
- failsafe 快速失败；
- Mission 停滞检测；
- 接地速度和翼尖保护检查；
- `finally` 中冻结、上锁和恢复控制参数。

不要仅凭脚本打印“passed”判定成功；还要核对：

- 没有 `Failsafe enabled`；
- 没有高速撞地；
- 最终姿态未翻倒；
- 接地后速度没有再次增长；
- PX4 最终状态为 ON_GROUND。

## 9. 人工 QGC 验证

自动测试关闭 QGC；人工验证可打开 QGC 使用飞控 USB。

旋翼 Stabilized 顺序：

1. `force_enable=0`；
2. 启动 MATLAB 并确认 TELEM2 链路；
3. 未解锁状态下明确切到 Multi-Rotor；
4. 确认 VTOL 状态不是 Fixed-Wing；
5. 选择 Stabilized；
6. 解锁并建立主桨；
7. 设置 `force_enable=1`；
8. 小幅打滚转、俯仰、偏航，松杆后确认不发散。

Mission 跃升时，进入 Mission 后再显式把 `TD_FW_TKO_EN` 从 0 改成 1。固件会再连续等待两秒。

## 10. 测试与验证命令

MATLAB 回归：

```powershell
& 'D:\Program Files\MATLAB\R2025b\bin\matlab.exe' -batch "cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/tests'); run_all_hitl_tests;"
```

Python 入口检查：

```powershell
python -B .\HITL\tests\run_mission_stand_takeoff.py --help
```

当前全套 MATLAB 输出应以：

`All HITL tests passed.`

结束。

## 11. PX4 编译与刷写

构建环境：

- WSL：`RflySim-20.04`
- ARM GCC：`/root/gcc-arm-none-eabi-7-2017-q4-major/bin`
- target：`cuav_nora_default`

```bash
cd /mnt/d/D_zx/26WORK/ShengTai/0710HITL_ST/PX4-WR-ST
export PATH=/root/gcc-arm-none-eabi-7-2017-q4-major/bin:$PATH
make cuav_nora_default
```

最近已刷入镜像 SHA256：

`BF7BC5B35686D44F771526DB1281D29E87B67FB11FE764CFB094095D220DACA5`

若 USB 串口号变化，先重新枚举端口。不要假设历史 COM3/COM5 永远不变。

## 12. 实飞边界

纯 HIL_STATE 和 Commander 的 HIL-only 位置有效性分支只用于本 HITL。实飞前必须：

- 关闭 HIL；
- 恢复真实传感器；
- 恢复实际板安装方向；
- 重新校准 IMU、磁罗盘和空速；
- 重新检查输出方向、failsafe、RC 和数据链。

不要把 HITL 的 `SENS_BOARD_ROT=0` 或零传感器偏置直接用于实飞。

## 13. 记录规范

每次调参或代码修改后，在根目录 `report` 新建或更新 Markdown，至少记录：

- 目标和修改原因；
- 修改文件和参数；
- 精确测试命令；
- 通过/失败判据；
- 关键数值和日志路径；
- 最终安全状态；
- Git 提交哈希。

总进度更新到：

`D:\D_zx\26WORK\ShengTai\0710HITL_ST\report\0718工作进度.md`

只在单项试验报告中保留失败过程；总进度和本交接文件只描述当前正确基线及尚未解决的边界。
