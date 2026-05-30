# 位置模拟链路优化设计：运动状态中心（MotionState）

- 日期：2026-05-31
- 状态：已批准设计，待编写实现计划
- 范围：xposed 系统进程侧为主，app 侧仅调整"下发目标"语义

## 1. 背景与问题

Portal 是基于 LSPosed 的虚拟定位模块，通过多层 Hook 在 `system_server` 进程注入位置数据。当前链路：

- **驱动端（app 进程）**：`MockServiceViewModel` 用协程按 `reportDuration` 周期调用 `MockServiceHelper.move(distance, bearing)`，距离按 `speed/(1000/delay)/0.85` 计算；摇杆给方向，路线模拟从路点反算方位角。
- **传递（IPC）**：`move` 经 `sendExtraCommand` → `RemoteCommandHandler.dispatchCommand("move")`，在系统进程一次性跳到新坐标，再 `callOnLocationChanged()` 主动推送。
- **注入（系统进程）**：`injectLocation()` 从静态 `FakeLoc.latitude/longitude` 抖动出坐标。

### 核心病灶：缺乏唯一真相源

速度（`speed`）、航向（`bearing`）、位移三者**各自独立计算**，没有共同的真相源，因此彼此对不上：

- `injectLocation` 里 `location.speed = FakeLoc.speed + 随机量`，与实际两点位移无关。
- `FakeLoc.bearing` 的 getter **有副作用**——每次读取自增 0.5°。`injectLocation`、debug 日志、`get_bearing` 命令各读一次，得到的值都不同。
- GNSS 卫星每次回调全部重新 `shuffle` 随机生成，方位角/仰角相邻帧乱跳，无时间连续性。
- NMEA 的速度、航向、卫星数与 Location 对象对不上。

检测方只要用连续两点算出"位移/时间"，再和上报的 `speed`、`bearing` 一比，立刻发现矛盾；或交叉比对 Location / 卫星 / NMEA 三个通道，发现数据不一致。这是反检测能力（C）和多源一致性（D）的共同根因，也连带导致轨迹（A）、抖动（B）不真实。

### 目标

在同一套架构下同时增强四个侧面（用户确认四者全要、且要求"静止像静止、移动像移动、静↔动平滑"的最高真实度）：

- **A 轨迹/运动真实性**：加减速曲线、转弯航向平滑过渡、位移与速度严格挂钩。
- **B 抖动/精度真实性**：相关随机游走的 GPS 漂移模型，取代固定 ±45° 独立随机。
- **C 反检测能力**：各字段自洽且随时间连续，检测方算不出矛盾。
- **D 多源数据一致性**：Location / GNSS / NMEA 全部从同一快照派生。

## 2. 架构：唯一真相源

新增 `MotionState`（运动状态中心，位于 xposed 系统进程），作为**所有位置数据的唯一真相源**。

```
            ┌─────────── app 进程 ───────────┐
摇杆/路线 ──▶│ 只下发"目标"：targetSpeed +    │
            │ targetBearing（或目标路点）    │── IPC(move/set_*) ──┐
            └────────────────────────────────┘                    │
                                                                   ▼
            ┌──────────────── system_server 进程 ─────────────────────┐
            │  MotionState（唯一真相源）                              │
            │   • anchor(lat, lon, alt, time)                        │
            │   • velocity(speed, bearing)  target(speed, bearing)   │
            │   • 心跳线程按 Δt 推进运动学，产出不可变快照 snapshot   │
            └────────────────────────────────────────────────────────┘
                 │ snapshot()        │ snapshot()         │ snapshot()
                 ▼                   ▼                    ▼
          injectLocation()      GNSS 卫星派生        NMEA 派生
          (Location 对象)       (方位/仰角/速度)     (GGA/RMC/速度航向)
```

**核心原则**：位移、速度、航向只在一处演进；Location / GNSS / NMEA 全部从同一份快照派生，因此数学上必然自洽。

## 3. 运动学模型（A 轨迹 + B 抖动）

`MotionState` 每个心跳 tick 做一步定步长积分，`dt = 当前 tick - 上次 tick`（由 `SystemClock.elapsedRealtimeNanos` 计算真实间隔，避免线程调度抖动导致位移不均）：

1. **速度趋近（加减速曲线）**：`speed` 以加速度 `a` 朝 `targetSpeed` 逼近，而非瞬间跳变。起步平滑加速、松摇杆时 `targetSpeed=0` 平滑减速，实现静↔动平滑切换。
2. **航向缓转（转弯过渡）**：`bearing` 以最大角速度（°/s）朝 `targetBearing` 旋转，走最短弧，不瞬转。
3. **位移积分**：`distance = speed × dt`，沿当前 `bearing` 用大地测量公式推进 anchor 坐标。位移与速度、航向由同一次积分产生，三者天生一致。
4. **GPS 抖动模型升级（B）**：把现有"固定 ±45° 独立随机"换成**相关随机游走**（Ornstein-Uhlenbeck 式）——抖动量在前一帧基础上小幅演进。静止时表现为缓慢游走的真实漂移，移动时叠加在轨迹上。`accuracy` 随之关联，而非孤立随机。

**移除 `FakeLoc.bearing` getter 的副作用**（每读自增 0.5°）——它是"无真相源"的病根，由 `MotionState` 的显式航向演进取代。

## 4. 命令语义改动（IPC 协议兼容）

保持现有命令名和 Bundle 协议不变（不破坏 IPC），只改语义：

| 命令 | 现状 | 改后语义 |
|------|------|----------|
| `move` | 立刻跳 distance 米 + 主动广播 | `targetBearing = bearing`；`targetSpeed`：`distance > 0` 时取 `FakeLoc.speed`（已由 `set_speed`/`start` 同步），`distance == 0` 时取 0（停止）。不再直接搬坐标、不再主动广播；位移交给心跳。**不反推 dt，避免系统侧耦合 app 的 reportDuration** |
| `set_bearing` | 硬写 bearing + 置 hasBearings | 设置 `targetBearing`（缓转目标） |
| `set_speed` | 硬写 speed | 设置 `targetSpeed` |
| `update_location`（`=`/`+`/`-`/`*`/`/`/`random`） | 硬写坐标 | **瞬移**语义：硬重置 anchor + 清零速度，瞬移后从静止重新起步 |
| `get_location` / `get_speed` / `get_bearing` | 读静态字段（bearing 有副作用） | 从 `snapshot()` 读，值自洽且稳定 |

- 新增内部开关 `motionDriven`（默认开）；`stop` 时心跳归位静止。
- 不新增 app 必须感知的命令，老版本 app 仍能驱动（`move` 退化为"设目标"，心跳照常跑）。

## 5. 心跳线程（系统侧自驱）

复活 `LocationServiceHook.startDaemon`（当前被注释），重写为：

- 单后台守护线程，`FakeLoc.enable` 为 true 时按固定周期（取 `reportDuration`，默认 ~1s）tick：推进 `MotionState` → 调用 `callOnLocationChanged()` 主动推送给已注册监听器。
- `enable` 为 false 时长睡（如 3s）轮询，不空转耗电。
- tick 用 `SystemClock.elapsedRealtimeNanos` 算真实 `dt`。
- 附带解决现状问题：现在仅 `move` 命令到来才广播，松开摇杆静止时不再推位置，导致静止时部分 app 收不到更新。心跳让静止也持续上报（带真实漂移），更像真机。

## 6. GNSS / NMEA 派生（D 多源一致）

- **GNSS 卫星（`onSvStatusChanged`）**：卫星集合在一次定位会话内保持稳定（不再每帧全 `shuffle`）；方位角/仰角随 `snapshot` 时间缓慢演进并缓存上一帧，相邻帧小步插值；`usedInFix` 卫星数与 `accuracy` 关联（精度好→用得多）。
- **NMEA（`injectNMEA`）**：GGA/RMC 的经纬度、速度（RMC speed）、航向（RMC course）全部取自同一 `snapshot`，与 Location 对象逐字段一致。
- 检测方无论从 Location、卫星、还是 NMEA 任一通道交叉验证，得到的速度/航向/位置都一致。

## 7. 线程安全与测试

- **快照模式**：`MotionState` 内部状态加锁更新，对外暴露 `@Volatile var snapshot: MotionSnapshot`（不可变 data class）原子替换。热点读路径（`injectLocation`）只读 volatile 引用，零锁、零副作用。呼应近期提交对线程安全的关注。
- **纯函数可测**：运动学积分抽成纯函数 `step(state, targetSpeed, targetBearing, dt): newState`，不依赖 Android API/系统时钟。
- **测试（遵循"不跑真机"约定）**：JVM 单元测试覆盖——加速到目标速度的曲线、减速到 0、航向最短弧缓转、位移 = ∫speed dt 的一致性、瞬移后归零、抖动游走有界。大地测量推进复用现有 `moveLocation` 数学或 `Geodesic`。

## 8. 影响的文件（预估）

- 新增：`xposed/.../utils/MotionState.kt`（状态中心 + 纯函数 `step`）、`MotionSnapshot`（不可变快照）。
- 修改：`utils/FakeLoc.kt`（移除 bearing 副作用，坐标/速度/航向改由 MotionState 托管或委托）、`RemoteCommandHandler.kt`（命令语义）、`LocationServiceHook.kt`（复活心跳、GNSS 派生）、`BaseLocationHook.kt`（`injectLocation`/`injectNMEA` 改为读快照）、`hooks/BasicLocationHook.kt`（writeToParcel 路径同步）。
- app 侧：`MockServiceViewModel.kt`、`MockServiceHelper.kt`（`move` 改为下发目标，移除每 tick 推坐标的重复驱动）。
- 测试：新增 `xposed` 模块 JVM 单元测试。

## 9. 非目标（YAGNI）

- 不引入完整物理引擎或传感器（加速度计/陀螺仪）与运动联动（方案 C，过度设计）。
- 不重新设计 IPC 协议。
- 不改动厂商特定 Hook（MIUI/OPLUS）的现有逻辑，除非它们直接读取被改动的字段。
- 不处理 WiFi/基站模拟（当前 README 标记未完成，超出本次范围）。
