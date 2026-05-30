# MotionState 运动状态中心 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 xposed 系统进程引入 `MotionState` 作为虚拟定位的唯一真相源，统一速度/航向/位移的演进，让 Location/GNSS/NMEA 全部从同一快照派生以保证多源自洽，并通过系统侧心跳自驱实现加减速曲线与静↔动平滑过渡。

**Architecture:** 新增不可变快照 `MotionSnapshot` + 纯函数运动学积分 `MotionKinematics.step()`（不依赖 Android API，可 JVM 单测）+ 状态容器 `MotionState`（加锁更新、`@Volatile` 暴露快照）。复活 `LocationServiceHook` 的守护心跳线程按真实 `dt` 推进状态并主动广播。各 Hook（`injectLocation`/`injectNMEA`/GNSS）改为读快照。命令语义改为"下发目标"，IPC 协议不变。app 侧 `move` 改为只设目标，移除每 tick 推坐标。

**Tech Stack:** Kotlin，LSPosed/Xposed API，JUnit 4.13.2（JVM 单元测试，`xposed/src/test`），GeographicLib（`net.sf.geographiclib.Geodesic`，app 侧已用）。

**设计依据：** `docs/superpowers/specs/2026-05-31-motion-state-design.md`

---

## 文件结构

**新增（xposed 模块）：**
- `xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionSnapshot.kt` — 不可变数据类，仅基本类型（lat/lon/alt/speed/bearing/accuracy/timeNanos），可在 JVM 测试中直接构造。
- `xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionKinematics.kt` — 纯函数对象：`step(snapshot, targetSpeed, targetBearing, dtSeconds, params)` 返回新 `MotionSnapshot`；含速度趋近、航向最短弧缓转、大地测量位移、相关随机游走抖动。无 Android 依赖。
- `xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionState.kt` — 状态容器：持有目标（targetSpeed/targetBearing）、`@Volatile var snapshot`、`tick(dtNanos)`、`teleport(lat,lon)`、`setTarget(...)`、`stop()`。加锁更新，读路径无锁。

**测试（xposed 模块）：**
- `xposed/src/test/java/moe/fuqiuluo/xposed/MotionKinematicsTest.kt` — 纯函数 JVM 单测。

**修改：**
- `utils/FakeLoc.kt` — 移除 `bearing` getter 副作用；坐标/速度/航向读取委托给 `MotionState.snapshot`。
- `RemoteCommandHandler.kt` — `move`/`set_speed`/`set_bearing` 改为设目标；`update_location` 改为瞬移；getter 命令读快照。
- `hooks/LocationServiceHook.kt` — 复活并重写 `startDaemon` 心跳；GNSS 卫星派生改为会话稳定 + 时间演进。
- `BaseLocationHook.kt` — `injectLocation`/`injectNMEA` 改为读快照。
- `hooks/BasicLocationHook.kt` — `writeToParcel` 路径读快照同步。
- app 侧 `MockServiceViewModel.kt`、`service/MockServiceHelper.kt` — `move` 改为下发目标，移除每 tick 推坐标。

---

## Task 1: MotionSnapshot 不可变快照

**Files:**
- Create: `xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionSnapshot.kt`

- [ ] **Step 1: 创建不可变数据类**

只用基本类型，不引用任何 Android 类，确保可在 JVM 测试中直接构造。

```kotlin
package moe.fuqiuluo.xposed.utils

/**
 * 运动状态的不可变快照，所有位置数据的唯一真相源。
 * 仅含基本类型，不依赖 Android API，可在纯 JVM 单元测试中构造。
 *
 * @param latitude 当前纬度（度）
 * @param longitude 当前经度（度）
 * @param altitude 海拔（米）
 * @param speed 当前地速（米/秒）
 * @param bearing 当前航向（度，[0,360)）
 * @param accuracy 水平精度（米）
 * @param jitterState 抖动随机游走的内部状态（米，带符号），供下一帧演进使用
 * @param elapsedRealtimeNanos 该快照对应的系统时刻（SystemClock.elapsedRealtimeNanos）
 */
data class MotionSnapshot(
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val speed: Double,
    val bearing: Double,
    val accuracy: Float,
    val jitterState: Double,
    val elapsedRealtimeNanos: Long,
)
```

- [ ] **Step 2: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionSnapshot.kt
git commit -m "feat(xposed): 新增 MotionSnapshot 不可变运动快照"
```

---

## Task 2: MotionKinematics 纯函数运动学（先写测试）

**Files:**
- Create: `xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionKinematics.kt`
- Test: `xposed/src/test/java/moe/fuqiuluo/xposed/MotionKinematicsTest.kt`

本任务遵循 TDD：先写失败测试，再写实现。

- [ ] **Step 1: 写失败测试**

覆盖：加速趋近、减速到 0、航向最短弧缓转、位移与速度一致、抖动有界。

```kotlin
package moe.fuqiuluo.xposed

import moe.fuqiuluo.xposed.utils.MotionKinematics
import moe.fuqiuluo.xposed.utils.MotionParams
import moe.fuqiuluo.xposed.utils.MotionSnapshot
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class MotionKinematicsTest {
    private val params = MotionParams(
        acceleration = 2.0,
        maxTurnRateDegPerSec = 90.0,
        jitterReversion = 0.3,
        jitterVolatility = 0.5,
    )

    private fun base(speed: Double = 0.0, bearing: Double = 0.0) = MotionSnapshot(
        latitude = 39.9, longitude = 116.4, altitude = 80.0,
        speed = speed, bearing = bearing, accuracy = 10.0f,
        jitterState = 0.0, elapsedRealtimeNanos = 0L,
    )

    @Test
    fun speed_accelerates_toward_target_not_instant() {
        // 目标 10 m/s，加速度 2 m/s²，一步 1s 后速度应约为 2，而非瞬间 10
        val next = MotionKinematics.step(base(), targetSpeed = 10.0, targetBearing = 0.0, dtSeconds = 1.0, params = params, randomGaussian = { 0.0 })
        assertEquals(2.0, next.speed, 1e-6)
    }

    @Test
    fun speed_decelerates_to_zero_without_overshoot() {
        // 当前 1 m/s，目标 0，加速度 2 m/s²，一步 1s 不应冲过 0 变负
        val next = MotionKinematics.step(base(speed = 1.0), targetSpeed = 0.0, targetBearing = 0.0, dtSeconds = 1.0, params = params, randomGaussian = { 0.0 })
        assertEquals(0.0, next.speed, 1e-6)
    }

    @Test
    fun bearing_turns_shortest_arc() {
        // 当前 350°，目标 10°：最短弧应朝 360/0 方向递增，转 +20° 上限内
        val next = MotionKinematics.step(base(speed = 5.0, bearing = 350.0), targetSpeed = 5.0, targetBearing = 10.0, dtSeconds = 0.1, params = params, randomGaussian = { 0.0 })
        // 0.1s * 90°/s = 9° 上限，350+9=359
        assertEquals(359.0, next.bearing, 1e-6)
    }

    @Test
    fun bearing_stays_in_range() {
        val next = MotionKinematics.step(base(speed = 5.0, bearing = 359.0), targetSpeed = 5.0, targetBearing = 5.0, dtSeconds = 1.0, params = params, randomGaussian = { 0.0 })
        assertTrue(next.bearing >= 0.0 && next.bearing < 360.0)
    }

    @Test
    fun displacement_matches_speed_times_dt() {
        // 5 m/s 正北走 2s，应位移约 10m。用 haversine 验证。
        val start = base(speed = 5.0, bearing = 0.0)
        val next = MotionKinematics.step(start, targetSpeed = 5.0, targetBearing = 0.0, dtSeconds = 2.0, params = params, randomGaussian = { 0.0 })
        val dist = haversine(start.latitude, start.longitude, next.latitude, next.longitude)
        assertEquals(10.0, dist, 0.5)
    }

    @Test
    fun jitter_random_walk_is_bounded() {
        // 静止下连续推进，抖动状态不应发散（高斯输入恒为 1.0 时趋于均衡）
        var snap = base(speed = 0.0)
        repeat(1000) {
            snap = MotionKinematics.step(snap, targetSpeed = 0.0, targetBearing = 0.0, dtSeconds = 0.1, params = params, randomGaussian = { 1.0 })
        }
        // 均衡值 = volatility/reversion 量级，远小于发散
        assertTrue("jitterState 不应发散: ${snap.jitterState}", abs(snap.jitterState) < 5.0)
    }

    private fun haversine(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val r = 6371000.0
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                Math.sin(dLon / 2) * Math.sin(dLon / 2)
        return r * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
    }
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `./gradlew :xposed:testDebugUnitTest --tests "moe.fuqiuluo.xposed.MotionKinematicsTest"`
Expected: 编译失败（`MotionKinematics`/`MotionParams` 未定义）

- [ ] **Step 3: 写实现**

创建 `MotionKinematics.kt`。`step` 是纯函数，`randomGaussian` 通过参数注入（测试传确定值，生产传 `Random.nextGaussian` 等价物）。大地测量推进沿用 spec 约定的球面公式（与现有 `FakeLoc.moveLocation` 同源）。

```kotlin
package moe.fuqiuluo.xposed.utils

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * 运动学参数。acceleration: m/s²；maxTurnRateDegPerSec: 最大角速度 °/s；
 * jitterReversion: 抖动回归系数（越大越快回到 0）；jitterVolatility: 抖动波动强度。
 */
data class MotionParams(
    val acceleration: Double = 2.0,
    val maxTurnRateDegPerSec: Double = 90.0,
    val jitterReversion: Double = 0.3,
    val jitterVolatility: Double = 0.5,
)

/**
 * 纯函数运动学积分。不依赖 Android API / 系统时钟，可在 JVM 单测中验证。
 */
object MotionKinematics {
    private const val EARTH_RADIUS = 6371000.0

    /**
     * 推进一步。
     * @param randomGaussian 注入的高斯随机源（均值0方差1）；测试可传常量。
     */
    fun step(
        current: MotionSnapshot,
        targetSpeed: Double,
        targetBearing: Double,
        dtSeconds: Double,
        params: MotionParams,
        randomGaussian: () -> Double,
    ): MotionSnapshot {
        if (dtSeconds <= 0.0) return current

        // 1. 速度趋近：朝 targetSpeed 逼近，步长 = acceleration * dt，不冲过目标
        val newSpeed = approach(current.speed, targetSpeed, params.acceleration * dtSeconds)

        // 2. 航向缓转：最短弧，限速 maxTurnRate * dt
        val newBearing = turnTowards(current.bearing, targetBearing, params.maxTurnRateDegPerSec * dtSeconds)

        // 3. 位移积分：distance = speed * dt，沿 newBearing 推进
        val distance = newSpeed * dtSeconds
        val (newLat, newLon) = advance(current.latitude, current.longitude, distance, newBearing)

        // 4. 抖动随机游走（Ornstein-Uhlenbeck 离散化）：
        //    next = prev + (-reversion*prev)*dt + volatility*sqrt(dt)*gauss
        val j = current.jitterState
        val newJitter = j + (-params.jitterReversion * j) * dtSeconds +
                params.jitterVolatility * Math.sqrt(dtSeconds) * randomGaussian()

        return current.copy(
            latitude = newLat,
            longitude = newLon,
            speed = newSpeed,
            bearing = newBearing,
            jitterState = newJitter,
            elapsedRealtimeNanos = current.elapsedRealtimeNanos + (dtSeconds * 1e9).toLong(),
        )
    }

    /** 朝 target 逼近，单步最大变化 maxDelta，不冲过。 */
    private fun approach(value: Double, target: Double, maxDelta: Double): Double {
        val diff = target - value
        if (kotlin.math.abs(diff) <= maxDelta) return target
        return value + if (diff > 0) maxDelta else -maxDelta
    }

    /** 航向走最短弧朝 target 旋转，单步最大 maxDeltaDeg，结果归一化到 [0,360)。 */
    fun turnTowards(current: Double, target: Double, maxDeltaDeg: Double): Double {
        var delta = (target - current) % 360.0
        if (delta > 180.0) delta -= 360.0
        if (delta < -180.0) delta += 360.0
        val applied = when {
            delta > maxDeltaDeg -> maxDeltaDeg
            delta < -maxDeltaDeg -> -maxDeltaDeg
            else -> delta
        }
        var result = (current + applied) % 360.0
        if (result < 0) result += 360.0
        return result
    }

    /** 沿 bearing 推进 distance 米（球面近似，与 FakeLoc.moveLocation 同源）。 */
    private fun advance(lat: Double, lon: Double, distance: Double, bearing: Double): Pair<Double, Double> {
        val radiusInDegrees = distance / EARTH_RADIUS * (180.0 / PI)
        val newLat = lat + radiusInDegrees * cos(Math.toRadians(bearing))
        val newLon = lon + radiusInDegrees * sin(Math.toRadians(bearing)) / cos(Math.toRadians(lat))
        return Pair(newLat, newLon)
    }
}
```

- [ ] **Step 4: 运行测试确认通过**

Run: `./gradlew :xposed:testDebugUnitTest --tests "moe.fuqiuluo.xposed.MotionKinematicsTest"`
Expected: PASS（6 个测试全绿）

- [ ] **Step 5: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionKinematics.kt xposed/src/test/java/moe/fuqiuluo/xposed/MotionKinematicsTest.kt
git commit -m "feat(xposed): 新增纯函数运动学积分 MotionKinematics 及单元测试"
```

---

## Task 3: MotionState 状态容器

**Files:**
- Create: `xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionState.kt`

状态容器：持有目标与当前快照，封装加锁更新与无锁读取。`tick` 由心跳调用，`teleport`/`setTarget`/`stop` 由命令调用。

- [ ] **Step 1: 创建 MotionState**

```kotlin
package moe.fuqiuluo.xposed.utils

import android.os.SystemClock
import java.util.Random

/**
 * 运动状态中心：虚拟定位的唯一真相源。
 * 写路径（tick/命令）加锁更新内部状态；读路径通过 @Volatile snapshot 无锁读取。
 */
object MotionState {
    private val lock = Any()
    private val random = Random()

    @Volatile
    var params = MotionParams()

    @Volatile
    private var targetSpeed: Double = 0.0
    @Volatile
    private var targetBearing: Double = 0.0

    /** 对外暴露的不可变快照，读路径只读此引用，零锁零副作用。 */
    @Volatile
    var snapshot: MotionSnapshot = MotionSnapshot(
        latitude = 0.0, longitude = 0.0, altitude = 80.0,
        speed = 0.0, bearing = 0.0, accuracy = 25.0f,
        jitterState = 0.0, elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos(),
    )
        private set

    /** 设置运动目标（move/set_speed/set_bearing 调用）。 */
    fun setTarget(speed: Double = targetSpeed, bearing: Double = targetBearing) {
        synchronized(lock) {
            targetSpeed = speed
            targetBearing = bearing
        }
    }

    /** 瞬移：硬重置锚点并清零速度（update_location 调用）。 */
    fun teleport(latitude: Double, longitude: Double, altitude: Double = snapshot.altitude, accuracy: Float = snapshot.accuracy) {
        synchronized(lock) {
            targetSpeed = 0.0
            snapshot = snapshot.copy(
                latitude = latitude, longitude = longitude, altitude = altitude,
                speed = 0.0, accuracy = accuracy,
                elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos(),
            )
        }
    }

    /** 停止：清零目标与当前速度，保持锚点（stop 调用）。 */
    fun stop() {
        synchronized(lock) {
            targetSpeed = 0.0
            snapshot = snapshot.copy(speed = 0.0, elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos())
        }
    }

    /** 更新精度/海拔等不参与积分的字段。 */
    fun updateMeta(altitude: Double = snapshot.altitude, accuracy: Float = snapshot.accuracy) {
        synchronized(lock) {
            snapshot = snapshot.copy(altitude = altitude, accuracy = accuracy)
        }
    }

    /** 心跳推进：用真实 dt 演进一步并原子替换快照。 */
    fun tick(nowNanos: Long = SystemClock.elapsedRealtimeNanos()) {
        synchronized(lock) {
            val dtSeconds = (nowNanos - snapshot.elapsedRealtimeNanos) / 1e9
            if (dtSeconds <= 0.0) return
            val next = MotionKinematics.step(
                current = snapshot,
                targetSpeed = targetSpeed,
                targetBearing = targetBearing,
                dtSeconds = dtSeconds,
                params = params,
                randomGaussian = { random.nextGaussian() },
            )
            snapshot = next.copy(elapsedRealtimeNanos = nowNanos)
        }
    }
}
```

- [ ] **Step 2: 编译确认**

Run: `./gradlew :xposed:compileDebugSources`
Expected: BUILD SUCCESSFUL（MotionState 引用的 MotionKinematics/MotionSnapshot/MotionParams 均已存在）

- [ ] **Step 3: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionState.kt
git commit -m "feat(xposed): 新增 MotionState 运动状态中心（加锁更新+快照读取）"
```

---

## Task 4: FakeLoc 委托 MotionState（移除 bearing 副作用）

**Files:**
- Modify: `xposed/src/main/java/moe/fuqiuluo/xposed/utils/FakeLoc.kt`

`FakeLoc.latitude/longitude/speed/bearing` 改为从 `MotionState.snapshot` 读取的计算属性；写入时转交 `MotionState`。这样所有旧调用点（`get_location` 等）自动读到自洽快照，且消除 bearing getter 每读自增 0.5° 的副作用。

- [ ] **Step 1: 替换坐标/速度/航向字段为委托属性**

将 FakeLoc.kt 中现有的这段（约 99-131 行）：

```kotlin
    @Volatile var lastLocation: Location? = null
    @Volatile var latitude = 0.0
    @Volatile var longitude = 0.0
    @Volatile var altitude = 80.0

    @Volatile var speed = 3.05

    @Volatile var speedAmplitude = 1.0

    @Volatile var hasBearings = false

    @Volatile var bearing = 0.0
        @Synchronized get() {
            if (hasBearings) {
                return field
            } else {
                if (field >= 360.0) {
                    field -= 360.0
                }
                field += 0.5
                return field
            }
        }
        @Synchronized set

    @Volatile var accuracy = 25.0f
        set(value) {
            field = if (value < 0) {
                -value
            } else {
                value
            }
        }
```

替换为（坐标/速度/航向委托 MotionState；altitude/accuracy 仍由 FakeLoc 持有但同步进 MotionState）：

```kotlin
    @Volatile var lastLocation: Location? = null

    // 坐标/速度/航向的唯一真相源是 MotionState.snapshot。
    // 这些属性保留是为兼容旧调用点（读快照、写转交 MotionState）。
    var latitude: Double
        get() = MotionState.snapshot.latitude
        set(value) { MotionState.teleport(value, MotionState.snapshot.longitude) }
    var longitude: Double
        get() = MotionState.snapshot.longitude
        set(value) { MotionState.teleport(MotionState.snapshot.latitude, value) }

    @Volatile var altitude = 80.0
        set(value) {
            field = value
            MotionState.updateMeta(altitude = value)
        }

    /** 目标速度（m/s）。读返回当前快照地速，写设为运动目标。 */
    var speed: Double
        get() = MotionState.snapshot.speed
        set(value) { MotionState.setTarget(speed = value) }

    @Volatile var speedAmplitude = 1.0

    // 保留字段以兼容旧引用；航向真相源已是 MotionState，副作用自增已移除。
    @Volatile var hasBearings = false

    /** 航向（度）。读返回当前快照航向，写设为缓转目标。 */
    var bearing: Double
        get() = MotionState.snapshot.bearing
        set(value) { MotionState.setTarget(bearing = value) }

    @Volatile var accuracy = 25.0f
        set(value) {
            field = if (value < 0) -value else value
            MotionState.updateMeta(accuracy = field)
        }
```

注意：删除 `import android.location.Location` 如果不再被其他代码使用则保留（`lastLocation` 仍用 Location，保留 import）。

- [ ] **Step 2: 编译确认**

Run: `./gradlew :xposed:compileDebugSources`
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/utils/FakeLoc.kt
git commit -m "refactor(xposed): FakeLoc 坐标/速度/航向委托 MotionState，移除 bearing 自增副作用"
```

---

## Task 5: RemoteCommandHandler 命令语义改为下发目标

**Files:**
- Modify: `xposed/src/main/java/moe/fuqiuluo/xposed/RemoteCommandHandler.kt`

按 spec 第 4 节改命令语义：`move`/`set_speed`/`set_bearing` 设目标，`update_location` 瞬移，getter 读快照。命令名与 Bundle 协议不变。

- [ ] **Step 1: 改 move 命令（约 166-182 行）**

将现有 `"move"` 分支：

```kotlin
            "move" -> {
                val distance = rely.getDouble("n", 0.0)
                if (distance == 0.0) return true
                val bearing = rely.getDouble("bearing", 0.0)
                val newLoc = FakeLoc.moveLocation(
                    n = distance,
                    angle = bearing
                )
                if (FakeLoc.enableDebugLog) {
                    Logger.debug("move: distance=$distance, bearing=$bearing, newLoc=$newLoc")
                }
                FakeLoc.bearing = bearing
                FakeLoc.hasBearings = true
                return updateCoordinate(newLoc.first, newLoc.second).also {
                    if (FakeLoc.isSystemServerProcess) LocationServiceHook.callOnLocationChanged()
                }
            }
```

替换为（distance>0 → 设目标速度=FakeLoc.speed + 目标航向；distance==0 → 停止。不再搬坐标、不再主动广播，位移交给心跳）：

```kotlin
            "move" -> {
                val distance = rely.getDouble("n", 0.0)
                val bearing = rely.getDouble("bearing", 0.0)
                if (FakeLoc.enableDebugLog) {
                    Logger.debug("move: distance=$distance, bearing=$bearing (设目标，位移交心跳)")
                }
                if (distance == 0.0) {
                    // 松摇杆/到点：目标速度归零，心跳平滑减速到静止
                    MotionState.setTarget(speed = 0.0, bearing = bearing)
                } else {
                    // 目标速度取已同步的 FakeLoc.speed（由 start/set_speed 设置），不反推 dt
                    MotionState.setTarget(speed = FakeLoc.speed, bearing = bearing)
                }
                return true
            }
```

注意：`FakeLoc.speed` 的 getter 现在返回快照速度（Task 4），而目标速度需要的是"用户设定的巡航速度"。因此需要在 MotionState 单独保存巡航速度——见 Step 1b。

- [ ] **Step 1b: MotionState 增加巡航速度字段**

`move` 需要"用户设定速度"而非"当前速度"。在 `MotionState`（Task 3 文件）中新增：

```kotlin
    /** 用户设定的巡航速度（m/s），由 set_speed/start 设置，move 用它作为目标速度。 */
    @Volatile
    var cruiseSpeed: Double = 0.0
        private set

    fun setCruiseSpeed(speed: Double) {
        synchronized(lock) { cruiseSpeed = speed }
    }
```

并把 Step 1 的 `move` 分支里 `MotionState.setTarget(speed = FakeLoc.speed, ...)` 改为 `MotionState.setTarget(speed = MotionState.cruiseSpeed, ...)`。

- [ ] **Step 2: 改 set_speed / set_bearing 分支（约 155-165 行）**

```kotlin
            "set_speed" -> {
                val speed = rely.getDouble("speed", 0.0)
                MotionState.setCruiseSpeed(speed)
                return true
            }
            "set_bearing" -> {
                val bearing = rely.getDouble("bearing", 0.0)
                MotionState.setTarget(bearing = bearing)
                FakeLoc.hasBearings = true
                return true
            }
```

- [ ] **Step 3: 改 start 分支同步巡航速度（约 72-87 行）**

在 `"start"` 分支内，`FakeLoc.speed = speed` 一行之后补一行 `MotionState.setCruiseSpeed(speed)`：

```kotlin
                FakeLoc.speed = speed
                MotionState.setCruiseSpeed(speed)
                FakeLoc.altitude = altitude
                FakeLoc.accuracy = accuracy
```

- [ ] **Step 4: 改 update_location 为瞬移（约 183-219 行）**

`updateCoordinate(newLat, newLon)` 现在通过 FakeLoc.latitude setter 已转交 `MotionState.teleport`（Task 4），但为语义明确，直接改 `updateCoordinate`（约 328-337 行）：

```kotlin
    private fun updateCoordinate(newLat: Double, newLon: Double): Boolean {
        if (newLat in -90.0..90.0 && newLon in -180.0..180.0) {
            MotionState.teleport(newLat, newLon)
            return true
        } else {
            Logger.error("Invalid latitude or longitude: $newLat, $newLon")
            return false
        }
    }
```

`update_location` 分支主体逻辑（`+`/`-`/`*`/`/`/`=`/`random` 的坐标运算）不变，它们最终都调 `updateCoordinate`。

- [ ] **Step 5: 改 stop 分支（约 88-95 行）**

在 `"stop"` 分支内 `FakeLoc.enable = false` 之后补 `MotionState.stop()`：

```kotlin
            "stop" -> {
                FakeLoc.enable = false
                FakeLoc.hasBearings = false
                MotionState.stop()
                if (isLoadedLibrary) {
                    Dobby.setStatus(false)
                }
                return true
            }
```

- [ ] **Step 6: 添加 import**

文件顶部 import 区补充：

```kotlin
import moe.fuqiuluo.xposed.utils.MotionState
```

- [ ] **Step 7: 编译确认**

Run: `./gradlew :xposed:compileDebugSources`
Expected: BUILD SUCCESSFUL

- [ ] **Step 8: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/RemoteCommandHandler.kt xposed/src/main/java/moe/fuqiuluo/xposed/utils/MotionState.kt
git commit -m "feat(xposed): 命令语义改为下发目标，update_location 改为瞬移"
```

---

## Task 6: 复活心跳线程 startDaemon（系统侧自驱）

**Files:**
- Modify: `xposed/src/main/java/moe/fuqiuluo/xposed/hooks/LocationServiceHook.kt`

复活当前被注释的 `startDaemon`（约 867-894 行），重写为推进 `MotionState` + 主动广播。在 `invoke` 里启用调用（约 170 行 `//startDaemon(classLoader)`）。

- [ ] **Step 1: 启用 startDaemon 调用**

将 `invoke` 方法中（约 170 行）的：

```kotlin
        //startDaemon(classLoader)
```

改为：

```kotlin
        startDaemon()
```

- [ ] **Step 2: 用新实现替换被注释的 startDaemon（约 867-894 行）**

删除整段被注释的 `//private fun startDaemon(...)`，替换为：

```kotlin
    private val daemonStarted = AtomicBoolean(false)

    private fun startDaemon() {
        if (!daemonStarted.compareAndSet(false, true)) return
        thread(name = "PortalMotionDaemon", isDaemon = true, start = true) {
            while (true) {
                kotlin.runCatching {
                    if (!FakeLoc.enable) {
                        Thread.sleep(3000)
                        return@runCatching
                    }
                    // 推进运动状态一步（真实 dt 由 MotionState 内部用 elapsedRealtimeNanos 计算）
                    MotionState.tick()
                    // 主动广播给已注册监听器（静止也持续上报，带真实漂移）
                    callOnLocationChanged()
                    Thread.sleep(FakeLoc.reportInterval)
                }.onFailure {
                    Logger.error("PortalMotionDaemon", it)
                }
            }
        }
    }
```

- [ ] **Step 3: 添加 import 与 FakeLoc.reportInterval**

在 `LocationServiceHook.kt` 顶部 import 区补充：

```kotlin
import moe.fuqiuluo.xposed.utils.MotionState
import kotlin.concurrent.thread
```

（`AtomicBoolean` 已 import；确认 `java.util.concurrent.atomic.AtomicBoolean` 在 import 列表中——文件已有）

在 `FakeLoc.kt` 中新增心跳周期字段（系统侧自驱周期，默认 1000ms；不依赖 app 的 reportDuration）：

```kotlin
    /**
     * 系统侧心跳周期（毫秒）。MotionState 每隔此时长推进一步并广播。
     */
    @Volatile var reportInterval: Long = 1000L
```

- [ ] **Step 4: 编译确认**

Run: `./gradlew :xposed:compileDebugSources`
Expected: BUILD SUCCESSFUL

- [ ] **Step 5: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/hooks/LocationServiceHook.kt xposed/src/main/java/moe/fuqiuluo/xposed/utils/FakeLoc.kt
git commit -m "feat(xposed): 复活 startDaemon 心跳线程，按真实 dt 推进 MotionState 并广播"
```

---

## Task 7: injectLocation / injectNMEA 改为读快照

**Files:**
- Modify: `xposed/src/main/java/moe/fuqiuluo/xposed/BaseLocationHook.kt`

`injectLocation` 的 speed/bearing/坐标改为读 `MotionState.snapshot`，使三者自洽；`injectNMEA` 的速度/航向也取自同一快照。

- [ ] **Step 1: injectLocation 用快照填充（修改约 40-72 行）**

将现有坐标/速度/航向填充段：

```kotlin
        val location = Location(originLocation.provider ?: LocationManager.GPS_PROVIDER)
        location.accuracy = if (FakeLoc.accuracy != 0.0f) FakeLoc.accuracy else originLocation.accuracy
        val jitterLat = FakeLoc.jitterLocation()
        location.latitude = jitterLat.first
        location.longitude = jitterLat.second
        location.altitude = FakeLoc.altitude
        val speedAmp = Random.nextDouble(-FakeLoc.speedAmplitude, FakeLoc.speedAmplitude)
        location.speed = (FakeLoc.speed + speedAmp).toFloat()
```

替换为（一次性取快照，坐标用快照锚点叠加抖动游走，speed/bearing 直接取快照）：

```kotlin
        val snap = MotionState.snapshot
        val location = Location(originLocation.provider ?: LocationManager.GPS_PROVIDER)
        location.accuracy = if (FakeLoc.accuracy != 0.0f) FakeLoc.accuracy else originLocation.accuracy
        // 抖动叠加在快照锚点上：jitterState 已是相关随机游走结果（米）
        val jitterLat = FakeLoc.jitterLocation(lat = snap.latitude, lon = snap.longitude, n = kotlin.math.abs(snap.jitterState), angle = snap.bearing)
        location.latitude = jitterLat.first
        location.longitude = jitterLat.second
        location.altitude = FakeLoc.altitude
        location.speed = snap.speed.toFloat()
```

- [ ] **Step 2: injectLocation 航向用快照（修改约 60-64 行）**

将：

```kotlin
        var modBearing = FakeLoc.bearing % 360.0 + 0.0
        if (modBearing < 0) {
            modBearing += 360.0
        }
        location.bearing = modBearing.toFloat()
```

替换为：

```kotlin
        location.bearing = snap.bearing.toFloat()
```

（`snap.bearing` 已由 MotionKinematics 归一化到 [0,360)）

- [ ] **Step 3: 移除 speed<=0 兜底（约 70-72 行）**

删除以下段（静止时 speed=0 是合法状态，不应强行抬到 FakeLoc.speed）：

```kotlin
        if (location.speed <= 0.0f) {
            location.speed = FakeLoc.speed.toFloat()
        }
```

- [ ] **Step 4: injectNMEA 经纬度用快照（修改约 145-159、172-219 行的 FakeLoc.latitude/longitude 引用）**

`injectNMEA` 中所有 `FakeLoc.latitude`、`FakeLoc.longitude` 引用改为先取一次快照。在 `injectNMEA` 方法体开头（`if (!FakeLoc.enable)` 之后）加：

```kotlin
        val snap = MotionState.snapshot
```

然后把方法内所有 `FakeLoc.latitude` → `snap.latitude`，`FakeLoc.longitude` → `snap.longitude`。RMC 分支额外设置速度与航向（若 NmeaValue.RMC 暴露这些字段；查 nmea 模块 RMC 定义，存在则赋值，不存在则跳过并在计划注记）：

```kotlin
                is NmeaValue.RMC -> {
                    // ...现有经纬度赋值改为 snap.latitude/snap.longitude...
                    // 速度(节)与航向取自同一快照，与 Location 自洽
                    // value.speedKnots = snap.speed * 1.943844  // 若字段存在
                    // value.course = snap.bearing               // 若字段存在
                    return value.toNmeaString()
                }
```

- [ ] **Step 5: 添加 import**

`BaseLocationHook.kt` 顶部补充：

```kotlin
import moe.fuqiuluo.xposed.utils.MotionState
```

- [ ] **Step 6: 编译确认**

Run: `./gradlew :xposed:compileDebugSources`
Expected: BUILD SUCCESSFUL

- [ ] **Step 7: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/BaseLocationHook.kt
git commit -m "feat(xposed): injectLocation/injectNMEA 改为读 MotionState 快照，速度航向位置自洽"
```

---

## Task 8: GNSS 卫星派生改为会话稳定 + 时间演进

**Files:**
- Modify: `xposed/src/main/java/moe/fuqiuluo/xposed/hooks/LocationServiceHook.kt`

当前 `onSvStatusChanged` 每帧 `satelliteList.shuffled().take(svCount)` 全随机，且方位角/仰角 `Random.nextFloat` 乱跳。改为：卫星集合在一次会话内固定缓存，方位角/仰角随时间小步演进。

- [ ] **Step 1: 新增会话级卫星缓存状态**

在 `LocationServiceHook` 对象内（靠近 `locationListeners` 声明处）新增：

```kotlin
    // GNSS 会话级缓存：卫星集合在一次定位会话内稳定，方位/仰角随时间小步演进
    private data class SvState(
        val sat: BDSSatellite,
        var cn0: Float,
        var elevation: Float,
        var azimuth: Float,
        val usedInFix: Boolean,
        val hasEphemeris: Boolean,
        val hasAlmanac: Boolean,
        val carrierFreq: Float,
    )
    @Volatile private var svSession: List<SvState>? = null
    @Volatile private var svSessionCount: Int = 0
```

- [ ] **Step 2: 提取卫星会话构建/演进函数**

在 `LocationServiceHook` 内新增（构建一次、之后只演进 azimuth/elevation/cn0）：

```kotlin
    private fun ensureSvSession(): List<SvState> {
        svSession?.let { return it }
        val count = Random.nextInt(FakeLoc.minSatellites, MAX_SATELLITES + 1)
        val accuracy = MotionState.snapshot.accuracy
        // 精度好→用于定位的卫星比例更高
        val usedRatio = (1.0f - (accuracy / 50f)).coerceIn(0.4f, 0.9f)
        val session = satelliteList.shuffled().take(count).map { sat ->
            SvState(
                sat = sat,
                cn0 = when (sat.type) {
                    is OrbitType.GEO -> Random.nextFloat(GEO_MIN_CN0, GEO_MAX_CN0)
                    is OrbitType.IGSO -> Random.nextFloat(IGSO_MIN_CN0, IGSO_MAX_CN0)
                    is OrbitType.MEO -> Random.nextFloat(MEO_MIN_CN0, MEO_MAX_CN0)
                },
                elevation = Random.nextFloat(sat.type.elevationRange.start, sat.type.elevationRange.endInclusive),
                azimuth = Random.nextFloat(0f, 360f),
                usedInFix = Random.nextFloat() < usedRatio,
                hasEphemeris = Random.nextFloat() > 0.1f,
                hasAlmanac = Random.nextFloat() > 0.05f,
                carrierFreq = when (Random.nextInt(3)) {
                    0 -> BDS_B1I_FREQ; 1 -> BDS_B2I_FREQ; else -> BDS_B3I_FREQ
                },
            )
        }
        svSession = session
        svSessionCount = count
        return session
    }

    private fun evolveSvSession(session: List<SvState>) {
        // 方位/仰角随时间小步漂移，相邻帧连续；cn0 小幅波动
        session.forEach { sv ->
            sv.azimuth = ((sv.azimuth + Random.nextFloat(-1.5f, 1.5f)) % 360f + 360f) % 360f
            sv.elevation = (sv.elevation + Random.nextFloat(-0.5f, 0.5f))
                .coerceIn(sv.sat.type.elevationRange.start, sv.sat.type.elevationRange.endInclusive)
            sv.cn0 = (sv.cn0 + Random.nextFloat(-0.8f, 0.8f))
                .coerceIn(sv.sat.type.minCn0, sv.sat.type.maxCn0)
        }
    }
```

注意：`OrbitType` 的 `minCn0`/`maxCn0`/`elevationRange` 已是 public（见 LocationServiceHook.kt 现有定义），可直接访问。

- [ ] **Step 3: 在 onSvStatusChanged 回调里改用会话**

将回调中现有的 `val svCount = Random.nextInt(...)` 到 `MockGnssData(...).apply { ... }` 整段（约 488-535 行）替换为：

```kotlin
                        if (!FakeLoc.enableMockGnss) return@beforeHook

                        val session = ensureSvSession().also { evolveSvSession(it) }
                        val svCount = session.size
                        val mockGps = MockGnssData(
                            svCount = svCount,
                            svidWithFlags = IntArray(svCount),
                            cn0s = FloatArray(svCount),
                            elevations = FloatArray(svCount),
                            azimuths = FloatArray(svCount),
                            carrierFreqs = FloatArray(svCount)
                        ).apply {
                            session.forEachIndexed { index, sv ->
                                var flags = GnssFlags.SVID_FLAGS_NONE
                                if (sv.hasEphemeris) flags = flags or GnssFlags.SVID_FLAGS_HAS_EPHEMERIS_DATA
                                if (sv.hasAlmanac) flags = flags or GnssFlags.SVID_FLAGS_HAS_ALMANAC_DATA
                                if (sv.usedInFix) flags = flags or GnssFlags.SVID_FLAGS_USED_IN_FIX
                                flags = flags or GnssFlags.SVID_FLAGS_HAS_CARRIER_FREQUENCY
                                flags = flags or GnssFlags.SVID_FLAGS_HAS_BASEBAND_CN0
                                svidWithFlags[index] = (sv.sat.prn shl GnssFlags.SVID_SHIFT_WIDTH) or
                                        ((GnssFlags.CONSTELLATION_BEIDOU and GnssFlags.CONSTELLATION_TYPE_MASK) shl GnssFlags.CONSTELLATION_TYPE_SHIFT_WIDTH) or
                                        flags
                                cn0s[index] = sv.cn0
                                elevations[index] = sv.elevation
                                azimuths[index] = sv.azimuth
                                carrierFreqs[index] = sv.carrierFreq
                            }
                        }
```

回调后续把 `mockGps` 写回 `args` 的逻辑（约 537-582 行）保持不变。

- [ ] **Step 4: 会话失效——stop 时清空缓存**

在 `RemoteCommandHandler` 的 `"stop"` 分支（Task 5 已改）中，`MotionState.stop()` 之后补一行清空卫星会话（通过 LocationServiceHook 暴露的方法）。先在 `LocationServiceHook` 增加：

```kotlin
    fun resetGnssSession() {
        svSession = null
        svSessionCount = 0
    }
```

再在 `RemoteCommandHandler` 的 `"stop"` 分支加 `LocationServiceHook.resetGnssSession()`。

- [ ] **Step 5: 编译确认**

Run: `./gradlew :xposed:compileDebugSources`
Expected: BUILD SUCCESSFUL

- [ ] **Step 6: 提交**

```bash
git add xposed/src/main/java/moe/fuqiuluo/xposed/hooks/LocationServiceHook.kt xposed/src/main/java/moe/fuqiuluo/xposed/RemoteCommandHandler.kt
git commit -m "feat(xposed): GNSS 卫星集合会话稳定，方位/仰角随时间演进，usedInFix 关联精度"
```

---

## Task 9: app 侧 move 改为下发目标，移除每 tick 推坐标

**Files:**
- Modify: `app/src/main/java/moe/fuqiuluo/portal/ui/viewmodel/MockServiceViewModel.kt`
- Modify: `app/src/main/java/moe/fuqiuluo/portal/service/MockServiceHelper.kt`

系统侧心跳现在自驱推进位移，app 侧不应再每 tick 推坐标（否则双重驱动、速度翻倍）。摇杆改为：方向变化时下发一次 `set_bearing` + 速度（巡航速度），松开时下发停止。路线模拟改为按段下发目标航向与速度。

- [ ] **Step 1: 摇杆循环改为仅在方向/速度变化时下发目标**

将 `initRocker` 中 rockerJob 的循环体（约 58-74 行）：

```kotlin
            rockerJob = GlobalScope.launch {
                do {
                    rockerCoroutineController.controlledCoroutine()
                    delay(delayTime)

                    CrashReport.setUserSceneTag(applicationContext, 261773)
                    if(!MockServiceHelper.move(locationManager!!, FakeLoc.speed / (1000 / delayTime) / 0.85, FakeLoc.bearing)) {
                        Log.e("MockServiceViewModel", "Failed to move")
                    }
                } while (isActive)
            }
```

替换为（仅下发"目标速度+航向"，distance 字段沿用 move 协议但语义已是"非0即移动"；不再每 tick 计算位移）：

```kotlin
            rockerJob = GlobalScope.launch {
                var lastBearing = Double.NaN
                var lastMoving = false
                do {
                    rockerCoroutineController.controlledCoroutine()
                    delay(delayTime)

                    CrashReport.setUserSceneTag(applicationContext, 261773)
                    val moving = FakeLoc.speed > 0.0
                    val bearing = FakeLoc.bearing
                    // 仅在航向或运动状态变化时下发目标，避免无谓 IPC
                    if (moving != lastMoving || bearing != lastBearing) {
                        // distance 传 1.0 表示"移动"、0.0 表示"停止"，系统侧据此设目标速度
                        val distance = if (moving) 1.0 else 0.0
                        if (!MockServiceHelper.move(locationManager!!, distance, bearing)) {
                            Log.e("MockServiceViewModel", "Failed to set move target")
                        }
                        lastMoving = moving
                        lastBearing = bearing
                    }
                } while (isActive)
            }
```

注意：`FakeLoc.bearing`/`FakeLoc.speed` 在 app 进程读的是 app 侧 FakeLoc 静态值（摇杆写入），与系统侧 MotionState 无关，仍可正常读取。

- [ ] **Step 2: 路线模拟改为下发段目标（修改约 99-170 行）**

路线循环里现有的两处 `MockServiceHelper.move(..., FakeLoc.speed / (1000 / delayTime) / 0.85, azimuth)` 与基于 `getLocation` 反复推进的逻辑，简化为：到达判定仍用 `getLocation`（读系统侧快照），但移动指令改为下发"目标航向 + 移动"。将段移动调用：

```kotlin
                    if (!MockServiceHelper.move(
                            locationManager!!,
                            FakeLoc.speed / (1000 / delayTime) / 0.85,
                            azimuth
                        )
                    ) {
                        Log.e("MockServiceViewModel", "移动失败")
                    }
```

替换为：

```kotlin
                    // 下发目标航向 + 移动，位移由系统侧心跳推进
                    if (!MockServiceHelper.move(locationManager!!, 1.0, azimuth)) {
                        Log.e("MockServiceViewModel", "移动失败")
                    }
```

到点判定阈值 `inverse.s12 < 1.0`（精确到点）与 `inverse.s12 < FakeLoc.speed/(1000/delayTime)/0.85`（跨段）保持不变——它们读 `getLocation` 的系统侧快照，随心跳推进会自然减小。

- [ ] **Step 3: MockServiceHelper.move 注释更新（约 264-276 行）**

`move` 方法签名与 Bundle 不变（IPC 协议兼容），仅更新文档注释说明语义已改为"下发目标"：

```kotlin
    /**
     * 下发运动目标到系统侧 MotionState。
     * @param distance >0 表示移动（目标速度取系统侧巡航速度），0 表示停止。
     * @param bearing 目标航向（度），系统侧缓转至此方向。
     * 实际位移由系统侧心跳按 dt 推进，本调用不再直接搬运坐标。
     */
```

- [ ] **Step 4: 编译确认**

Run: `./gradlew :app:compileDebugSources`
Expected: BUILD SUCCESSFUL

- [ ] **Step 5: 提交**

```bash
git add app/src/main/java/moe/fuqiuluo/portal/ui/viewmodel/MockServiceViewModel.kt app/src/main/java/moe/fuqiuluo/portal/service/MockServiceHelper.kt
git commit -m "feat(app): move 改为下发运动目标，移除每 tick 推坐标的重复驱动"
```

---

## Task 10: 全链路编译与单测回归

**Files:** 无新增，仅验证。

- [ ] **Step 1: 全模块编译**

Run: `./gradlew :xposed:compileDebugSources :app:compileDebugSources`
Expected: BUILD SUCCESSFUL

- [ ] **Step 2: 运行 xposed 单测**

Run: `./gradlew :xposed:testDebugUnitTest`
Expected: PASS（含 MotionKinematicsTest 6 个测试）

- [ ] **Step 3: 静态自检清单（不跑真机，遵循 CLAUDE.md 第 13 条）**

逐项确认：
- `FakeLoc.bearing` 已无自增副作用（grep 确认无 `field += 0.5`）
- `RemoteCommandHandler` 内 `move` 不再调 `callOnLocationChanged`（位移交心跳）
- `injectLocation`/`injectNMEA`/GNSS 三处均读同一 `MotionState.snapshot`
- 心跳线程仅启动一次（`AtomicBoolean` 守卫）

- [ ] **Step 4: 无新增提交（验证任务）**

若前序任务均已提交，本任务无代码变更；如自检发现遗漏，补修后单独提交。

---

## Self-Review 结论

- **Spec 覆盖**：spec 第 2 节架构→Task 1/3；第 3 节运动学→Task 2；bearing 副作用移除→Task 4；第 4 节命令语义→Task 5；第 5 节心跳→Task 6；Location 派生→Task 7；GNSS 派生→Task 8；NMEA 派生→Task 7 Step 4；app 侧→Task 9；测试→Task 2 + Task 10。全部有对应任务。
- **类型一致性**：`MotionSnapshot`/`MotionParams`/`MotionKinematics.step`/`MotionState.setTarget/teleport/stop/tick/setCruiseSpeed/cruiseSpeed/resetGnssSession` 在定义与引用处签名一致。
- **已知风险注记**：Task 7 Step 4 的 RMC speed/course 字段赋值取决于 nmea 模块 `NmeaValue.RMC` 是否暴露可写字段——执行时需先查 `nmea` 模块定义，存在则赋值，不存在则仅同步经纬度并在提交信息注明。`FakeLoc.jitterLocation` 的参数签名需在 Task 7 执行时核对（现有定义可能无参，需相应调整调用或重载）。
