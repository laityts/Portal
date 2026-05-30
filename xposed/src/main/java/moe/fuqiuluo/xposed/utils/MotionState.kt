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
