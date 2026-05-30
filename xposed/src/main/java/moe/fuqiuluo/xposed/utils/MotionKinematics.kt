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
