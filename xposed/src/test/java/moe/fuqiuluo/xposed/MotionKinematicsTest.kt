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
        val next = MotionKinematics.step(base(), targetSpeed = 10.0, targetBearing = 0.0, dtSeconds = 1.0, params = params, randomGaussian = { 0.0 })
        assertEquals(2.0, next.speed, 1e-6)
    }

    @Test
    fun speed_decelerates_to_zero_without_overshoot() {
        val next = MotionKinematics.step(base(speed = 1.0), targetSpeed = 0.0, targetBearing = 0.0, dtSeconds = 1.0, params = params, randomGaussian = { 0.0 })
        assertEquals(0.0, next.speed, 1e-6)
    }

    @Test
    fun bearing_turns_shortest_arc() {
        val next = MotionKinematics.step(base(speed = 5.0, bearing = 350.0), targetSpeed = 5.0, targetBearing = 10.0, dtSeconds = 0.1, params = params, randomGaussian = { 0.0 })
        assertEquals(359.0, next.bearing, 1e-6)
    }

    @Test
    fun bearing_stays_in_range() {
        val next = MotionKinematics.step(base(speed = 5.0, bearing = 359.0), targetSpeed = 5.0, targetBearing = 5.0, dtSeconds = 1.0, params = params, randomGaussian = { 0.0 })
        assertTrue(next.bearing >= 0.0 && next.bearing < 360.0)
    }

    @Test
    fun displacement_matches_speed_times_dt() {
        val start = base(speed = 5.0, bearing = 0.0)
        val next = MotionKinematics.step(start, targetSpeed = 5.0, targetBearing = 0.0, dtSeconds = 2.0, params = params, randomGaussian = { 0.0 })
        val dist = haversine(start.latitude, start.longitude, next.latitude, next.longitude)
        assertEquals(10.0, dist, 0.5)
    }

    @Test
    fun jitter_random_walk_is_bounded() {
        var snap = base(speed = 0.0)
        repeat(1000) {
            snap = MotionKinematics.step(snap, targetSpeed = 0.0, targetBearing = 0.0, dtSeconds = 1.0, params = params, randomGaussian = { 1.0 })
        }
        // OU 离散不动点 j* = volatility*sqrt(dt) / (reversion*dt) = 0.5/0.3 ≈ 1.667（dt=1.0），远小于发散
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
