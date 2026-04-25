// 文件名: BaseLocationHook.kt
// 移除坐标跳跃距离限制，仅保留自然抖动和速度平滑
package moe.fuqiuluo.xposed

import android.location.Location
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.SystemClock
import de.robv.android.xposed.XposedHelpers
import moe.fuqiuluo.xposed.utils.FakeLoc
import moe.fuqiuluo.xposed.utils.Logger
import moe.microbios.nmea.NMEA
import moe.microbios.nmea.NmeaValue
import kotlin.math.sin
import kotlin.random.Random

abstract class BaseLocationHook: BaseDivineService() {
    // 用于平滑速度变化的低通滤波器系数 (0 < alpha <= 1)
    companion object {
        private var lastFilteredSpeed = FakeLoc.speed
        private var lastSpeedUpdateTime = 0L
        private const val SPEED_ALPHA = 0.6f  // 滤波系数，越接近1响应越快，越小越平滑
    }

    fun injectLocation(originLocation: Location, realLocation: Boolean = true): Location {
        if (realLocation) {
            if (if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    originLocation.provider == LocationManager.GPS_PROVIDER && originLocation.isComplete
                } else {
                    originLocation.provider == LocationManager.GPS_PROVIDER
                }
            ) {
                FakeLoc.lastLocation = originLocation
                // 修复时间戳冻结：更新 lastLocation 的时间为当前系统时间
                FakeLoc.lastLocation?.apply {
                    time = System.currentTimeMillis()
                    elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
                }
            }
        } else {
            originLocation.altitude = FakeLoc.altitude
        }
    
        if (!FakeLoc.enable)
            return originLocation
    
        if (originLocation.latitude + originLocation.longitude == FakeLoc.latitude + FakeLoc.longitude) {
            // Already processed
            return originLocation
        }
    
        if (FakeLoc.disableNetworkLocation && originLocation.provider == LocationManager.NETWORK_PROVIDER) {
            originLocation.provider = LocationManager.GPS_PROVIDER
        }
    
        val location = Location(originLocation.provider ?: LocationManager.GPS_PROVIDER)
        location.accuracy = if (FakeLoc.accuracy != 0.0f) FakeLoc.accuracy else originLocation.accuracy
        
        // ========== 修复坐标过于平滑：使用增强的抖动函数 ==========
        // 获取抖动后的坐标（基于理想位置），不再限制单步移动距离
        val jittered = FakeLoc.jitterLocation(lat = FakeLoc.latitude, lon = FakeLoc.longitude, angle = FakeLoc.bearing)
        val newLat = jittered.first
        val newLon = jittered.second
        // ===============================================================
        
        location.latitude = newLat
        location.longitude = newLon
        location.altitude = FakeLoc.altitude
        
        // ========== 速度模拟：使用低通滤波器平滑速度变化 ==========
        val nowTime = System.currentTimeMillis()
        // 仅当时间间隔合理时才更新滤波器，避免长时间暂停后突变
        if (lastSpeedUpdateTime == 0L || (nowTime - lastSpeedUpdateTime) < 2000) {
            // 目标速度：配置速度 + 微小随机波动（波动幅度减小，且与加速度相关）
            val targetSpeed = FakeLoc.speed + Random.nextDouble(-FakeLoc.speedAmplitude * 0.3, FakeLoc.speedAmplitude * 0.3)
            val newFiltered = SPEED_ALPHA * targetSpeed + (1 - SPEED_ALPHA) * lastFilteredSpeed
            lastFilteredSpeed = newFiltered.coerceIn(0.0, 40.0)  // 不超过40m/s
        } else {
            // 如果长时间未更新，直接使用配置速度，避免滞后太大
            lastFilteredSpeed = FakeLoc.speed
        }
        lastSpeedUpdateTime = nowTime
        var newSpeed = lastFilteredSpeed
        // 额外添加基于正弦波的微小波动，模拟自然加速度变化（周期约30秒）
        val cycle = sin(nowTime / 30000.0 * Math.PI) * FakeLoc.speedAmplitude * 0.2
        newSpeed += cycle
        if (newSpeed < 0) newSpeed = 0.0
        location.speed = newSpeed.toFloat()
        // ==============================================================
        
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && originLocation.hasSpeedAccuracy()) {
            location.speedAccuracyMetersPerSecond = (newSpeed).toFloat()
        }
    
        if (location.altitude == 0.0) {
            location.altitude = 80.0
        }
    
        location.time = originLocation.time
    
        // final addition of zero is to remove -0 results
        var modBearing = FakeLoc.bearing % 360.0 + 0.0
        if (modBearing < 0) {
            modBearing += 360.0
        }
        location.bearing = modBearing.toFloat()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            location.bearingAccuracyDegrees = modBearing.toFloat()
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            if (location.hasBearingAccuracy() && location.bearingAccuracyDegrees == 0.0f) {
                location.bearingAccuracyDegrees = 1.0f
            }
        }
    
        if (location.speed == 0.0f) {
            location.speed = 1.2f
        }
    
        location.elapsedRealtimeNanos = originLocation.elapsedRealtimeNanos
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            location.elapsedRealtimeUncertaintyNanos = originLocation.elapsedRealtimeUncertaintyNanos
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            location.verticalAccuracyMeters = originLocation.verticalAccuracyMeters
        }
        originLocation.extras?.let {
            location.extras = it
        }
        if (location.extras == null) {
            location.extras = Bundle()
        }
        location.extras?.putDouble("latlon", location.latitude + location.longitude)
        location.extras?.putInt("satellites", FakeLoc.updateSatelliteCount())
        location.extras?.putInt("maxCn0", Random.nextInt(30, 50))
        location.extras?.putInt("meanCn0", Random.nextInt(20, 30))
    
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            if (originLocation.hasMslAltitude()) {
                location.mslAltitudeMeters = FakeLoc.altitude
            }
            if (originLocation.hasVerticalAccuracy()) {
                location.mslAltitudeAccuracyMeters = FakeLoc.altitude.toFloat()
            }
        }
        if (FakeLoc.hideMock) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                location.isMock = false
            }
        } else {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                location.isMock = true
            }
            location.extras?.putBoolean("portal.enable", true)
            location.extras?.putBoolean("is_mock", true)
        }
    
        kotlin.runCatching {
            XposedHelpers.callMethod(location, "makeComplete")
        }.onFailure {
            Logger.error("makeComplete failed", it)
        }
    
        // ========== 修复时间戳冻结：强制将时间改为当前系统时间 ==========
        location.time = System.currentTimeMillis()
        location.elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
        // =============================================================
    
        if (FakeLoc.enableDebugLog) {
            Logger.debug("injectLocation success! $location")
        }
    
        return location
    }

    fun injectNMEA(nmeaStr: String): String? {
        // 过滤非 NMEA 语句（必须以 '$' 开头）
        if (!nmeaStr.startsWith('$')) {
            if (FakeLoc.enableDebugLog) {
                Logger.debug("Ignoring non-NMEA string: $nmeaStr")
            }
            return null
        }

        if (!FakeLoc.enable) {
            return null
        }

        kotlin.runCatching {
            val nmea = NMEA.valueOf(nmeaStr)
            when(val value = nmea.value) {
                is NmeaValue.DTM -> {
                    return null
                }
                is NmeaValue.GGA -> {
                    if (value.latitude == null || value.longitude == null) {
                        return null
                    }

                    if (value.fixQuality == 0) {
                        return null
                    }

                    val latitudeHemisphere = if (FakeLoc.latitude >= 0) "N" else "S"
                    val longitudeHemisphere = if (FakeLoc.longitude >= 0) "E" else "W"

                    value.latitudeHemisphere = latitudeHemisphere
                    value.longitudeHemisphere = longitudeHemisphere

                    var degree = FakeLoc.latitude.toInt()
                    var minute = (FakeLoc.latitude - degree) * 60
                    value.latitude = degree + minute / 100

                    degree = FakeLoc.longitude.toInt()
                    minute = (FakeLoc.longitude - degree) * 60
                    value.longitude = degree + minute / 100

                    return value.toNmeaString()
                }
                is NmeaValue.GNS -> {
                    if (value.latitude == null || value.longitude == null) {
                        return null
                    }

                    if (value.mode == "N") {
                        return null
                    }

                    val latitudeHemisphere = if (FakeLoc.latitude >= 0) "N" else "S"
                    val longitudeHemisphere = if (FakeLoc.longitude >= 0) "E" else "W"

                    value.latitudeHemisphere = latitudeHemisphere
                    value.longitudeHemisphere = longitudeHemisphere

                    var degree = FakeLoc.latitude.toInt()
                    var minute = (FakeLoc.latitude - degree) * 60
                    value.latitude = degree + minute / 100

                    degree = FakeLoc.longitude.toInt()
                    minute = (FakeLoc.longitude - degree) * 60
                    value.longitude = degree + minute / 100

                    return value.toNmeaString()
                }
                is NmeaValue.GSA -> {
                    return null
                }
                is NmeaValue.GSV -> {
                    return null
                }
                is NmeaValue.RMC -> {
                    if (value.latitude == null || value.longitude == null) {
                        return null
                    }

                    if (value.status == "V") {
                        return null
                    }

                    val latitudeHemisphere = if (FakeLoc.latitude >= 0) "N" else "S"
                    val longitudeHemisphere = if (FakeLoc.longitude >= 0) "E" else "W"

                    value.latitudeHemisphere = latitudeHemisphere
                    value.longitudeHemisphere = longitudeHemisphere

                    var degree = FakeLoc.latitude.toInt()
                    var minute = (FakeLoc.latitude - degree) * 60
                    value.latitude = degree + minute / 100

                    degree = FakeLoc.longitude.toInt()
                    minute = (FakeLoc.longitude - degree) * 60
                    value.longitude = degree + minute / 100

                    return value.toNmeaString()
                }
                is NmeaValue.VTG -> {
                    return null
                }
            }
        }.onFailure {
            Logger.error("NMEA parse failed: ${it.message}, source = $nmeaStr")
            return null
        }
        return null
    }
}