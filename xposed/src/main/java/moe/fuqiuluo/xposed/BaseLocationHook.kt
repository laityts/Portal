package moe.fuqiuluo.xposed

import android.location.Location
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import de.robv.android.xposed.XposedHelpers
import moe.fuqiuluo.xposed.utils.FakeLoc
import moe.fuqiuluo.xposed.utils.Logger
import moe.fuqiuluo.xposed.utils.MotionState
import moe.microbios.nmea.NMEA
import moe.microbios.nmea.NmeaValue
import kotlin.random.Random

abstract class BaseLocationHook: BaseDivineService() {
    fun injectLocation(originLocation: Location, realLocation: Boolean = true): Location {
        if (realLocation) {
            if (if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    originLocation.provider == LocationManager.GPS_PROVIDER && originLocation.isComplete
                } else {
                    originLocation.provider == LocationManager.GPS_PROVIDER
                }
            ) {
                FakeLoc.lastLocation = originLocation
            }
        } else {
            originLocation.altitude = FakeLoc.altitude
        }

        if (!FakeLoc.enable)
            return originLocation

        if (originLocation.extras?.containsKey("portal.injected") == true) {
            return originLocation
        }

        if (FakeLoc.disableNetworkLocation && originLocation.provider == LocationManager.NETWORK_PROVIDER) {
            originLocation.provider = LocationManager.GPS_PROVIDER
        }

        val snap = MotionState.snapshot
        val location = Location(originLocation.provider ?: LocationManager.GPS_PROVIDER)
        location.accuracy = if (FakeLoc.accuracy != 0.0f) FakeLoc.accuracy else originLocation.accuracy
        // 抖动：snap.jitterState 是 OU 相关随机游走的带符号位移（米），帧间连续。
        // 不复用 FakeLoc.jitterLocation（它有 /15 缩放和每帧 ±45° 独立随机，正是要替换的旧模型）。
        // 沿垂直于航向方向（bearing+90，随航向缓变而非每帧随机）叠加偏移，保持帧间相关。
        val earthRadius = 6371000.0
        val driftDeg = snap.jitterState / earthRadius * (180.0 / Math.PI)
        val driftAngle = Math.toRadians(snap.bearing + 90.0)
        location.latitude = snap.latitude + driftDeg * Math.cos(driftAngle)
        location.longitude = snap.longitude + driftDeg * Math.sin(driftAngle) / Math.cos(Math.toRadians(snap.latitude))
        location.altitude = FakeLoc.altitude
        location.speed = snap.speed.toFloat()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && originLocation.hasSpeedAccuracy()) {
            location.speedAccuracyMetersPerSecond = FakeLoc.speedAmplitude.toFloat()
        }

        if (location.altitude == 0.0) {
            location.altitude = 80.0
        }

        location.time = originLocation.time

        location.bearing = snap.bearing.toFloat()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            location.bearingAccuracyDegrees = if (originLocation.hasBearingAccuracy() && originLocation.bearingAccuracyDegrees > 0)
                originLocation.bearingAccuracyDegrees else 10.0f
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
        location.extras?.putBoolean("portal.injected", true)
        location.extras?.putInt("satellites", Random.nextInt(8, 45))
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

        if (FakeLoc.enableDebugLog) {
            Logger.debug("injectLocation success! $location")
        }

        return location
    }

    fun injectNMEA(nmeaStr: String): String? {
        if (!FakeLoc.enable) {
            return null
        }

        val snap = MotionState.snapshot
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

                    val latitudeHemisphere = if (snap.latitude >= 0) "N" else "S"
                    val longitudeHemisphere = if (snap.longitude >= 0) "E" else "W"

                    value.latitudeHemisphere = latitudeHemisphere
                    value.longitudeHemisphere = longitudeHemisphere

                    val absLat = kotlin.math.abs(snap.latitude)
                    var degree = absLat.toInt()
                    var minute = (absLat - degree) * 60
                    value.latitude = degree + minute / 100

                    val absLon = kotlin.math.abs(snap.longitude)
                    degree = absLon.toInt()
                    minute = (absLon - degree) * 60
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

                    val latitudeHemisphere = if (snap.latitude >= 0) "N" else "S"
                    val longitudeHemisphere = if (snap.longitude >= 0) "E" else "W"

                    value.latitudeHemisphere = latitudeHemisphere
                    value.longitudeHemisphere = longitudeHemisphere

                    val absLat = kotlin.math.abs(snap.latitude)
                    var degree = absLat.toInt()
                    var minute = (absLat - degree) * 60
                    value.latitude = degree + minute / 100

                    val absLon = kotlin.math.abs(snap.longitude)
                    degree = absLon.toInt()
                    minute = (absLon - degree) * 60
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

                    val latitudeHemisphere = if (snap.latitude >= 0) "N" else "S"
                    val longitudeHemisphere = if (snap.longitude >= 0) "E" else "W"

                    value.latitudeHemisphere = latitudeHemisphere
                    value.longitudeHemisphere = longitudeHemisphere

                    val absLat = kotlin.math.abs(snap.latitude)
                    var degree = absLat.toInt()
                    var minute = (absLat - degree) * 60
                    value.latitude = degree + minute / 100

                    val absLon = kotlin.math.abs(snap.longitude)
                    degree = absLon.toInt()
                    minute = (absLon - degree) * 60
                    value.longitude = degree + minute / 100

                    // 速度(节,1 m/s≈1.943844 节)与航向取自同一快照，与 Location 对象逐字段自洽
                    val rmc = value.copy(
                        speedKnots = snap.speed * 1.943844,
                        trackAngle = snap.bearing,
                    )
                    return rmc.toNmeaString()
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