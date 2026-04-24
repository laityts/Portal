package moe.fuqiuluo.xposed.utils

import android.location.Location
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random
import java.io.File
import java.io.FileOutputStream
import java.nio.channels.FileLock
import java.util.concurrent.locks.ReentrantReadWriteLock

object FakeLoc {
    /**
     * 是否允许打印日志
     */
    @Volatile
    var enableLog = true

    /**
     * 是否允许打印调试日志
     */
    @Volatile
    var enableDebugLog = true

    /**
     * 模拟定位服务开关
     */
    @Volatile
    var enable = false

    /**
     * 模拟Gnss卫星数据开关
     */
    @Volatile
    var enableMockGnss = false

    /**
     * 模拟WLAN数据
     */
    @Volatile
    var enableMockWifi = false

    /**
     * 是否禁用GetCurrentLocation方法（在部分系统不禁用可能导致hook失效）
     */
    @Volatile
    var disableGetCurrentLocation = true

    /**
     * 是否禁用RegisterLocationListener方法
     */
    @Volatile
    var disableRegisterLocationListener = false

    /**
     * 如果TelephonyHook失效，可能需要打开此开关
     */
    @Volatile
    var disableFusedLocation = true
    @Volatile
    var disableNetworkLocation = true

    @Volatile
    var disableRequestGeofence = false
    @Volatile
    var disableGetFromLocation = false

    /**
     * 是否允许AGPS模块（当前没什么鸟用）
     */
    @Volatile
    var enableAGPS = false

    /**
     * 是否允许NMEA模块
     */
    @Volatile
    var enableNMEA = false

    /**
     * 是否隐藏模拟位置
     */
    @Volatile
    var hideMock = true

    /**
     * may cause system to crash
     */
    @Volatile
    var hookWifi = true

    /**
     * 将网络定位降级为Cdma
     */
    @Volatile
    var needDowngradeToCdma = true
    @Volatile
    var isSystemServerProcess = false

    /**
     * 模拟最小卫星数量
     */
    @Volatile
    var minSatellites = 12

    /**
     * 反定位复原加强（启用后将导致部分应用在关闭Portal后需要重新启动才能重新获取定位）
     */
    @Volatile
    var loopBroadcastLocation = false

    /**
     * 上一次的位置
     */
    @Volatile var lastLocation: Location? = null
    @Volatile var latitude = 0.0
    @Volatile var longitude = 0.0
    @Volatile var altitude = 80.0

    @Volatile var speed = 3.05

    @Volatile var speedAmplitude = 1.0

    @Volatile var hasBearings = false

    var bearing = 0.0
        get() {
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

    var accuracy = 25.0f
        set(value) {
            field = if (value < 0) {
                -value
            } else {
                value
            }
        }

    fun haversine(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val radius = 6371000.0
        val phi1 = Math.toRadians(lat1)
        val phi2 = Math.toRadians(lat2)
        val deltaPhi = Math.toRadians(lat2 - lat1)
        val deltaLambda = Math.toRadians(lon2 - lon1)
        val a = sin(deltaPhi / 2).pow(2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2).pow(2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return radius * c
    }

    fun jitterLocation(lat: Double = latitude, lon: Double = longitude, n: Double = Random.nextDouble(0.0, accuracy.toDouble()), angle: Double = bearing): Pair<Double, Double> {
        val earthRadius = 6371000.0
        val radiusInDegrees = n / 15 / earthRadius * (180 / PI)

        val jitterAngle = if (Random.nextBoolean()) angle + 45 else angle - 45

        val newLat = lat + radiusInDegrees * cos(Math.toRadians(jitterAngle))
        val newLon = lon + radiusInDegrees * sin(Math.toRadians(jitterAngle)) / cos(Math.toRadians(lat))

        return Pair(newLat, newLon)
    }

    fun moveLocation(lat: Double = latitude, lon: Double = longitude, n: Double, angle: Double = bearing): Pair<Double, Double> {
        val earthRadius = 6371000.0
        val radiusInDegrees = Random.nextDouble(n, n + 1.2) / earthRadius * (180 / PI)
        val newLat = lat + radiusInDegrees * cos(Math.toRadians(angle))
        val newLon = lon + radiusInDegrees * sin(Math.toRadians(angle)) / cos(Math.toRadians(lat))
        return Pair(newLat, newLon)
    }

    // Cross-Process Sync
    private const val CONFIG_FILE_PATH = "/data/local/tmp/portal_config.json"

    // 文件读写锁，防止多进程/多线程同时写入
    private val fileLock = ReentrantReadWriteLock()
    private val writeLock = fileLock.writeLock()
    private val readLock = fileLock.readLock()

    fun syncConfigToFile() {
        if (!isSystemServerProcess) return 
        // Only System Server writes to avoid conflicts (though User might change settings in App UI too? 
        // Actually App UI talks to RemoteCommandHandler(SystemServer), so System Server is the source of truth.
        
        writeLock.lock()
        try {
            val json = org.json.JSONObject()
            json.put("enable", enable)
            json.put("speed", speed)
            json.put("bearing", bearing)
            json.put("speedAmplitude", speedAmplitude)
            json.put("enableMockGnss", enableMockGnss)
            
            // JNI Native Update
            moe.fuqiuluo.xposed.FakeLocation.instance?.nativeUpdateConfig(enable, speed, bearing)
            
            // 使用临时文件 + 原子替换避免读进程读到不完整内容
            val file = File(CONFIG_FILE_PATH)
            val tempFile = File("${CONFIG_FILE_PATH}.tmp")
            tempFile.writeText(json.toString())
            // 确保权限正确
            tempFile.setReadable(true, false)
            tempFile.setWritable(true, false)
            // 原子替换
            if (tempFile.renameTo(file)) {
                // 成功
            } else {
                // 降级：直接写原文件
                file.writeText(json.toString())
                file.setReadable(true, false)
                file.setWritable(true, false)
            }
        } catch (e: Exception) {
            // moe.fuqiuluo.xposed.utils.Logger.error("Failed to sync config to file", e)
        } finally {
            writeLock.unlock()
        }
    }

    fun readConfigFromFile() {
        readLock.lock()
        try {
            val file = File(CONFIG_FILE_PATH)
            if (!file.exists()) return

            val text = file.readText()
            val json = org.json.JSONObject(text)

            if (json.has("enable")) enable = json.getBoolean("enable")
            if (json.has("speed")) speed = json.getDouble("speed")
            if (json.has("bearing")) {
                val newBearing = json.getDouble("bearing")
                // Only update if changed to avoid unnecessary re-calcs?
                // Actually field setter logic in FakeLoc handles normalisation but not field backing.
                // We should set the backing field or use setter.
                // But `bearing` property has no setter in the code snippet, it uses `field`.
                // Wait, the provided code for FakeLoc showed `bearing` has a getter but no explicit setter in the snippet?
                // Looking at snippet: `var bearing = 0.0` then `get()`. 
                // Since it's a `var`, it has a setter.
                bearing = newBearing
            }
            if (json.has("speedAmplitude")) speedAmplitude = json.getDouble("speedAmplitude")
            if (json.has("enableMockGnss")) enableMockGnss = json.getBoolean("enableMockGnss")

        } catch (e: Exception) {
            if (enableDebugLog) {
                Logger.error("Failed to read config from $CONFIG_FILE_PATH", e)
            }
        } finally {
            readLock.unlock()
        }
    }

    // ==================== 平滑卫星数量生成器 ====================
    private const val MIN_SATELLITES = 4
    private const val MAX_SATELLITES = 35
    
    private var lastSatCountUpdateTime = 0L
    private var cachedSatCount = 12
    
    /**
     * 获取当前模拟的可见卫星数量（平滑模型）
     * 基于：
     * - 时间正弦波（模拟卫星轨道变化）
     * - 当前速度（开阔地增加，城市峡谷减少）
     */
    @Synchronized
    fun updateSatelliteCount(): Int {
        val now = System.currentTimeMillis()
        // 每秒更新一次，避免高频计算
        if (now - lastSatCountUpdateTime < 1000) {
            return cachedSatCount
        }
        lastSatCountUpdateTime = now
    
        // 正弦波周期 60 秒，振幅 10，基线 12 -> 范围 2~22
        val angle = (now % 60000) / 60000.0 * 2 * Math.PI
        val sinVal = (Math.sin(angle) + 1) / 2  // 0..1
        var baseCount = 8 + (sinVal * 20).toInt()  // 8..28
    
        // 根据当前速度调整（速度越快越可能开阔）
        val spd = speed
        when {
            spd > 30.0 -> baseCount += 4
            spd > 15.0 -> baseCount += 2
            spd < 5.0  -> baseCount -= 3
            spd < 10.0 -> baseCount -= 1
        }
    
        cachedSatCount = baseCount.coerceIn(MIN_SATELLITES, MAX_SATELLITES)
        return cachedSatCount
    }
}