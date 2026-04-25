// 文件名: FakeLoc.kt
// 修复平滑卫星数量生成器不生效的问题：使卫星数在 minSatellites 到 MAX_SATELLITES 之间动态变化
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

    /**
     * 为位置添加自然抖动模拟真实 GPS 误差和行走/驾驶时的随机偏航。
     * 改进：抖动幅度大幅降低，且偏向运动方向，避免出现大幅跳跃。
     * @param lat 原始纬度
     * @param lon 原始经度
     * @param n 抖动幅度（米），若不指定则根据 speed 动态计算（速度越快抖动越大，但上限降低）
     * @param angle 基准方向（用于产生非对称抖动，使轨迹更真实）
     * @return 抖动后的经纬度对
     */
    fun jitterLocation(lat: Double = latitude, lon: Double = longitude, n: Double = -1.0, angle: Double = bearing): Pair<Double, Double> {
        val earthRadius = 6371000.0
        // 基础抖动幅度降低：最大不超过5米，速度因子降低
        val baseJitter = if (n > 0) n else {
            val speedFactor = (speed.coerceIn(0.0, 30.0) / 30.0) * 3.0  // 0~3 米
            1.5 + speedFactor + Random.nextDouble(0.0, 1.5)  // 总体 1.5~6 米
        }
        // 抖动方向更倾向于运动方向（角度偏移范围缩小到 ±20°）
        val jitterAngleOffset = Random.nextDouble(-20.0, 20.0)
        val effectiveAngle = angle + jitterAngleOffset
        val radiusInMeters = baseJitter * Random.nextDouble(0.7, 1.3)  // 幅度随机范围缩小
        val radiusInDegrees = radiusInMeters / earthRadius * (180 / PI)

        val newLat = lat + radiusInDegrees * cos(Math.toRadians(effectiveAngle))
        val newLon = lon + radiusInDegrees * sin(Math.toRadians(effectiveAngle)) / cos(Math.toRadians(lat))
        return Pair(newLat, newLon)
    }

    fun moveLocation(lat: Double = latitude, lon: Double = longitude, n: Double, angle: Double = bearing): Pair<Double, Double> {
        val earthRadius = 6371000.0
        // 真实移动距离添加微小随机误差（±5%）
        val realDistance = n * Random.nextDouble(0.95, 1.05)
        // 移动方向也添加小角度漂移（±3°）
        val realAngle = angle + Random.nextDouble(-3.0, 3.0)
        val radiusInDegrees = realDistance / earthRadius * (180 / PI)
        val newLat = lat + radiusInDegrees * cos(Math.toRadians(realAngle))
        val newLon = lon + radiusInDegrees * sin(Math.toRadians(realAngle)) / cos(Math.toRadians(lat))
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

    // ==================== 平滑卫星数量生成器（修复版本） ====================
    private const val MAX_SATELLITES = 35
    
    private var lastSatCountUpdateTime = 0L
    private var cachedSatCount = 12
    
    /**
     * 获取当前模拟的可见卫星数量（平滑模型）
     * 基于：
     * - 时间正弦波（模拟卫星轨道变化），在 minSatellites 到 MAX_SATELLITES 之间波动
     * - 当前速度（开阔地增加，城市峡谷减少）
     * - 尊重用户配置的最低卫星数 minSatellites
     *
     * 修复：不再只是将低值钳位到 minSatellites，而是让平均值围绕 minSatellites 上方波动，
     *       确保卫星数量在 minSatellites ~ MAX_SATELLITES 范围内动态变化。
     */
    @Synchronized
    fun updateSatelliteCount(): Int {
        val now = System.currentTimeMillis()
        // 每秒更新一次，避免高频计算
        if (now - lastSatCountUpdateTime < 1000) {
            return cachedSatCount
        }
        lastSatCountUpdateTime = now

        // 确定动态范围：最小值 = minSatellites，最大值 = MAX_SATELLITES
        val minAllowed = minSatellites.coerceIn(1, MAX_SATELLITES)
        val range = MAX_SATELLITES - minAllowed
        // 正弦波周期 60 秒，相位 0..2π
        val angle = (now % 60000) / 60000.0 * 2 * Math.PI
        val sinVal = (Math.sin(angle) + 1) / 2  // 0..1，平滑波动
        // 基础卫星数在 minAllowed 到 max 之间随正弦变化
        var baseCount = minAllowed + (sinVal * range).toInt()

        // 根据当前速度调整（速度越快越可能开阔）
        val spd = speed
        when {
            spd > 30.0 -> baseCount += 4
            spd > 15.0 -> baseCount += 2
            spd < 5.0  -> baseCount -= 2
            spd < 10.0 -> baseCount -= 1
        }

        // 添加微小随机波动（±1~2颗），使每次查询都有细微差异
        val randomDelta = Random.nextInt(-2, 3)  // -2..2
        baseCount += randomDelta

        // 最终限制在允许范围内
        cachedSatCount = baseCount.coerceIn(minAllowed, MAX_SATELLITES)
        return cachedSatCount
    }
}