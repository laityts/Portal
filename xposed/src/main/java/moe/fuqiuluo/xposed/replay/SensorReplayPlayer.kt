package moe.fuqiuluo.xposed.replay

import android.hardware.Sensor
import android.util.Log
import org.json.JSONObject
import java.io.File
import kotlin.math.cos
import kotlin.math.sin

object SensorReplayPlayer {
    private const val TAG = "SensorReplayPlayer"

    // 配置：允许通过外部设置文件路径
    private var walkingFilePath = "/data/local/tmp/motion_0pvsIohTUfTfZ1eBhnodR.json"
    private var idleFilePath = "/data/local/tmp/idle.json"

    fun setWalkingFilePath(path: String) { walkingFilePath = path }
    fun setIdleFilePath(path: String) { idleFilePath = path }

    // State
    private var isInitialized = false
    private val walkingTracks =  mutableMapOf<Int, SensorTrack>()
    private val idleTracks = mutableMapOf<Int, SensorTrack>()

    private var walkingDuration = 0L
    private var idleDuration = 0L

    // Replay State
    private var lastQueryTime = 0L
    private var replayStartTime = 0L
    private var replayOffset = 0L

    // Trajectory Defs
    private const val WALKING_BASE_HEADING = 180.0 // User said they walked South (180)

    data class SensorFrame(
        val timeOffset: Long, // ms from start of track
        val values: FloatArray
    )

    class SensorTrack(val frames: List<SensorFrame>) {
        fun getInterpolatedFrame(offset: Long): FloatArray? {
            if (frames.isEmpty()) return null
            var low = 0
            var high = frames.size - 1
            
            if (offset <= frames.first().timeOffset) return frames.first().values
            if (offset >= frames.last().timeOffset) return frames.last().values

            while (low <= high) {
                val mid = (low + high) / 2
                val midVal = frames[mid].timeOffset

                if (midVal < offset) {
                    low = mid + 1
                } else if (midVal > offset) {
                    high = mid - 1
                } else {
                    return frames[mid].values
                }
            }
            val idx = if (low >= frames.size) high else low
            return frames[idx].values
        }
    }

    fun init() {
        if (isInitialized) return
        try {
            val walkingFile = File(walkingFilePath)
            val idleFile = File(idleFilePath)

            if (walkingFile.exists() && idleFile.exists()) {
                val walkingJson = walkingFile.readText()
                val idleJson = idleFile.readText()

                parseFile(walkingJson, walkingTracks, cropStart = 0L, cropEnd = 60000L).let { walkingDuration = it }
                parseFile(idleJson, idleTracks).let { idleDuration = it }

                replayStartTime = System.currentTimeMillis()
                isInitialized = true
                Log.i(TAG, "Initialized SensorReplay. Walking: ${walkingDuration}ms, Idle: ${idleDuration}ms")
            } else {
                Log.e(TAG, "Sensor files not found at $walkingFilePath or $idleFilePath")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to init SensorReplay", e)
        }
    }

    private fun parseFile(jsonStr: String, targetMap: MutableMap<Int, SensorTrack>, cropStart: Long = 0, cropEnd: Long = Long.MAX_VALUE): Long {
        val root = JSONObject(jsonStr)
        val moments = root.getJSONArray("moments")
        
        val tempTracks = mutableMapOf<Int, MutableList<SensorFrame>>()
        var startTime = -1L
        var maxTime = 0L

        for (i in 0 until moments.length()) {
            val moment = moments.getJSONObject(i)
            val elapsedSec = moment.getDouble("elapsed")
            val elapsedMs = (elapsedSec * 1000).toLong()

            if (elapsedMs < cropStart) continue
            if (elapsedMs > cropEnd) break

            if (startTime == -1L) startTime = elapsedMs
            val relativeTime = elapsedMs - startTime
            if (relativeTime > maxTime) maxTime = relativeTime

            val data = moment.optJSONObject("data") ?: continue
            val keys = data.keys()
            while (keys.hasNext()) {
                val sensorTypeStr = keys.next()
                val sensorType = sensorTypeStr.toIntOrNull() ?: continue
                
                val valuesArr = data.getJSONArray(sensorTypeStr)
                val values = FloatArray(valuesArr.length()) { valuesArr.getDouble(it).toFloat() }
                
                tempTracks.getOrPut(sensorType) { mutableListOf() }
                    .add(SensorFrame(relativeTime, values))
            }
        }

        tempTracks.forEach { (type, list) ->
            targetMap[type] = SensorTrack(list)
        }
        
        return maxTime
    }

    private var accumulatedSteps = 0f
    private var lastStateChangeTime = 0L
    private var wasMoving = false

    fun getSensorValues(sensorType: Int, currentSpeed: Double, currentHeading: Double): FloatArray {
        val now = System.currentTimeMillis()
        if (!isInitialized) return FloatArray(0)

        if (now - lastQueryTime > 1000) {
            moe.fuqiuluo.xposed.utils.FakeLoc.readConfigFromFile()
            lastQueryTime = now
        }
        
        val localSpeed = moe.fuqiuluo.xposed.utils.FakeLoc.speed
        val localHeading = moe.fuqiuluo.xposed.utils.FakeLoc.bearing
        
        val isMoving = localSpeed > 0.5
        if (isMoving != wasMoving) {
            val durationInPrevState = now - lastStateChangeTime
            val prevIsMoving = wasMoving
            val prevDuration = if (prevIsMoving) walkingDuration else idleDuration
            val prevTrack = if (prevIsMoving) walkingTracks[19] else idleTracks[19]
            
            if (prevDuration > 0 && prevTrack != null && prevTrack.frames.isNotEmpty()) {
                val loops = durationInPrevState / prevDuration
                val remainder = durationInPrevState % prevDuration
                val trackTotal = prevTrack.frames.last().values[0] - prevTrack.frames.first().values[0]
                val currentInTrack = (prevTrack.getInterpolatedFrame(remainder)?.get(0) ?: prevTrack.frames.first().values[0]) - prevTrack.frames.first().values[0]
                
                accumulatedSteps += (loops * trackTotal) + currentInTrack
            }
            
            lastStateChangeTime = now
            wasMoving = isMoving
        }

        val loopDuration = if (isMoving) walkingDuration else idleDuration
        val tracks = if (isMoving) walkingTracks else idleTracks
        
        // 修复：空安全处理
        if (loopDuration == 0L || !tracks.containsKey(sensorType)) {
            return FloatArray(0)
        }

        val timeSinceStateStart = now - lastStateChangeTime
        val pointer = timeSinceStateStart % loopDuration

        if (sensorType == 19) {
            val track = tracks[19] ?: return floatArrayOf(accumulatedSteps)
            if (track.frames.isEmpty()) return floatArrayOf(accumulatedSteps)

            val trackStartVal = track.frames.first().values[0]
            val trackEndVal = track.frames.last().values[0]
            val trackTotal = trackEndVal - trackStartVal
            
            val valInLoop = (track.getInterpolatedFrame(pointer)?.get(0) ?: trackStartVal)
            val deltaInLoop = valInLoop - trackStartVal
            
            val loops = timeSinceStateStart / loopDuration
            
            val currentTotal = accumulatedSteps + (loops * trackTotal) + deltaInLoop
            return floatArrayOf(currentTotal)
        }

        val originalValues = tracks[sensorType]?.getInterpolatedFrame(pointer) ?: FloatArray(3)

        if (isVectorSensor(sensorType)) {
            val deltaDegrees = localHeading - WALKING_BASE_HEADING
            val thetaRad = Math.toRadians(-deltaDegrees)
            
            val modPhase = (now % 10000) / 10000.0 * 2 * Math.PI
            val amplitudeScale = 1.0 + (0.02 * sin(modPhase))
            
            val rotated = rotateVector(originalValues, thetaRad)
            for (i in rotated.indices) {
                rotated[i] = (rotated[i] * amplitudeScale).toFloat()
            }
            return rotated
        }

        return originalValues
    }

    private fun isVectorSensor(type: Int): Boolean {
        return type == Sensor.TYPE_ACCELEROMETER || 
               type == Sensor.TYPE_GYROSCOPE || 
               type == Sensor.TYPE_MAGNETIC_FIELD ||
               type == Sensor.TYPE_LINEAR_ACCELERATION ||
               type == Sensor.TYPE_GRAVITY
    }

    private fun rotateVector(values: FloatArray, thetaRad: Double): FloatArray {
        if (values.size < 3) return values
        val x = values[0]
        val y = values[1]
        val z = values[2]

        val c = cos(thetaRad).toFloat()
        val s = sin(thetaRad).toFloat()

        val newX = x * c - y * s
        val newY = x * s + y * c
        
        val result = values.clone()
        result[0] = newX
        result[1] = newY
        return result
    }
}