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
