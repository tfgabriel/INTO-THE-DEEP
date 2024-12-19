package org.firstinspires.ftc.teamcode.ALGORITHMS

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.sign


@Config
class PDF(@JvmField val p: Double, @JvmField var d: Double, @JvmField var f: Double) {
    constructor(): this(0.0,0.0,0.0)
    constructor(p: Double, d: Double): this(p, d, 0.0)
    constructor(f: Double): this(0.0, 0.0, f)

    private var proportionate = 0.0
    private var derivative = 0.0
    private val force = f
    var ep = ElapsedTime()

    fun update(err: Double): Double{
        derivative = (err - proportionate) / ep.milliseconds()
        ep.reset()
        proportionate = err

        return proportionate * p + derivative * d + sign(err) * force
    }
}