package org.firstinspires.ftc.teamcode.ALGORITHMS

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class PDFC(@JvmField var p: Double, @JvmField var d: Double, @JvmField var f: Double) {
    fun duplicate() = PDFC(p, d, f)
}

class PDF(@JvmField var coef: PDFC) {
    constructor(): this(PDFC(0.0,0.0,0.0))
    constructor(p: Double, d: Double): this(PDFC(p, d, 0.0))
    constructor(p: Double, d: Double, f: Double): this(PDFC(p, d, f))
    constructor(f: Double): this(PDFC(0.0, 0.0, f))

    private var proportionate = 0.0
    private var derivative = 0.0
    var ep = ElapsedTime()

    fun update(err: Double): Double{
        derivative = (err - proportionate) / ep.milliseconds()
        ep.reset()
        proportionate = err

        return proportionate * coef.p + derivative * coef.d + sign(err) * coef.f
    }
}

class SQUID(@JvmField var coef: PDFC) {
    constructor(): this(PDFC(0.0,0.0,0.0))
    fun update(err: Double): Double {
        return sign(err) * (sqrt(abs(err)) * coef.p + coef.f)
    }
}