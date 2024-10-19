package org.firstinspires.ftc.teamcode.ALGORITHMS

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.sign


class PDFLCoef(@JvmField var p: Double, @JvmField var d: Double, @JvmField var f: Double, @JvmField var l: Double){

}

class PDFL(val coef: PDFLCoef) {
    constructor(): this(PDFLCoef(0.0,0.0,0.0,0.0))

    private var proportionate = 0.0
    private var derivative = 0.0
    private val force = coef.f
    private val lower_limit = coef.l

    var ep = ElapsedTime()

    fun update(err: Int, tolerance: Double): Double{
        val error = err.toDouble()

        derivative = (err - proportionate) / ep.milliseconds()
        ep.reset()
        proportionate = error

        return if(abs(err) > tolerance)
            proportionate * coef.p + derivative + coef.d + sign(error) * force + sign(error) * lower_limit
        else
            force * sign(error)
    }
}