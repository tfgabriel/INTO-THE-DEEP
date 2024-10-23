package org.firstinspires.ftc.teamcode.ALGORITHMS

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.sign

@Config
class PDFLCoef(@JvmField var p: Double, @JvmField var d: Double, @JvmField var f: Double, @JvmField var l: Double){

}

class PDFL(val coef: PDFLCoef) {
    constructor(): this(PDFLCoef(0.0,0.0,0.0,0.0))

    private var proportionate = 0.0
    private var derivative = 0.0
    private val force = coef.f
    private val lower_limit = coef.l

    var ep = ElapsedTime()

    fun update(err: Double, tolerance: Double): Double{

        derivative = (err - proportionate) / ep.milliseconds()
        ep.reset()
        proportionate = err

        return if(abs(err) > tolerance)
            proportionate * coef.p + derivative + coef.d + sign(err) * force + sign(err) * lower_limit
        else
            force * sign(err)
    }
}