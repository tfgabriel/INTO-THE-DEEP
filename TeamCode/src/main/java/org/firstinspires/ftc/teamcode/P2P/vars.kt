package org.firstinspires.ftc.teamcode.P2P

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFL
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose

@Config
object vars{
    lateinit var path: Pose
    lateinit var current_pos: Pose
    lateinit var err: Pose

    var x_err: Double = 0.0
    var y_err: Double = 0.0
    var h_err: Double = 0.0

    @JvmField
    var x_p: Double = 0.0
    @JvmField
    var x_d: Double = 0.0
    @JvmField
    var x_f: Double = 0.0

    @JvmField
    var y_p: Double = 0.0
    @JvmField
    var y_d: Double = 0.0
    @JvmField
    var y_f: Double = 0.0

    @JvmField
    var h_p: Double = 0.0
    @JvmField
    var h_d: Double = 0.0
    @JvmField
    var h_f: Double = 0.0

    var xPDFL: PDFL = PDFL()
    var yPDFL: PDFL = PDFL()
    var hPDFL: PDFL = PDFL()

    @JvmField
    var tolerance: Double = 0.0
    @JvmField
    var angular_tolerance: Double = 0.0

    @JvmField
    val slow: Double = 1.0
}

@Config
object red_vars {

}

@Config
object blue_vars{

}