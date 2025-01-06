package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF

@Config
object lift_vars {
    @JvmField
    var low_rung: Int = 750
    @JvmField
    var high_rung: Int = 2300

    @JvmField
    var low_chamber: Int = 350
    @JvmField
    var high_chamber: Int = 1300

    @JvmField
    var transfer: Int = 500

    @JvmField
    var low_basket: Int = 1550
    @JvmField
    var high_basket: Int = 2465

    @JvmField
    var home: Int = 0

    @JvmField
    var max_extension: Int = 2440

    var lift_pdf: PDF = PDF()

    /// THE PID WORKS !!!
    @JvmField
    var proportional: Double = 0.00155
    @JvmField
    var derivative: Double = -0.01
    @JvmField
    var force: Double = 0.008
    @JvmField
    var tolerance: Double = 8.0

    var lift_target: Int = 0

    @JvmField
    var lazy_coef: Double = 0.003

    @JvmField
    var rebepe: Double = 0.000005
}