package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF

@Config
object lift_vars {
    @JvmField
    var low_rung: Int = 900
    @JvmField
    var high_rung: Int = 2300

    @JvmField
    var low_chamber: Int = 350
    @JvmField
    var high_chamber: Int = 1200

    @JvmField
    var low_basket: Int = 1550
    @JvmField
    var high_basket: Int = 2440

    @JvmField
    var home: Int = 0

    @JvmField
    var max_extension: Int = 2440

    var lift_pdf: PDF = PDF()

    /// THE PID WORKS !!!
    @JvmField
    var proportional: Double = 0.0012
    @JvmField
    var derivative: Double = -0.01
    @JvmField
    var force: Double = 0.008
    @JvmField
    var tolerance: Double = 3.0

    var lift_target: Int = 0
}