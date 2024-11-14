package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF

@Config
object lift_vars {
    @JvmField
    var low_rung: Int = 0
    @JvmField
    var high_rung: Int = 0

    @JvmField
    var low_chamber: Int = 0
    @JvmField
    var high_chamber: Int = 0

    @JvmField
    var low_basket: Int = 0
    @JvmField
    var high_basket: Int = 0

    @JvmField
    var home: Int = 0

    @JvmField
    var max_extension: Int = 0

    var lift_pdf: PDF = PDF()

    @JvmField
    var proportional: Double = 0.0
    @JvmField
    var derivative: Double = 0.0
    @JvmField
    var force: Double = 0.0
    @JvmField
    var tolerance: Double = 0.0

    var lift_target: Int = 0
}