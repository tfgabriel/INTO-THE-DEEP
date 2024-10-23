package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFL

@Config
object vars {
    @JvmField
    var low_rung: Int = 0
    @JvmField
    var high_rung: Int = 0

    @JvmField
    var low_chamber: Int = 0
    @JvmField
    var high_chamber: Int = 0

    @JvmField
    var home: Int = 0

    @JvmField
    var max_extension: Int = 0

    var lift_pdfl: PDFL = PDFL()

    @JvmField
    var proportional: Double = 0.0
    @JvmField
    var derivative: Double = 0.0
    @JvmField
    var lower_limit: Double = 0.0
    @JvmField
    var force: Double = 0.0
    @JvmField
    var tolerance: Double = 0.0
}