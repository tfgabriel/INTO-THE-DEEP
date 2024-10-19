package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFL

object vars {
    var low_rung: Int = 0
    var high_rung: Int = 0

    var low_chamber: Int = 0
    var high_chamber: Int = 0

    var home: Int = 0

    var max_extension: Int = 0

    var lift_pdfl: PDFL = PDFL()

    var proportional: Double = 0.0
    var derivative: Double = 0.0
    var lower_limit: Double = 0.0
    var force: Double = 0.0
    var tolerance: Double = 0.0
}