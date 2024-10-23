package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFL

@Config
object vars {
    @JvmField
    var max_submersible: Int = 0
    @JvmField
    var home_submersible: Int = 0

    @JvmField
    var max_examination: Int = 0
    @JvmField
    var home_examination: Int = 0

    var extendo_pdfl: PDFL = PDFL()

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