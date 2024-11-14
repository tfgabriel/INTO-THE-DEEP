package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF

@Config
object extendo_vars {
    @JvmField
    var max_submersible: Int = 0
    @JvmField
    var home_submersible: Int = 0

    @JvmField
    var max_examination: Int = 0
    @JvmField
    var home_examination: Int = 0

    var extendo_pdf: PDF = PDF()

    @JvmField
    var proportional: Double = 0.0
    @JvmField
    var derivative: Double = 0.0
    @JvmField
    var force: Double = 0.0
    @JvmField
    var tolerance: Double = 0.0

    var extendo_target: Int = 0
}