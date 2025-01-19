package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFC
import org.firstinspires.ftc.teamcode.ALGORITHMS.SQUID

@Config
object extendo_vars {
    // NOT YET FINISHED
    @JvmField
    var home_extendo: Int = 0
    @JvmField
    var mid_examination: Int = -300


    @JvmField
    var max_examination: Int = -725
    @JvmField
    var home_examination: Int = 0

    @JvmField
    var manual_tresh: Int = 350

    var extendo_pdf: SQUID = SQUID()

    @JvmField
    var extc = PDFC(0.05, 0.002, 0.0)
    @JvmField
    var tolerance: Double = 5.5

    @JvmField
    var hometolerance: Double = -15.0

    @JvmField
    var extendo_target: Int = 0

    @JvmField
    var modify_tresh: Int = 50

    @JvmField
    var home_extendo_tolerance = 15.0
}