package org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec4D

@Config
object chassis_vars {
    var slowdown: Double = 0.0

    @JvmField
    var slowcoef: Double = 1.0

    var denominator: Double = 0.0

    @JvmField
    var p: Double = 0.35

    @JvmField
    var d: Double = 0.15

    @JvmField
    var f: Double = 0.1

    @JvmField
    var angular_tolerance: Double = 0.1

    @JvmField
    var heading_timeout: Double = 50.0

    @JvmField
    var imu_offset: Double = -.45

    var h_PDF = PDF()

    @JvmField
    var chassis_f = Vec4D(0.05, 0.05, 0.05, 0.07)
}