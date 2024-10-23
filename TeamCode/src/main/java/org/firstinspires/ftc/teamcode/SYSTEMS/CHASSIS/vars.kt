package org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS

import com.acmerobotics.dashboard.config.Config

@Config
object vars {
    var slowdown: Double = 0.0

    @JvmField
    var slowcoef: Double = 1.0

    var denominator: Double = 0.0
}