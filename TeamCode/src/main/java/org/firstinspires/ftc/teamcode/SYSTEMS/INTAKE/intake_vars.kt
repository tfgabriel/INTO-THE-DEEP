package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object intake_vars {
    @JvmField
    var transfer: Double = 0.81 //+0.08

    @JvmField
    var hover:Double = 0.57

    @JvmField
    var intake : Double = 0.48
    @JvmField
    var wrist_neutral: Double = 0.6 //+0.11

    @JvmField
    var reset_offset = 0.04
    var fourbar_intake: Double = 0.6 + reset_offset//+0.078
    @JvmField
    var fourbar_third: Double = 0.708 + reset_offset
    @JvmField
    var fourbar_transfer: Double = 0.6 + reset_offset
    var fourbar_hover: Double = 0.63 + reset_offset

    @JvmField
    var fourbar_yummy: Double = 0.63 + reset_offset

    @JvmField
    var fourbar_testing: Double = 0.8

    @JvmField
    var transverse_time: Double = 0.7
    @JvmField
    var spit_time: Double = 0.5
    @JvmField
    var intake_time:Double = 0.3

    @JvmField
    var claws_open: Double = 0.59
    @JvmField
    var claws_closed: Double = 0.38

}