package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object intake_vars {
    @JvmField
    var transfer: Double = 0.75

    @JvmField
    var hover:Double = 0.43

    @JvmField
    var intake : Double = 0.37

    var wrist_intake: Double = 0.60
    @JvmField
    var wrist_neutral: Double = 0.6 //+0.11

    var intaker_power: Double = 1.0
    var intaker_spit_power: Double = -1.0

    var fourbar_intake: Double = 0.56
    @JvmField
    var fourbar_third: Double = 0.63
    @JvmField
    var fourbar_transfer: Double = 0.522
    var fourbar_hover: Double = 0.6

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