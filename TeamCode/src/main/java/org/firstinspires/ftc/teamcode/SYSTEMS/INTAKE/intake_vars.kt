package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object intake_vars {
    @JvmField
    var chub_arm_neutral: Double = 0.25
    @JvmField
    var chub_arm_intake: Double = 0.6
    var chub_arm_transfer: Double = 0.65
    @JvmField
    var chub_arm_specimen: Double = 0.35

    @JvmField
    var ehub_arm_neutral: Double = 0.3
    @JvmField
    var ehub_arm_intake: Double = 0.00
    var ehub_arm_transfer: Double = 0.00
    @JvmField
    var ehub_arm_specimen: Double = 0.23

    var wrist_intake: Double = 0.42
    var wrist_neutral: Double = 0.48

    var intaker_power: Double = 1.0
    var intaker_spit_power: Double = -1.0

    var fourbar_intake: Double = 0.0
    @JvmField
    var fourbar_transfer: Double = 0.64
    var fourbar_up: Double = 0.85
    var fourbar_mid: Double = 0.7

    @JvmField
    var fourbar_testing: Double = 0.8
    @JvmField
    var chub_arm_testing: Double = 0.0
    @JvmField
    var ehub_arm_testing: Double = 0.0
    @JvmField
    var wrist_testing: Double = 0.48
    @JvmField
    var intaker_power_testing: Double = 0.0

    @JvmField
    var transverse_time: Double = 0.5
    @JvmField
    var spit_time: Double = 0.5
    @JvmField
    var intake_time:Double = 0.3

}