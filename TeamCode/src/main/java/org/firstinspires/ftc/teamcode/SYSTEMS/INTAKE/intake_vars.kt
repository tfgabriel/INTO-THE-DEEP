package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object intake_vars {
    @JvmField
    var chub_arm_neutral: Double = 0.59
    @JvmField
    var chub_arm_intake: Double = 0.87
    @JvmField
    var chub_arm_transfer: Double = 0.90
    @JvmField
    var chub_arm_specimen: Double = 0.64
    var chub_arm_intermediary: Double = 0.85
    @JvmField
    var ehub_arm_neutral: Double = 0.41
    @JvmField
    var ehub_arm_intake: Double = 0.18
    @JvmField
    var ehub_arm_transfer: Double = 0.17
    @JvmField
    var ehub_arm_specimen: Double = 0.39
    var ehub_arm_intermediary: Double = 0.20

    var wrist_intake: Double = 0.37
    var wrist_neutral: Double = 0.67

    var intaker_power: Double = 1.0
    var intaker_spit_power: Double = -1.0

    var fourbar_intake: Double = 0.45
    @JvmField
    var fourbar_transfer: Double = 0.965
    var fourbar_up: Double = 0.94
    var fourbar_mid: Double = 0.0
    @JvmField
    var fourbar_deaaa: Double = 0.97

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
    var transverse_time: Double = 0.7
    @JvmField
    var spit_time: Double = 0.5
    @JvmField
    var intake_time:Double = 0.3

    @JvmField
    var claws_open: Double = 0.59
    @JvmField
    var claws_closed: Double = 0.4

}