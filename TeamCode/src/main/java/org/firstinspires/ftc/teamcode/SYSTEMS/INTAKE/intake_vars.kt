package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object intake_vars {
    @JvmField
    var chub_arm_neutral: Double = 0.63
    @JvmField
    var chub_arm_intake: Double = 0.90
    @JvmField
    var chub_arm_transfer: Double = 0.94
    @JvmField
    var chub_arm_specimen: Double = 0.70

    @JvmField
    var ehub_arm_neutral: Double = 0.31
    @JvmField
    var ehub_arm_intake: Double = 0.03
    @JvmField
    var ehub_arm_transfer: Double = 0.05
    @JvmField
    var ehub_arm_specimen: Double = 0.27

    var wrist_intake: Double = 0.42
    var wrist_neutral: Double = 0.55

    var intaker_power: Double = 1.0
    var intaker_spit_power: Double = -1.0

    var fourbar_intake: Double = 0.1
    var fourbar_transfer: Double = 0.73
    var fourbar_up: Double = 0.79
    var fourbar_mid: Double = 0.74

    @JvmField
    var fourbar_testing: Double = 0.0
    @JvmField
    var chub_arm_testing: Double = 0.0
    @JvmField
    var ehub_arm_testing: Double = 0.0
    @JvmField
    var wrist_testing: Double = 0.55
    @JvmField
    var intaker_power_testing: Double = 0.0

    @JvmField
    var transverse_time: Double = 2.0
    @JvmField
    var spit_time: Double = 0.5
    @JvmField
    var intake_time: Double = 0.3

}