package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object intake_vars {
    @JvmField
    var chub_arm_neutral: Double = 0.0
    @JvmField
    var chub_arm_intake: Double = 0.0

    @JvmField
    var ehub_arm_neutral: Double = 0.0
    @JvmField
    var ehub_arm_intake: Double = 0.0

    var wrist_intake: Double = 0.42
    var wrist_neutral: Double = 0.73 //on the other end, 0.11

    var intaker_power: Double = 1.0
    var intaker_spit_power: Double = -1.0

    @JvmField
    var chub_arm_testing: Double = 0.0
    @JvmField
    var ehub_arm_testing: Double = 0.0
    @JvmField
    var wrist_testing: Double = 0.0
    @JvmField
    var intaker_power_testing: Double = 0.0

}