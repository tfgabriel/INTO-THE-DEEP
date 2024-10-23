package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object vars {
    @JvmField
    var chub_arm_neutral: Double = 0.0
    @JvmField
    var chub_arm_intake: Double = 0.0

    @JvmField
    var ehub_arm_neutral: Double = 0.0
    @JvmField
    var ehub_arm_intake: Double = 0.0

    @JvmField
    var wrist_intake: Double = 0.0
    @JvmField
    var wrist_neutral: Double = 0.0

    @JvmField
    var intaker_power: Double = 0.0
    @JvmField
    var intaker_spit_power: Double = 0.0
}