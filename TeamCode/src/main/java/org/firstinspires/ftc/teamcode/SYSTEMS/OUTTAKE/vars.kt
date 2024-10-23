package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object vars {
    @JvmField
    var positioner_neutral: Double = 0.0
    @JvmField
    var positioner_chub: Double = 0.0
    @JvmField
    var positioner_ehub: Double = 0.0

    @JvmField
    var wrist_neutral: Double = 0.0
    @JvmField
    var wrist_intake: Double = 0.0

    @JvmField
    var ehub_arm_pickup: Double = 0.0
    @JvmField
    var ehub_arm_place: Double = 0.0

    @JvmField
    var chub_arm_pickup: Double = 0.0
    @JvmField
    var chub_arm_place: Double = 0.0

    @JvmField
    var ehub_claw_open: Double = 0.0
    @JvmField
    var ehub_claw_close: Double = 0.0

    @JvmField
    var chub_claw_open: Double = 0.0
    @JvmField
    var chub_claw_close: Double = 0.0
}