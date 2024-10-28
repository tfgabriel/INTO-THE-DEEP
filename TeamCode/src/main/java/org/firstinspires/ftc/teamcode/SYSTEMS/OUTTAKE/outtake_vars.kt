package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object outtake_vars {

    //positioner offsets are ~0.31
    var positioner_neutral: Double = 0.39
    var positioner_chub: Double = 0.08
    var positioner_ehub: Double = 0.7

    var wrist_neutral: Double = 0.086
    var wrist_intake: Double = 0.0

    @JvmField
    var ehub_arm_pickup: Double = 0.0
    @JvmField
    var ehub_arm_place: Double = 0.0

    @JvmField
    var chub_arm_pickup: Double = 0.0
    @JvmField
    var chub_arm_place: Double = 0.0

    //claw offsets are ~0.25-0.30
    var ehub_claw_open: Double = 0.65
    var ehub_claw_close: Double = 0.4

    var chub_claw_open: Double = 1.0
    var chub_claw_close: Double = 0.75

    @JvmField
    var positioner_testing: Double = 0.0
    @JvmField
    var wrist_testing: Double = 0.0
    @JvmField
    var chub_claw_testing: Double = 0.0
    @JvmField
    var ehub_claw_testing: Double = 0.0
    @JvmField
    var chub_arm_testing: Double = 0.0
    @JvmField
    var ehub_arm_testing: Double = 0.0
}