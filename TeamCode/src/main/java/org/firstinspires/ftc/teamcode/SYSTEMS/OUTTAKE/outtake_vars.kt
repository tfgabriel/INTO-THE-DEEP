package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object outtake_vars {

    //positioner offsets are ~0.31
    var positioner_neutral: Double = 0.69
    var positioner_chub: Double = 0.08
    var positioner_ehub: Double = 0.7

    var wrist_neutral: Double = 0.086
    var wrist_intake: Double = 0.0

    var ehub_arm_pickup: Double = 0.25
    var ehub_arm_place: Double = 0.09
    var ehub_arm_basket: Double = 0.0

    var chub_arm_pickup: Double = 0.625
    var chub_arm_place: Double = 0.79
    var chub_arm_basket: Double = 0.89
    //offsets ~0.15y
    var ehub_claw_open: Double = 0.8
    var ehub_claw_close: Double = 0.57
    var ehub_claw_intermed: Double = 0.61

    var chub_claw_open: Double = 0.68
    var chub_claw_close: Double = 0.39
    var chub_claw_intermed: Double = 0.43


    @JvmField
    var place_time: Double = 0.5
    @JvmField
    var prepare_time_sample: Double = 0.5
    @JvmField
    var prepare_time_specimen: Double = 0.5
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