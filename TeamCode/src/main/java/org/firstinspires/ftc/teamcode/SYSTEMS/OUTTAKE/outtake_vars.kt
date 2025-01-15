package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import com.acmerobotics.dashboard.config.Config

@Config
object outtake_vars {

    //positioner offsets are ~0.31
    @JvmField
    var transfer_outtake = 0.26
    @JvmField
    var idle = 0.28
    @JvmField
    var score_basket = 0.55
    @JvmField
    var score_specimen = 0.51
    @JvmField
    var steal = 0.72


    @JvmField
    var fb_transfer = 0.93
    @JvmField
    var fb_score = 0.51
    @JvmField
    var fb_steal = 0.93

    @JvmField
    var claw_open = 0.5
    @JvmField
    var claw_close = 0.31


    @JvmField
    var place_time: Double = 0.3
    @JvmField
    var prepare_time_sample: Double = 0.5
    @JvmField
    var prepare_time_specimen: Double = 0.5
    @JvmField
    var positioner_testing: Double = 0.7
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