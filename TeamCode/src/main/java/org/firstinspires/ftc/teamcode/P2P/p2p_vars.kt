package org.firstinspires.ftc.teamcode.P2P

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import kotlin.math.PI

@Config
object p2p_vars{

    @JvmField
    var x_p: Double = 0.055
    @JvmField
    var x_d: Double = 0.0011
    @JvmField
    var x_f: Double = 0.025

    @JvmField
    var y_p: Double = 0.022
    @JvmField
    var y_d: Double = 0.15
    @JvmField
    var y_f: Double = 0.1

    @JvmField
    var h_p: Double = 0.41
    @JvmField
    var h_d: Double = 0.0
    @JvmField
    var h_f: Double = 0.031
    @JvmField
    var tolerance: Double = 4.0
    @JvmField
    var angular_tolerance: Double = 0.2

    @JvmField
    val slow: Double = 1.0

}

@Config
object red_vars_sample {
    var sample0 = Pose()
    var sample1 = Pose()
    var sample2 = Pose()

    var sample_mid = Pose()

    var score_sample_0 = Pose()
    var score_sample_1 = Pose()
    var score_sample_2 = Pose()

    var smash_wall_samp = Pose()
}

@Config
object red_vars_specimen {
    var spec0 = Pose()
    var spec1 = Pose()
    var spec2 = Pose()

    var spec_mid = Pose()

    var score_spec_0 = Pose()
    var score_spec_1 = Pose()
    var score_spec_2 = Pose()

    var smash_wall_spec = Pose()
}

@Config
object blue_vars_sample{
    var sample0 = Pose()
    var sample1 = Pose()
    var sample2 = Pose()

    var sample_mid = Pose()

    var score_sample_0 = Pose()
    var score_sample_1 = Pose()
    var score_sample_2 = Pose()

    var smash_wall_samp = Pose()
}

@Config
object blue_vars_specimen {
    var spec0 = Pose()
    var spec1 = Pose()
    var spec2 = Pose()

    var spec_mid = Pose()

    var score_spec_0 = Pose()
    var score_spec_1 = Pose()
    var score_spec_2 = Pose()

    var smash_wall_spec = Pose()
}