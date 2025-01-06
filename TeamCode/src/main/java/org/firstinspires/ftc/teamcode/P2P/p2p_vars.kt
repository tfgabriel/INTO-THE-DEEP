package org.firstinspires.ftc.teamcode.P2P

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFC
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import kotlin.math.PI

@Config
object p2p_vars{

    @JvmField
    var PDFCX: PDFC = PDFC(0.05, 0.0, 0.005)
    @JvmField
    var PDFCY: PDFC = PDFC(0.05, 0.0, 0.005)
    @JvmField
    var PDFCH: PDFC = PDFC(0.41, 0.0, 0.0)

    @JvmField
    var PDFCFinalX: PDFC = PDFC(0.02, 0.0, 0.00)
    @JvmField
    var PDFCFinalY: PDFC = PDFC(0.035, 0.0, 0.0)
    @JvmField

    var PDFCFinalH: PDFC = PDFC(0.4, 0.0, 0.0)

    @JvmField
    var tolerance: Double = 3.0
    @JvmField
    var angular_tolerance: Double = 0.2

    @JvmField
    var anti_p: Double = 0.4

    @JvmField
    var PeruMin = 0.4
    @JvmField
    var PeruMax = 1.0

    @JvmField
    var speed_limit_linear = 2.0
    @JvmField
    var speed_limit_angular = 2.0
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