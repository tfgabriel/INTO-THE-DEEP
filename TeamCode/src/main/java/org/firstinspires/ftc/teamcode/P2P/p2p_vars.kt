package org.firstinspires.ftc.teamcode.P2P

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFC
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec4D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.control_hub
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.expansion_hub
import kotlin.math.sqrt

@Config
object p2p_vars{

    fun setCol(col: Int) {
        expansion_hub.setConstant(col)
        control_hub.setConstant(col)
    }
    @JvmField
    var PDFCX: PDFC = PDFC(0.05, 0.0, 0.00)
    @JvmField
    var PDFCY: PDFC = PDFC(0.05, 0.0, 0.00)
    @JvmField
    var PDFCH: PDFC = PDFC(0.5, 0.0, 0.0)

    @JvmField
    var PDFCFinalX: PDFC = PDFC(0.012, 0.0, 0.0)
    @JvmField
    var PDFCFinalY: PDFC = PDFC(0.012, 0.0, 0.0)
    @JvmField
    var PDFCFinalH: PDFC = PDFC(0.4, 0.0, 0.0)

    @JvmField
    var PDFCsebiH: PDFC = PDFC(0.3, 0.0, 0.0)
    @JvmField
    var deftones: Vec4D = Vec4D(5.2, 0.1, 6.0, 6.0) /// Pose, Ang, PoseVel, AngVel

    @JvmField
    var bigP = PDFC(0.8, 0.0, 0.0)
    @JvmField
    var smalPPX = PDFC(0.012, 0.0, 0.0)
    @JvmField
    var smalPPY = PDFC(0.012, 0.0, 0.0)
    @JvmField
    var smalPPH = PDFC(0.5, 0.0, 0.15)

    @JvmField
    var anti_ch_fa: Double = 0.17
    @JvmField
    var anti_eh_fa: Double = 0.4

    @JvmField
    var retardationTimer = 0.00

    @JvmField
    var PeruMin = 0.2
    @JvmField
    var PeruMinTime = 0.22
    @JvmField
    var PeruMinAngCoef = 0.8
    @JvmField
    var PeruMax = sqrt(2.0)
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