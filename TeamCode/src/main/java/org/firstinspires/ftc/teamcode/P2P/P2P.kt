package org.firstinspires.ftc.teamcode.P2P

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.angNorm
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.angular_tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.slow
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_p
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

class P2P {
    var target_pose = Pose()
    var start_pose = Pose()
    var current_pos = Pose()
    var err = Pose()

    var xPDF: PDF = PDF()
    var yPDF: PDF = PDF()
    var hPDF: PDF = PDF()

    val ep: ElapsedTime = ElapsedTime()

    var done: Boolean = false

    fun followpath(current_path: Pose) {
        xPDF = PDF(x_p, x_d, x_f)
        yPDF = PDF(y_p, y_d, y_f)
        hPDF = PDF(h_p, h_d, h_f)
        done = false

        target_pose = current_path
        start_pose = Pose(localizer.pose.x, localizer.pose.y, angNorm(localizer.pose.h))
    }

    fun isBotinTolerance() = err.distance() < tolerance && abs(err.h) < angular_tolerance

    fun update() {
        current_pos = Pose(localizer.pose.x, localizer.pose.y, angNorm(localizer.pose.h))
        err = target_pose - current_pos

        err = err.rotate(current_pos.h)
        err.h = ang_diff(current_pos.h, target_pose.h)
        // If the robot is not in tolerance, run with the pd
        if (!isBotinTolerance()) {
            chassis.rc_drive(
                yPDF.update(err.y) * slow,
                xPDF.update(-err.x),
                hPDF.update(err.h),
                0.0
            )
            ep.reset()
        } else {
            // else, run with the pd in the opposite direction in order to stop it and counteract the slip for a bit then stop
            if (ep.milliseconds() < 50)
                chassis.rc_drive(x_f * sign(err.y), y_f * sign(err.x), h_f * sign(err.h), 0.0)
            else {
                chassis.rc_drive(0.0, 0.0, 0.0, 0.0)
                done = true
            }
        }

    }
}