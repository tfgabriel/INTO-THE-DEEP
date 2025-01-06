package org.firstinspires.ftc.teamcode.P2P

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.angNorm
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.clamp
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMax
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMin
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.angular_tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_p
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import kotlin.math.abs
import kotlin.math.sign

class P2P {
    var target_pose = Pose()
    var start_pose = Pose()
    var current_pos = Pose()
    var err = Pose()

    var path = Pose()

    var xPDF: PDF = PDF()
    var yPDF: PDF = PDF()
    var hPDF: PDF = PDF()
    var xFinalPDF: PDF = PDF()
    var yFinalPDF: PDF = PDF()
    var hFinalPDF: PDF = PDF()
    val ep: ElapsedTime = ElapsedTime()
    var done: Boolean = false

    var decelDistanceFar: Double = 0.0
    var decelDistanceClose: Double = 0.0
    var closeEnoughTM: Double = 0.0

    fun followpath(current_path: Pose, closeEnough: Double, decelFar: Double, decelClose: Double) {
        xPDF = PDF(x_p, x_d, x_f)
        yPDF = PDF(y_p, y_d, y_f)
        hPDF = PDF(h_p, h_d, h_f)
        xFinalPDF = PDF(x_p, x_d, x_f)
        yFinalPDF = PDF(y_p, y_d, y_f)
        hFinalPDF = PDF(h_p, h_d, h_f)
        done = false
        path = current_path
        decelDistanceFar = decelFar
        decelDistanceClose = decelClose
        closeEnoughTM = closeEnough
        target_pose = current_path
        start_pose =
            Pose(localizer.pose.x, localizer.pose.y, angNorm(localizer.pose.h), current_path.vel)
    }

    fun followpath(current_path: Pose) = followpath(current_path, 20.0, 0.00000001, 0.0)

    fun isBotinTolerance() = err.distance() < tolerance && abs(err.h) < angular_tolerance

    private fun getPeruCoef(dist: Double) = clamp(
        (dist - decelDistanceFar) * (PeruMax - PeruMin) / (decelDistanceClose - decelDistanceFar) + PeruMin,
        PeruMin,
        PeruMax
    )

    fun update() {
        current_pos = localizer.pose + Pose(0.0, 0.0, 0.0, target_pose.vel)
        err = target_pose - current_pos

        err = err.rotate(-current_pos.h)
        err.h = ang_diff(current_pos.h, target_pose.h)
        // If the robot is not in tolerance, run with the pd
        if (!isBotinTolerance()) {

            //var move_pdf_val = Vec2D(-yPDF.update(err.y) * target_pose.vel, xPDF.update(-err.x) * target_pose.vel)
            //val h_pdf_val = -hPDF.update(err.h)
            //move_pdf_val = move_pdf_val.rotate(h_pdf_val * anti_p)

            val dist = err.distance()
            val move = (if (dist < closeEnoughTM)
                Vec2D(
                    -yFinalPDF.update(err.y) * target_pose.vel,
                    xFinalPDF.update(-err.x) * target_pose.vel
                )
            else
                Vec2D(
                    -yPDF.update(err.y) * target_pose.vel,
                    xPDF.update(-err.x) * target_pose.vel
                )) * getPeruCoef(err.distance())
            send_toall("movex", move.x)
            send_toall("movey", move.y)

            chassis.rc_drive(
                move.x, move.y,
                if (dist < closeEnoughTM) -hFinalPDF.update(err.h) else -hPDF.update(err.h),
                0.0
            )
            ep.reset()
        } else {
            // else, run with the pd in the opposite direction in order to stop it and counteract the slip for a bit then stop
            if (ep.milliseconds() < 10)
                chassis.rc_drive(y_f * sign(err.y), -x_f * sign(err.x), h_f * sign(err.h), 0.0)
            else {
                chassis.rc_drive(0.0, 0.0, 0.0, 0.0)
                done = true
            }
        }

        send_toall("currpos", current_pos)
        send_toall("err y", err.y)
        send_toall("err x", err.x)
        send_toall("err h", err.h)
        send_toall("is in tol", isBotinTolerance())
        send_toall("isin lin tol", err.distance() < tolerance)
        send_toall("is in ang tol", abs(err.h) < angular_tolerance)
        send_toall("velocity", target_pose.vel)

    }
}