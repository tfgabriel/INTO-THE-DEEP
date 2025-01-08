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
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCFinalH
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCFinalX
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCFinalY
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCH
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCX
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCY
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMax
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMin
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.angular_tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.speed_limit_angular
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.speed_limit_linear
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.tolerance
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

    var robot_vel = Pose()


    fun followpath(current_path: Pose) {
        xPDF = PDF(PDFCX)
        yPDF = PDF(PDFCY)
        hPDF = PDF(PDFCH)
        xFinalPDF = PDF(PDFCFinalX)
        yFinalPDF = PDF(PDFCFinalY)
        hFinalPDF = PDF(PDFCFinalH)
        done = false
        path = current_path
        target_pose = current_path
        start_pose =
            Pose(localizer.pose.x, localizer.pose.y, angNorm(localizer.pose.h), current_path.vel)
    }


    fun isBotinTolerance() = err.distance() < tolerance
            && abs(err.h) < angular_tolerance
            && abs(robot_vel.distance()) < speed_limit_linear
            && robot_vel.h < speed_limit_angular

    private fun getPeruCoef(dist: Double) = clamp(
        (dist - target_pose.decelPose.x) * (PeruMax - PeruMin) / (target_pose.decelPose.y - target_pose.decelPose.x) + PeruMin,
        PeruMin,
        PeruMax
    )

    fun update() {
        current_pos = localizer.pose + Pose(0.0, 0.0, 0.0, target_pose.vel)
        robot_vel = localizer.vel
        err = target_pose - current_pos
        send_toall("err y", err.y)
        send_toall("err x", err.x)
        send_toall("err h", err.h)

        err = err.rotate(-current_pos.h)
        err.h = ang_diff(current_pos.h, target_pose.h)
        // If the robot is not in tolerance, run with the pd
        if (!isBotinTolerance()) {



            val dist = err.distance()
            val peruCoef = getPeruCoef(err.distance())

            val move = clamp(
                (if (dist < target_pose.goodEnough)
                Vec2D(
                    clamp(xFinalPDF.update(-err.x) * target_pose.vel, -1.0, 1.0),
                    clamp(-yFinalPDF.update(err.y) * target_pose.vel, -1.0, 1.0)
                )
                else
                Vec2D(
                    clamp(xPDF.update(-err.x) * target_pose.vel, -1.0, 1.0),
                    clamp(-yPDF.update(err.y) * target_pose.vel, -1.0, 1.0)
                )),

                -peruCoef,
                peruCoef)
            send_toall("2PeruCoef", peruCoef)


            send_toall("2movex", move.x)
            send_toall("2movey", move.y)
            send_toall("2moveh", if (dist < target_pose.goodEnough) -hFinalPDF.update(err.h) else -hPDF.update(err.h))

            chassis.rc_drive(
                move.y, move.x,
                if (dist < target_pose.goodEnough) -hFinalPDF.update(err.h) else -hPDF.update(err.h),
                0.0
            )
            ep.reset()
        } else {
            // else, run with the pd in the opposite direction in order to stop it and counteract the slip for a bit then stop
            if (ep.milliseconds() < 10) {
                chassis.rc_drive(
                    PDFCY.f * sign(err.y),
                    -PDFCX.f * sign(-err.x),
                    PDFCH.f * sign(err.h),
                    0.0
                )
                done = true
            }
            else {
                chassis.rc_drive(0.0, 0.0, 0.0, 0.0)
                done = true
            }
        }


        send_toall("currpos", current_pos)
        send_toall("is in tol", isBotinTolerance())
        send_toall("isin lin tol", err.distance() < tolerance)
        send_toall("is in ang tol", abs(err.h) < angular_tolerance)
        send_toall("idiot speed", localizer.vel)

    }
}