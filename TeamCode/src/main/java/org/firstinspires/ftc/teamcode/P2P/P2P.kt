package org.firstinspires.ftc.teamcode.P2P

import android.graphics.Color
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.angDiff
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.angNorm
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.clamp
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.pos_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.SQUID
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.HANDYMIN
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.control_hub
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.expansion_hub
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCFinalH
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCFinalX
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCFinalY
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCH
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCX
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PDFCY
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMax
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMin
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMinAngCoef
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.PeruMinTime
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.retardationTimer
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.setCol
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
    var xFinalPDF: SQUID = SQUID()
    var yFinalPDF: SQUID = SQUID()
    var hFinalPDF: PDF = PDF()
    val ep: ElapsedTime = ElapsedTime()
    var done: Boolean = false

    var robot_vel = Pose()

    var p2p_logid = 0

    fun followpath(current_path: Pose) {
        xPDF = PDF(PDFCX)
        yPDF = PDF(PDFCY)
        hPDF = PDF(PDFCH)
        val pfx = PDFCFinalX.duplicate()
        if (current_path.customff < 20.0) {
            pfx.f = current_path.customff
        }
        val pfy = PDFCFinalY.duplicate()
        if (current_path.customff < 20.0) {
            pfy.f = current_path.customff
        }
        xFinalPDF = SQUID(pfx)
        yFinalPDF = SQUID(pfy)
        hFinalPDF = PDF(PDFCFinalH)
        done = false
        path = current_path
        target_pose = current_path
        if (target_pose.decelPose.x > target_pose.decelPose.y) {
            send_toall("Invalid decel pose for ${target_pose.name}", target_pose.decelPose)
        }
        start_pose =
            Pose(localizer.pose.x, localizer.pose.y, angNorm(localizer.pose.h), current_path.vel)

        setCol(Color.rgb(255, 255, 255))
    }

    fun isBotinTolerance() = err.distance() < target_pose.tolerance[0]
            && abs(err.h) < target_pose.tolerance[1]
            && abs(robot_vel.distance()) < target_pose.tolerance[2]
            && robot_vel.h < target_pose.tolerance[3]

    private fun getPeruCoef(dist: Double) = clamp(
        (dist - target_pose.decelPose.x) * (PeruMax - PeruMin) / (target_pose.decelPose.y - target_pose.decelPose.x) + PeruMin,
        PeruMin,
        PeruMax
    )

    var hitPeruMin = false
    var et = ElapsedTime()

    fun fe(p: Pose) = String.format("%.2f %.2f %.2f", p.x, p.y, angDiff(p.h, 0.0))

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
            if (!hitPeruMin && pos_diff(peruCoef, PeruMin)) {
                hitPeruMin = true
                et.reset()
            }

            val move = clamp(
                (if (dist < target_pose.goodEnough) // Use Final pid when close enough
                Vec2D(
                    clamp(xFinalPDF.update(-err.x) * target_pose.vel, -1.0, 1.0),
                    clamp(-yFinalPDF.update(err.y) * target_pose.vel, -1.0, 1.0)
                )
                else // Use regular pid
                Vec2D(
                    clamp(xPDF.update(-err.x) * target_pose.vel, -1.0, 1.0),
                    clamp(-yPDF.update(err.y) * target_pose.vel, -1.0, 1.0)
                )),

                -peruCoef,
                peruCoef)
            send_toall("2PeruCoef", peruCoef)


            send_toall("2movex", move.x)
            send_toall("2movey", move.y)
            send_toall("2moveh", if (dist < target_pose.goodEnough) -hFinalPDF.update(err.h) else -hPDF.update(err.h)) /// Tmp

            if (hitPeruMin && et.seconds() < PeruMinTime) {
                chassis.rc_brake()
            } else {
                if(!path.is_headingonly) {

                    chassis.rc_drive(
                        move.y, move.x,
                        clamp(
                            if (dist < target_pose.goodEnough) -hFinalPDF.update(err.h) else -hPDF.update(
                                err.h
                            ), -peruCoef * PeruMinAngCoef, peruCoef * PeruMinAngCoef
                        ),
                        0.0
                    )
                }
                else{
                    var sebi = if(HANDYMIN)
                        PeruMin
                    else
                        PeruMax

                    val handicapped_move =
                        clamp(Vec2D(
                            clamp(xFinalPDF.update(-err.x) * target_pose.vel, -1.0, 1.0),
                            clamp(-yFinalPDF.update(err.y) * target_pose.vel, -1.0, 1.0)
                        ), -sebi,
                            sebi)

                    chassis.rc_drive(
                        handicapped_move.y, handicapped_move.x,
                        clamp(
                            if (dist < target_pose.goodEnough) -hFinalPDF.update(err.h) else -hPDF.update(
                                err.h
                            ), -peruCoef * PeruMinAngCoef, peruCoef * PeruMinAngCoef
                        ),
                        0.0
                    )
                }
            }
            ep.reset()
        } else {
            // else, run with the pd in the opposite direction in order to stop it and counteract the slip for a bit then stop
            if (ep.seconds() < retardationTimer) {
                chassis.rc_drive(
                    PDFCY.f * sign(err.y),
                    -PDFCX.f * sign(-err.x),
                    PDFCH.f * sign(err.h),
                    0.0
                )
                if (!done) {
                    send_toall("0errs $p2p_logid", "${fe(err)} ${target_pose.name}")
                    ++p2p_logid
                    setCol(Color.rgb(255, 0, 0))
                }
            }
            else {
                chassis.rc_drive(0.0, 0.0, 0.0, 0.0)
                if (!done) {
                    send_toall("0errs $p2p_logid", "${fe(err)} ${target_pose.name}")
                    ++p2p_logid
                    setCol(Color.rgb(255, 0, 0))
                }
                done = true
            }
        }


        send_toall("currpos", current_pos)
        send_toall("is in tol", isBotinTolerance())
        send_toall("is in lin tol", err.distance() < target_pose.tolerance[0])
        send_toall("is in ang tol", abs(err.h) < target_pose.tolerance[1])
        send_toall("idiot speed", localizer.vel)

    }
}
