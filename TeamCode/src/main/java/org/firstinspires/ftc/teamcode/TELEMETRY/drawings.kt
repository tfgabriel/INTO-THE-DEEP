package org.firstinspires.ftc.teamcode.TELEMETRY

import com.acmerobotics.dashboard.canvas.Canvas
import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.ALGORITHMS.VecVec2D
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.field_scale
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.robot_color
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.robot_radius
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.robot_width
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.sample_diagonal
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.sample_outline
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.trajectory_color
import java.lang.Math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 *  ____                    _     _     _  _____
 * |  _ \  ___  _ __   __ _| | __|     | ||_   __ __ _   _ _ __ ___  _ __
 * | | | |/ _ \| '_ \ / _` | |/ _`  _  | |  | || '__| | | | '_ ` _ \| '_ \
 * | |_| | (_) | | | | (_| | | (_| | |_| _  | || |  | |_| | | | | | | |_) |
 * |____/ \___/|_| |_|\__,_|_|\__,_ \___(_  |_||_|   \__,_|_| |_| |_| .__/
 *                                                                  |_|
 */

object drawings {
    private fun drawVector(canvas: Canvas, pos: Pose, v: Pose, col: String, sz: Int, sc: Double) {
        canvas.setStrokeWidth(sz)
        canvas.setStroke(col)
        canvas.strokeLine(
            pos.x * field_scale,
            pos.y * field_scale,
            pos.x * field_scale + v.x * sc,
            pos.y * field_scale + v.y * sc
        )
    }

    private fun draw_line(canvas: Canvas, p1: Vec2D, p2: Vec2D, scale: Double) {
        canvas.strokeLine(p1.x * scale, p1.y * scale, p2.x * scale, p2.y * scale)
    }

    private fun draw_rect(
        canvas: Canvas,
        ptVec: VecVec2D,
        color: String,
        color2: String,
        size: Int,
        scale: Double,
        angle: Double
    ) {
        canvas.setStrokeWidth(size)
        canvas.setStroke(color)
        ptVec.rotate(angle)
        draw_line(canvas, ptVec[0], ptVec[1], scale)
        draw_line(canvas, ptVec[1], ptVec[2], scale)
        draw_line(canvas, ptVec[2], ptVec[3], scale)
        draw_line(canvas, ptVec[3], ptVec[0], scale)

        canvas.setStroke(color2)
        draw_line(canvas, ptVec[0], ptVec[2], scale)
        draw_line(canvas, ptVec[1], ptVec[3], scale)
    }

    fun draw_sample(canvas: Canvas, result: LLResult) {
        if (result.colorResults.isNotEmpty()) {
            draw_rect(
                canvas,
                VecVec2D(result.colorResults[0].targetCorners),
                sample_outline,
                sample_diagonal,
                1,
                field_scale * 1 / 2,
                PI / 2
            )
        }
    }

    fun corr(p: Pose): Pose {
        return p.rotate(-PI / 2)
    }

    fun drawRobot(canvas: Canvas, p: Pose) {
        canvas.setStrokeWidth(robot_width)
        canvas.setStroke(robot_color)
        val cp = corr(p)
        cp.h = p.h
        canvas.strokeCircle(cp.x * field_scale, cp.y * field_scale, robot_radius)
        drawVector(
            canvas,
            cp,
            Pose(cos(p.h), sin(p.h), 0.0, 0.0),
            robot_color,
            robot_width,
            robot_radius
        )
    }

    fun drawCircle(canvas: Canvas, r: Double, p: Pose, col: String) {
        canvas.setStrokeWidth(1)
        canvas.setStroke(col)
        canvas.strokeCircle(p.x * field_scale, p.y * field_scale, r * field_scale)
    }

    fun drawP2P(canva: Canvas) {
        val misc = p2p.target_pose - p2p.start_pose
        val farSlowPoint = misc * p2p.decelDistanceFar
        val closeSlowPoint = misc * p2p.decelDistanceClose

        drawVector(
            canva,
            corr(p2p.start_pose), corr(misc),
            trajectory_color, 1,
            field_scale
        )

        /*
        drawVector(
            canva,
            corr(p2p.start_pose), corr(farSlowPoint),
            trajectory_color, 1,
            field_scale
        )

        drawVector(
            canva,
            corr(p2p.start_pose + farSlowPoint), corr(closeSlowPoint - farSlowPoint),
            "#FF0000", 1,
            field_scale
        )

        drawVector(
            canva,
            corr(p2p.start_pose + closeSlowPoint), corr(p2p.target_pose - closeSlowPoint),
            trajectory_color, 1,
            field_scale
        )*/

        drawCircle(
            canva, p2p.closeEnoughTM, corr(p2p.target_pose), "#010805"
        )

    }
}