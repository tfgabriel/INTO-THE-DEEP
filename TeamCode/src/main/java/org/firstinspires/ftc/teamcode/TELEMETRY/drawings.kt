package org.firstinspires.ftc.teamcode.TELEMETRY

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.field_scale
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.robot_color
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.robot_radius
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.robot_width
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
        canvas.strokeLine(pos.x * field_scale, pos.y * field_scale, pos.x * field_scale + v.x * sc, pos.y * field_scale + v.y * sc)
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
        drawVector(canvas, cp, Pose(cos(p.h), sin(p.h), 0.0), robot_color, robot_width, robot_radius)
    }
    fun drawP2P(canva: Canvas) {
        drawVector(
            canva,
            corr(p2p.start_pose),
            corr(p2p.target_pose - p2p.start_pose),
            trajectory_color,
            1,
            field_scale
        )
    }
}