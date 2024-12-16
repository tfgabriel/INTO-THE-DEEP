package org.firstinspires.ftc.teamcode.TELEMETRY

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.field_scale
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.robot_radius

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

    fun drawRobot(canvas: Canvas, p: Pose, tcol: String) {
        canvas.setStrokeWidth(2)
        canvas.setStroke(tcol)
        val cp = p * field_scale
        canvas.strokeCircle(cp.x, cp.y, robot_radius)
    }
}