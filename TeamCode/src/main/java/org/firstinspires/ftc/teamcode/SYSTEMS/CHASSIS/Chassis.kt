package org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS

import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.P1
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.P2
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.P3
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.P4
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.PIMew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.PTurn
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.XC
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.YC
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.linearopmode
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.anti_ch_fa
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.anti_eh_fa
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.MOTOR
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.chassis_f
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.denominator
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.imu_offset
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.slowcoef
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.slowdown
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

class Chassis {
    var leftfront = MOTOR("LF", false, true)
    var leftback = MOTOR("LB", false, true)
    var rightfront = MOTOR("RF", false, false)
    var rightback = MOTOR("RB", false, false)

    private fun loosySign(v: Double) = if (abs(v) < 0.001) 0.0 else if (v > 0) 1.0 else -1.0

    fun rc_drive(y: Double, x: Double, rx: Double, slow: Double) {
        var move = Vec2D(x, y)
        move = move.rotate(rx * (if (rx < 0) anti_ch_fa else anti_eh_fa))
        slowdown = (1 * slowcoef) * (1 - slow)

        denominator = max(abs(move.y) + abs(move.x) + abs(rx), 1.0);

        move *= 1 - rx * PTurn

        val lfp = (move.y - move.x + rx) * slowdown / denominator
        val lbp = (move.y + move.x + rx) * slowdown / denominator
        val rfp = (move.y + move.x - rx) * slowdown / denominator
        val rbp = (move.y - move.x - rx) * slowdown / denominator
        leftfront.power = lfp + loosySign(lfp) * chassis_f[0]
        leftback.power = lbp + loosySign(lbp) * chassis_f[1]
        rightfront.power = rfp + loosySign(rfp) * chassis_f[2]
        rightback.power = rbp + loosySign(rbp) * chassis_f[3]
    }

    fun rc_drive(move: Vec2D, rx: Double, slow: Double) {
        slowdown = (1 * slowcoef) * (1 - slow)

        denominator = max(abs(move.y) + abs(move.x) + abs(rx), 1.0);

        leftfront.power = (move.y - move.x + rx) * slowdown / denominator
        leftback.power = (move.y + move.x + rx) * slowdown / denominator
        rightfront.power = (move.y + move.x - rx) * slowdown / denominator
        rightback.power = (move.y - move.x - rx) * slowdown / denominator
    }

    fun fc_drive(y: Double, x: Double, rx: Double, slow: Double) {
        val h = imew.yaw
        rc_drive(
            cos(h + imu_offset) * y - sin(h) * x,
            sin(h + imu_offset) * y + cos(h) * x,
            rx,
            slow
        )
    }

    fun sebidrive() {
        val speed = hypot(
            linearopmode.gamepad1.left_stick_x * XC,
            linearopmode.gamepad1.left_stick_y * YC
        )
        val turn = -linearopmode.gamepad1.right_stick_x.toDouble()
        val angle = atan2(
            linearopmode.gamepad1.left_stick_y * YC,
            linearopmode.gamepad1.left_stick_x * XC
        ) - Math.PI / 4 + imew.yaw * PIMew + turn * PTurn
        val ms = speed * sin(angle)
        val mc = speed * cos(angle)

        val lfPower = ms + turn
        val rfPower = mc - turn
        val lbPower = mc + turn
        val rbPower = ms - turn

        val spcoef: Double = 1 - 0.65 * linearopmode.gamepad1.right_trigger
        leftfront.power = lfPower * spcoef * P1
        rightfront.power = rfPower * spcoef * P2
        leftback.power = lbPower * spcoef * P3
        rightback.power = rbPower * spcoef * P4
    }
}