package org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS

import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.anti_p
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.denominator
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.imu_offset
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.slowcoef
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.slowdown
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.MOTOR
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class Chassis {
    var leftfront = MOTOR("LF", false, true)
    var leftback = MOTOR("LB", false, true)
    var rightfront = MOTOR("RF", false, false)
    var rightback = MOTOR("RB", false, false)

    fun rc_drive(y: Double, x: Double, rx: Double, slow: Double) {
        var move = Vec2D(x, y)
        move = move.rotate(rx * anti_p)
        slowdown = (1 * slowcoef) * (1 - slow)

        denominator = max(abs(move.y) + abs(move.x) + abs(rx), 1.0);

        leftfront.power = (move.y - move.x + rx) * slowdown / denominator
        leftback.power = (move.y + move.x + rx) * slowdown / denominator
        rightfront.power = (move.y + move.x - rx) * slowdown / denominator
        rightback.power = (move.y - move.x - rx) * slowdown / denominator
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
}