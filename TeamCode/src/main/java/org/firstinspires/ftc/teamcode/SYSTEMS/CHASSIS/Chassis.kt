package org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.denominator
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.imu_offset
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.slowcoef
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.slowdown
import org.firstinspires.ftc.teamcode.WRAPPERS.MOTOR
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class Chassis {
    var leftfront = MOTOR("LF", false, true)
    var leftback = MOTOR("LB", false, true)
    var rightfront = MOTOR("RF", false, false)
    var rightback = MOTOR("RB", false, false)

    fun rc_drive(y: Double, x: Double, rx: Double, slow: Double){
        slowdown = (1 * slowcoef) * (1 - slow)

        denominator = max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftfront.power = (y - x + rx) * slowdown/ denominator
        leftback.power = (y + x + rx) * slowdown / denominator
        rightfront.power = (y + x - rx) * slowdown / denominator
        rightback.power = (y - x - rx) * slowdown / denominator
    }

    fun fc_drive(y: Double, x: Double, rx: Double, slow: Double){
        val h = imew.yaw
        rc_drive(cos(h + imu_offset)*y - sin(h)*x, sin(h + imu_offset)*y + cos(h)*x, rx, slow)
    }
}