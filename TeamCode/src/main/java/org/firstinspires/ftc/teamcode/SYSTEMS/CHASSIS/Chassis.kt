package org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.vars.denominator
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.vars.slowcoef
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.vars.slowdown
import org.firstinspires.ftc.teamcode.WRAPPERS.MOTOR
import kotlin.math.max

class Chassis {

    var leftfront = MOTOR("LF", false, false)
    var leftback = MOTOR("LB", false, false)
    var rightfront = MOTOR("RF", false, true)
    var rightback = MOTOR("RB", false, true)

    fun rc_drive(y: Double, x: Double, rx: Double, slow: Double){
        slowdown = (1 - slow) * slowcoef

        denominator = max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftfront.power = (-y + x + rx) / denominator
        leftback.power = (-y - x + rx) / denominator
        rightfront.power = (-y - x - rx) / denominator
        rightback.power = (-y + x - rx) / denominator
    }

    fun fc_drive(y: Double, x: Double, rx: Double, slow: Double){
        slowdown = (1 - slow) * slowcoef

        denominator = max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);


    }
}