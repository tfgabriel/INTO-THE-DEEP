package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.REVERSE_LIFT
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.USE_LIFT
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.liftl_first_open
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.liftr_first_open
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.MOTOR

class Lift {
    var chub_slides = MOTOR("CHUB_SLIDE", true, !REVERSE_LIFT, liftl_first_open, USE_LIFT)
    var ehub_slides = MOTOR("EHUB_SLIDE", true, REVERSE_LIFT, liftr_first_open, USE_LIFT)
    var intolerance = false
}