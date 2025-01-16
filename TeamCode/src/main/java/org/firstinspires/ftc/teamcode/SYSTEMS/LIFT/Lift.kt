package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.liftl_first_open
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.liftr_first_open
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.MOTOR

class Lift {
    var chub_slides = MOTOR("CHUB_SLIDE", true, true, liftl_first_open)
    var ehub_slides = MOTOR("EHUB_SLIDE", true, false, liftr_first_open)
    var intolerance = false
}