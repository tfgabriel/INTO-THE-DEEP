package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo_first_open
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.MOTOR

class Extendo {
    var chub_rails = MOTOR("CHUB_RAIL", true, true, extendo_first_open)
    //var ehub_rails = MOTOR("EHUB_RAIL", true, true)
}