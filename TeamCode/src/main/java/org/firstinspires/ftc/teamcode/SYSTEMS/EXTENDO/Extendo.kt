package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.WRAPPERS.MOTOR

class Extendo {

    var chub_rails = MOTOR("CHUB_RAIL", true, false)
    var ehub_rails = MOTOR("EHUB_RAIL", true, true)

}