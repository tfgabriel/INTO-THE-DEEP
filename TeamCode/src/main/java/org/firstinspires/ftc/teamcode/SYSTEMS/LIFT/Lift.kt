package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.WRAPPERS.MOTOR

class Lift {
    var chub_slides = MOTOR("CHUB_SLIDE", true, false)
    var ehub_slides = MOTOR("EHUB_SLIDE", true, true)
}