package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.WRAPPERS.CR_SERVO

class Intake {
    var chub_arm = hardwareMap.servo.get("CHUB_ARM_INTAKE")
    var ehub_arm = hardwareMap.servo.get("EHUB_ARM_INTAKE")

    var wrist = hardwareMap.servo.get("WRIST_INTAKE")

    var chub_intaker = CR_SERVO("CHUB_INTAKER", false)
    var ehub_intaker = CR_SERVO("EHUB_INTAKER", true)
}