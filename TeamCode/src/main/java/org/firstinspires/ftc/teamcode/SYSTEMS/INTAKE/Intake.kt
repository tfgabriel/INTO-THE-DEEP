package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.WRAPPERS.CR_SERVO

class Intake {
    //var chub_arm = hardwareMap.servo.get("CHUB_ARM_INTAKE") //0 - to submbersible, 1 - to exam zone
    //var ehub_arm = hardwareMap.servo.get("EHUB_ARM_INTAKE") //1 - to submersible, 0 - to exam zone

    //var wrist = hardwareMap.servo.get("WRIST_INTAKE")

    var chub_intaker = CR_SERVO("CHUB_INTAKER", true)
    //var ehub_intaker = CR_SERVO("EHUB_INTAKER", true)
}