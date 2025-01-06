package org.firstinspires.ftc.teamcode.SYSTEMS.ACTIVE_INTAKE

import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.CR_SERVO
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.CServo

class ActiveIntake {
    val chub_intaker = CR_SERVO("CHUB_INTAKER", false, false)
    val ehub_intaker = CR_SERVO("EHUB_INTAKER", true, false)

    val chub_arm = CServo("CHUB_ARM", false)
    val ehub_arm = CServo("EHUB_ARM", true)
}