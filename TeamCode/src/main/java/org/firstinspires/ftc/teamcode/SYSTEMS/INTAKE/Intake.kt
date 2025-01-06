package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.CServo

class Intake {
    var chub_arm = CServo("CHUB_ARM_INTAKE") // 0 - to submbersible, 1 - to exam zone
    var ehub_arm = CServo("EHUB_ARM_INTAKE", true) // 1 - to submersible, 0 - to exam zone

    var wrist = CServo("WRIST_INTAKE")
    var fourbar = CServo("fourbar",)

    var claws = CServo("CLAWS")

}