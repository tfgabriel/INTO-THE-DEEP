package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.CServo


class Outtake {
    var chub_arm = CServo("CHUB_ARM_OUTTAKE")
    var ehub_arm = CServo("EHUB_ARM_OUTTAKE", true)

    var fourbar = CServo("FOURBAR")
    var claw = CServo("CLAW")
}