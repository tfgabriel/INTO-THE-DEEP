package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.WRAPPERS.CServo

class Outtake {
//    var chub_arm = hardwareMap.servo.get("CHUB_ARM_OUTTAKE")
    var chub_arm = CServo("NOTHING")
    var ehub_arm = CServo("EHUB_ARM_OUTTAKE")

    var positioner = CServo("POSITIONER")
    //var wrist = hardwareMap.servo.get("WRIST_OUTTAKE")

    var chub_claw = CServo("CHUB_CLAW")
    var ehub_claw = CServo("EHUB_CLAW")
}