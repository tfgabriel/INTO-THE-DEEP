package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap

class Outtake {

    var chub_arm = hardwareMap.servo.get("CHUB_ARM_OUTTAKE")
    var ehub_arm = hardwareMap.servo.get("EHUB_ARM_OUTTAKE")

    var positioner = hardwareMap.servo.get("POSITIONER")
    var wrist = hardwareMap.servo.get("WRIST_OUTTAKE")

    var chub_claw = hardwareMap.servo.get("CHUB_CLAW")
    var ehub_claw = hardwareMap.servo.get("EHUB_CLAW")

}