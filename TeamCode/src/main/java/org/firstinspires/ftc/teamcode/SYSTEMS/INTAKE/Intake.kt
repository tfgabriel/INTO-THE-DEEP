package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.pos_diff
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.WRAPPERS.CR_SERVO
import org.firstinspires.ftc.teamcode.WRAPPERS.CServo

class Intake {
    var chub_arm = CServo("CHUB_ARM_INTAKE") // 0 - to submbersible, 1 - to exam zone
    var ehub_arm = CServo("EHUB_ARM_INTAKE", true) // 1 - to submersible, 0 - to exam zone

    var wrist = CServo("WRIST_INTAKE")
    var fourbar = CServo("fourbar")

    var claws = CServo("CLAWS")

}