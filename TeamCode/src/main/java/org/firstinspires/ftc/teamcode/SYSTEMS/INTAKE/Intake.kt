package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.pos_diff
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.WRAPPERS.CR_SERVO

class Intake {
    var chub_arm = hardwareMap.servo.get("CHUB_ARM_INTAKE") // 0 - to submbersible, 1 - to exam zone
    var ehub_arm = hardwareMap.servo.get("EHUB_ARM_INTAKE") // 1 - to submersible, 0 - to exam zone

    var wrist = hardwareMap.servo.get("WRIST_INTAKE")
    var fourbar = hardwareMap.servo.get("fourbar")

    var claws = hardwareMap.servo.get("CLAWS")

}