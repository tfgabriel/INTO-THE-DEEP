package org.firstinspires.ftc.teamcode.TELEOP

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_claw_testing
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_claw_testing
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_testing
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.wrist_testing

// @Photon
@TeleOp
class opTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)
        waitForStart()

        while(!isStopRequested){
            outtake.wrist.position = wrist_testing
            outtake.positioner.position = positioner_testing
            outtake.ehub_claw.position = ehub_claw_testing
            outtake.chub_claw.position = chub_claw_testing
            robot.update()
        }
    }
}