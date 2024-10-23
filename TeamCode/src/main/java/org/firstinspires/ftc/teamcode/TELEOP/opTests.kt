package org.firstinspires.ftc.teamcode.TELEOP

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot

@TeleOp
class opTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(false)
        robot.start()
    }
}