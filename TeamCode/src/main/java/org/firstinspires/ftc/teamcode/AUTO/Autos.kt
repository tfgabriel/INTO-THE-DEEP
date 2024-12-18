package org.firstinspires.ftc.teamcode.AUTO

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.moving
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.targetPos
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p

@Config
object AutoTestVars {
    @JvmField
    var targetPos = Pose(0.0, 0.0, 0.0)
    @JvmField
    var moving = true
}

@Autonomous
class AutoTest: LinearOpMode() {
    @SuppressLint("DefaultLocale")
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        waitForStart()
        localizer.reset()
        var zerounu = 0
        p2p.followpath(targetPos)
        while (!isStopRequested){
            if (p2p.done) {
                if (zerounu == 0) {
                    p2p.followpath(Pose())
                    zerounu = 1
                } else {
                    p2p.followpath(targetPos)
                    zerounu = 0
                }
            }
            if (moving) {
                p2p.update()
            }
            robot.update()
        }
    }
}