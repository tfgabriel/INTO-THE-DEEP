package org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.freakyyyy.angScalar
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.freakyyyy.linearScalar
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall

class LocalizerT(name: String) {
    private val ts = ThreadedSparkfun(name)

    var pose: Pose = Pose()
    var lpose: Pose = Pose()
    val ep = ElapsedTime()
    var vel: Pose = Pose()

    init {
        ts.init()
        ts.initThread()
    }

    fun update() {
        val et = ElapsedTime()
        et.reset()
        lpose = pose
        pose = ts.pose
        vel = (pose - lpose) / ep.seconds()
        ep.reset()
        send_toall("UPDATE SPARKFUN TIME", et.seconds())
    }
    fun reset() { ts.toReset = true }
}
