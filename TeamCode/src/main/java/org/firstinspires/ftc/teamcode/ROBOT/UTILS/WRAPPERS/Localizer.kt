package org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.freakyyyy.angScalar
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.freakyyyy.linearScalar

@Config
object freakyyyy{
    @JvmField
    var angScalar: Double = 1.002
    @JvmField
    var linearScalar: Double = 1.0075
}
class Localizer(name: String) {
    private val otos = hardwareMap.get(SparkFunOTOS::class.java, name)
    var pose: Pose = Pose()
    init {
        otos.resetTracking()
        otos.initialize()
        otos.resetTracking()
        otos.calibrateImu()
        otos.linearScalar = linearScalar
        otos.angularScalar = angScalar
        otos.angularUnit = AngleUnit.RADIANS
        otos.linearUnit = DistanceUnit.CM
        otos.offset.set(SparkFunOTOS.Pose2D(0.0, 0.0, 0.0))
    }
    fun update() { pose = Pose(otos.position) }
    fun reset() = otos.resetTracking()
}