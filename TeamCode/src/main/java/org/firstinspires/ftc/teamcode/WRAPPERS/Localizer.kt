package org.firstinspires.ftc.teamcode.WRAPPERS

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.angNorm
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import kotlin.math.PI

class Localizer(name: String) {
    private val otos = hardwareMap.get(SparkFunOTOS::class.java, name)
    var pose: Pose = Pose()

    init {
        otos.resetTracking()
        otos.initialize()
        otos.resetTracking()
        otos.calibrateImu()
        otos.angularUnit = AngleUnit.RADIANS
        otos.linearUnit = DistanceUnit.CM
        otos.linearScalar = 1.0054
        otos.angularScalar = 1.0075
        otos.offset.set(SparkFunOTOS.Pose2D(0.4, 0.1, 0.0))
    }

    fun update() { pose = Pose(otos.position) }

    fun reset() = otos.resetTracking()
}