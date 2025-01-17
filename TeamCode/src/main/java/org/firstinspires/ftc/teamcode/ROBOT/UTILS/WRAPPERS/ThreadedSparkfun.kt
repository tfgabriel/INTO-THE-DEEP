package org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_norm
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.freakyyyy.angScalar
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.freakyyyy.linearScalar
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall

class ThreadedSparkfun(val name: String) {

    private val otos = hardwareMap.get(SparkFunOTOS::class.java, name)
    var updTime = 0.0
    var pose: Pose = Pose()
    val ep = ElapsedTime()
    var toReset = true

    var running: Boolean = false
    var initialized: Boolean = false

    private class ispark(val ispark: ThreadedSparkfun) : Runnable {
        override fun run() {
            val ep = ElapsedTime()
            ep.reset()
            while (ispark.running) {
                if (ispark.toReset) {
                    ispark.otos.resetTracking()
                    ispark.otos.resetTracking()
                    ispark.toReset = false
                }
                ispark.pose = Pose(ispark.otos.position)
                ispark.updTime = ep.seconds()
                ep.reset()
            }
        }
    }

    var thread = Thread()
    fun init() {
        ep.reset()
        otos.resetTracking()
        otos.initialize()
        otos.resetTracking()
        otos.calibrateImu()
        otos.linearScalar = linearScalar
        otos.angularScalar = angScalar
        otos.angularUnit = AngleUnit.RADIANS
        otos.linearUnit = DistanceUnit.CM
        otos.offset.set(SparkFunOTOS.Pose2D(0.0, 0.0, 0.0))
        initialized = true
        ep.reset()

        thread = Thread(ispark(this))
        thread.setUncaughtExceptionHandler { th, er -> send_toall("GOT ERR Ispark ${th.id}", er.stackTraceToString()) }
    }

    fun initThread() {
        if (!running) {
            running = true
            thread.start()
        }
    }

    fun closeThread() {
        if (running) {
            running = false
        }
    }

    fun close() {
        closeThread()
        if (initialized) {
            initialized = false
        }
    }

    fun reset() {
        close()
        init()
        initThread()
    }
}
