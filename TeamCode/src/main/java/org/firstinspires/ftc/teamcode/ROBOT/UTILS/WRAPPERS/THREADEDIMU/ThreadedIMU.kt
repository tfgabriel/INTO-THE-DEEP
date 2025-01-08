package org.firstinspires.ftc.teamcode.Systems

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_norm
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall

class ThreadedIMU(val name: String) {
    private val imu: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, name)
    var imuoffset = 0.0

    var running: Boolean = false
    var initialized: Boolean = false

    val yaw: Double
        get() = ang_norm(_yaw + imuoffset)
    private var _yaw = 0.0
    var yawVel: Double = 0.0
    var localizerAccessed = false

    fun resetYaw(d: Double) {
        imuoffset = -_yaw + d
    }



    val ep = ElapsedTime()

    private class imew(val threadedimew: ThreadedIMU) : Runnable {
        override fun run() {
            val ep = ElapsedTime()
            ep.reset()
            while (threadedimew.running) {
                val fixed = threadedimew.imu.angularOrientation
                val y = fixed.firstAngle.toDouble()
                threadedimew._yaw = y
                threadedimew.localizerAccessed = false
                ep.reset()
            }
        }
    }

    var thread = Thread()
    fun init() {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        initialized = true
        ep.reset()

        thread = Thread(imew(this))
        thread.setUncaughtExceptionHandler { th, er -> send_toall("GOT ERR TIMMY ${th.id}", er.stackTraceToString()) }

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
            imu.close()
            initialized = false
        }
    }

    fun reset(){
        close()
        init()
        initThread()
    }
}
