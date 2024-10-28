package org.firstinspires.ftc.teamcode.WRAPPERS.THREADEDIMU

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.WRAPPERS.THREADEDIMU.threadedimu_vars.imu_offset
import org.firstinspires.ftc.teamcode.WRAPPERS.THREADEDIMU.threadedimu_vars.initialized
import org.firstinspires.ftc.teamcode.WRAPPERS.THREADEDIMU.threadedimu_vars.running

class ThreadedIMU(name: String) {
    private val imu: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, name)

    var yaw: Double = 0.0
    var localizeraccessed: Boolean = false


    fun resetYaw(d: Double) {
        imu_offset = -yaw + d
    }

    val ep = ElapsedTime()

    private class imew(val threadedimew: ThreadedIMU) : Runnable {
        override fun run() {
            val ep = ElapsedTime()
            ep.reset()
            while (running) {
                val fixed = threadedimew.imu.angularOrientation
                val y = fixed.firstAngle.toDouble()
                threadedimew.yaw = y
                threadedimew.localizeraccessed = false
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