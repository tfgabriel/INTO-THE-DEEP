package org.firstinspires.ftc.teamcode.LOCALIZATION

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.teamcode.ALGORITHMS.Array
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall

// Not in use :()
class Sparkfun {
    val sparkfun: SparkFunOTOS = hardwareMap.get(SparkFunOTOS::class.java, "sparkfun")

    val x: Double
        get(){
            return sparkfun.position.x
        }

    val y: Double
        get(){
            return sparkfun.position.y
        }

    val h: Double
        get(){
            return sparkfun.position.h
        }

    val pos: Pose
        get(){
            return Pose(x, y, h)
        }

    var scalars: Array = Array()
        get(){
            return Array(sparkfun.linearScalar, sparkfun.angularScalar)
        }
        set(value){
            field = value
        }

    var offsets: Array = Array()
        get(){
            return Array(sparkfun.offset.x, sparkfun.offset.y, sparkfun.offset.h)
        }
        set(value){
            field = value
        }

    fun start(){
        if(!sparkfun.begin())
            send_toall("ERROR", "SPARKFUN UNABLE TO OPEN")
    }
}