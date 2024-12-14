package org.firstinspires.ftc.teamcode.WRAPPERS

import com.outoftheboxrobotics.photoncore.Photon
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap

class CR_SERVO(name: String, reversed: Boolean) {
    val crservo = hardwareMap.get(CRServo::class.java, name) as PhotonCRServo
    //val crservo = hardwareMap.crservo.get(name)
    init{
        if(reversed)
            crservo.direction = DcMotorSimple.Direction.REVERSE
    }

    var power: Double = 0.0
        set(v) {
            if (v != field) {
                crservo.power = v
                field = v
            }
        }
}

class SERVO(name: String, reversed: Boolean){
    val servo = hardwareMap.get(Servo::class.java, name) as PhotonServo

    var position: Double = 0.0
        get(){
            return servo.position
        }
        set(v){
            if (v != field) {
                servo.position = v
                field = v
            }
        }
}