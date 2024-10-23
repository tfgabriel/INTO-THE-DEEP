package org.firstinspires.ftc.teamcode.WRAPPERS

import com.outoftheboxrobotics.photoncore.Photon
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap

@Photon
class CR_SERVO(name: String, reversed: Boolean) {
    val crservo: PhotonCRServo = hardwareMap.get(CRServo::class.java, name) as PhotonCRServo

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