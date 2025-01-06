package org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap

class CR_SERVO(name: String, reversed: Boolean, var disabled: Boolean) {
    //val crservo = hardwareMap.get(CRServo::class.java, name) as PhotonCRServo
    private val crservo: CRServo? = if (disabled) null else hardwareMap.crservo.get(name)
    init{
        if(reversed)
            crservo?.direction = DcMotorSimple.Direction.REVERSE
    }

    var power: Double = 0.0
        set(v) {
            if (v != field) {
                if(!disabled) {
                    crservo?.power = v
                }
                field = v
            }
        }
}
