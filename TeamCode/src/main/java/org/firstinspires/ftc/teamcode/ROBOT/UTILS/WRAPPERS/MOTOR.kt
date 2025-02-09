package org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.pos_diff
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.isAuto

class FIRSTOPEN(@JvmField var opened: Boolean = false)

class MOTOR(name: String, encoder: Boolean, reversed: Boolean, opened: FIRSTOPEN? = null) {
    //val motor: PhotonDcMotor = hardwareMap.get(DcMotorEx::class.java, name) as PhotonDcMotor
    val motor = hardwareMap.dcMotor.get(name) as DcMotorEx

    init {
        if ((opened == null || !opened.opened) && encoder) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            if (opened != null) { opened.opened = true }
        } else if (encoder) {
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        } else
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.setCurrentAlert(3.0, CurrentUnit.AMPS)

        if (reversed)
            motor.direction = DcMotorSimple.Direction.REVERSE
        motor.power = 0.0
    }

    val currentpos: Int
        get() {
            return motor.currentPosition
        }

    val amps: Double
        get() {
            return motor.getCurrent(CurrentUnit.MILLIAMPS)
        }


    var power: Double = 0.0
        set(p) {
            if (!pos_diff(p, field)) {
                field = p
                motor.power = p
            }
        }
}