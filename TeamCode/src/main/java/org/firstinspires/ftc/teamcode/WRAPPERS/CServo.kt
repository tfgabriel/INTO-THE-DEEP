package org.firstinspires.ftc.teamcode.WRAPPERS

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap

class CServo(name: String, var reverse: Boolean, var disable: Boolean) {

    constructor(name: String): this(name, false, false)
    constructor(name: String, reverse: Boolean): this(name, reverse, false)

    private val servo: Servo? = if (disable) null else hardwareMap.get(Servo::class.java, name)
    var position: Double = 0.0
        set(value) {
            if (!disable) { servo?.position = if (reverse) 1 - value else value }
            field = value
        }
}