package org.firstinspires.ftc.teamcode.BOT_CONFIG

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift

object robot_vars {
    lateinit var hardwareMap: HardwareMap

    lateinit var outtake: Outtake
    lateinit var chassis: Chassis
    lateinit var extendo: Extendo
    lateinit var intake: Intake
    lateinit var lift: Lift
}