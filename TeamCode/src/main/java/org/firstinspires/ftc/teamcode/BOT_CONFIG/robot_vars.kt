package org.firstinspires.ftc.teamcode.BOT_CONFIG

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.LOCALIZATION.Sparkfun
import org.firstinspires.ftc.teamcode.P2P.P2P
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.WRAPPERS.CAMERA.Camera
import org.firstinspires.ftc.teamcode.WRAPPERS.THREADEDIMU.ThreadedIMU

@SuppressLint("StaticFieldLeak")
object robot_vars {
    lateinit var dashboard: FtcDashboard
    lateinit var telemetry: Telemetry
    lateinit var control_hub: LynxModule
    lateinit var expansion_hub: LynxModule

    lateinit var hardwareMap: HardwareMap

    lateinit var voltage_sensor: PhotonLynxVoltageSensor

    lateinit var outtake: Outtake
    lateinit var chassis: Chassis
    lateinit var extendo: Extendo
    lateinit var intake: Intake
    lateinit var lift: Lift
    lateinit var imew: ThreadedIMU
    lateinit var localizer: Sparkfun
    lateinit var camera: Camera
    lateinit var pose_set: Any
    lateinit var p2p: P2P
}