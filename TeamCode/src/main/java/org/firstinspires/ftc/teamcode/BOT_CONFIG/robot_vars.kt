package org.firstinspires.ftc.teamcode.BOT_CONFIG

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.LOCALIZATION.Sparkfun
import org.firstinspires.ftc.teamcode.P2P.P2P
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_target
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.Systems.ThreadedIMU
import org.firstinspires.ftc.teamcode.WRAPPERS.CAMERA.Camera
import org.firstinspires.ftc.teamcode.WRAPPERS.Localizer
import kotlin.math.abs

@Config
@SuppressLint("StaticFieldLeak")
object robot_vars {
    lateinit var dashboard: FtcDashboard
    lateinit var telemetry: Telemetry
    lateinit var telemetry_packet: TelemetryPacket
    lateinit var control_hub: LynxModule
    lateinit var expansion_hub: LynxModule

    lateinit var linearopmode: LinearOpMode
    lateinit var hardwareMap: HardwareMap

    lateinit var voltage_sensor: PhotonLynxVoltageSensor

    lateinit var outtake: Outtake
    lateinit var chassis: Chassis
    lateinit var extendo: Extendo
    lateinit var intake: Intake
    lateinit var lift: Lift
    lateinit var imew: ThreadedIMU
    lateinit var localizer: Localizer
    lateinit var camera: Camera
    lateinit var pose_set: Any
    lateinit var p2p: P2P

    @JvmField
    var WITH_PID: Boolean = false
    @JvmField
    var EXTENDO_STATE: Int = 0
    @JvmField
    var LIFT_STATE: Int = 0

    var READY_FOR_TRANSFER: Boolean = false
    var READY_FOR_PLACEMENT: Boolean = false
    var READY_FOR_LEAVING: Boolean = false
    var READY_FOR_INTAKING: Boolean = false
    var READY_FOR_EXAMINING: Boolean = false

    var servo_range = 355.0
    var camera_ang = 0.0
    var camera_distance_from_ground = 0.0
    var cm_to_ticks = 0.0

    @JvmField
    var field_scale = 1 / 2.54
    @JvmField
    var robot_radius = 7.0
    var robot_width = 1

    @JvmField
    var trajectory_color = "#00FF00"

    @JvmField
    var robot_color = "#FF0000"

}