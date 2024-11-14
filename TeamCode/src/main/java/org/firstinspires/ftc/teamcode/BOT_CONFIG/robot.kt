package org.firstinspires.ftc.teamcode.BOT_CONFIG

import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.control_hub
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.dashboard
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.expansion_hub
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.linearopmode
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.pose_set
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.voltage_sensor
import org.firstinspires.ftc.teamcode.LOCALIZATION.Sparkfun
import org.firstinspires.ftc.teamcode.P2P.P2P
import org.firstinspires.ftc.teamcode.P2P.blue_vars
import org.firstinspires.ftc.teamcode.P2P.red_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.Systems.ThreadedIMU

class robot(var isAuto: Boolean, var isRed: Boolean) {
    constructor(isAuto: Boolean): this(isAuto, true)

    fun start(lom: LinearOpMode){
        base_init(lom)
        if(isAuto)
            init_auto(isRed)
        init_systems()
    }

    //dash, telemetry, hubs
    fun base_init(lom: LinearOpMode){
        linearopmode = lom
        hardwareMap = linearopmode.hardwareMap

        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        if (lynxModules[0].isParent && LynxConstants.isEmbeddedSerialNumber(lynxModules[0].serialNumber)) {
            control_hub = lynxModules[0]
            expansion_hub = lynxModules[1]
        } else {
            control_hub = lynxModules[1]
            expansion_hub = lynxModules[0]
        }

        imew = ThreadedIMU("IMU")
        imew.init()
        imew.reset()
        dashboard = FtcDashboard.getInstance()
        telemetry = dashboard.telemetry

        //voltage_sensor = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()
    }

    //all systems
    fun init_systems(){
        chassis = Chassis()
        lift = Lift()
        extendo = Extendo()
        intake = Intake()
        outtake = Outtake()
    }

    //camera, localization
    fun init_auto(isRed: Boolean){
        localizer = Sparkfun()
        pose_set = if(isRed)
            red_vars
        else
            blue_vars
        p2p = P2P()
    }

    private val et = ElapsedTime()
    fun update() {
        send_toall("framerate", 1 / et.seconds())
        telemetry.update()
        et.reset()
        control_hub.clearBulkCache()
        expansion_hub.clearBulkCache()
    }
}