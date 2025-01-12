package org.firstinspires.ftc.teamcode.BOT_CONFIG

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera
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
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.result
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
import org.firstinspires.ftc.teamcode.P2P.P2P
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_neutral
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.Systems.ThreadedIMU
import org.firstinspires.ftc.teamcode.TELEMETRY.drawings
import org.firstinspires.ftc.teamcode.TELEOPS.DISABLE_CAM
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.Localizer
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinMaxTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinTolerance

class robot(var isAuto: Boolean, var isRed: Boolean, var isSample: Boolean) {
    constructor(isAuto: Boolean): this(isAuto, true, true)

    fun start(lom: LinearOpMode){
        base_init(lom)
        if(isAuto)
            init_auto(isRed, isSample)
        init_systems()
        init_positions(isAuto)
    }

    fun init_systems(){
        val tp = TelemetryPacket()
        chassis = Chassis()
        tp.put("0Chassis", ep.seconds()); dashboard.sendTelemetryPacket(tp)
        lift = Lift()
        tp.put("0Lift", ep.seconds()); dashboard.sendTelemetryPacket(tp)
        extendo = Extendo()
        tp.put("0Extendo", ep.seconds()); dashboard.sendTelemetryPacket(tp)
        intake = Intake()
        tp.put("0Intake", ep.seconds()); dashboard.sendTelemetryPacket(tp)
        outtake = Outtake()
        tp.put("0Outtake", ep.seconds()); dashboard.sendTelemetryPacket(tp)
        localizer = Localizer("sparkfun")
        tp.put("0Localizer", ep.seconds()); dashboard.sendTelemetryPacket(tp)
    }
    //camera, localization
    fun init_auto(isRed: Boolean, isSample: Boolean){
        p2p = P2P()
    }


    //dash, telemetry, hubs
    val ep = ElapsedTime()
    fun base_init(lom: LinearOpMode){
        val tp = TelemetryPacket()
        ep.reset()
        dashboard = FtcDashboard.getInstance()
        tp.put("0Start", ep.seconds()); dashboard.sendTelemetryPacket(tp)
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
        tp.put("0Lynx", ep.seconds()); dashboard.sendTelemetryPacket(tp)

        imew = ThreadedIMU("IMU")
        imew.init()
        imew.initThread()
        tp.put("0Imew", ep.seconds()); dashboard.sendTelemetryPacket(tp)
        telemetry = dashboard.telemetry
        telemetry_packet = TelemetryPacket()

        //voltage_sensor = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()
    }

    //all systems

    fun init_positions(isAuto: Boolean){
        intake.wrist.position = intake_vars.wrist_neutral
        outtake.positioner.position = positioner_neutral
        outtake.chub_arm.position = outtake_vars.chub_arm_pickup
        outtake.ehub_arm.position = outtake_vars.ehub_arm_pickup

        if(isAuto){
            outtake.ehub_claw.position = outtake_vars.ehub_claw_close
            outtake.chub_claw.position = outtake_vars.chub_claw_close
        }
        else {
            outtake.ehub_claw.position = outtake_vars.ehub_claw_open
            outtake.chub_claw.position = outtake_vars.chub_claw_open
        }

        intake.chub_arm.position = transfer
        intake.ehub_arm.position = transfer
        intake.claws.position = intake_vars.claws_open

        intake.fourbar.position = intake_vars.fourbar_transfer
    }

    private val et = ElapsedTime()
    fun update() {
        send_toall("framerate", 1 / et.seconds())
        send_toall("slide chub", lift.chub_slides.motor.isOverCurrent)
        send_toall("slide ehub", lift.ehub_slides.motor.isOverCurrent)
        send_toall("chassis lf", chassis.leftfront.motor.isOverCurrent)
        send_toall("chassis lb", chassis.leftback.motor.isOverCurrent)
        send_toall("chassis rf", chassis.rightfront.motor.isOverCurrent)
        send_toall("chassis rb", chassis.rightback.motor.isOverCurrent)
        send_toall("extendo", extendo.chub_rails.motor.isOverCurrent)
        send_toall("lift in tolerance", isLiftinMaxTolerance())
        send_toall("lift pos", lift.chub_slides.currentpos)
        send_toall("extendo pos", extendo.chub_rails.currentpos)



        val tp = TelemetryPacket()
        val canvas = tp.fieldOverlay()
        drawings.drawRobot(canvas, localizer.pose)
        send_toall("isato", isAuto)
        if (isAuto) { drawings.drawP2P(canvas) }
        send_toall("POse", localizer.pose)



        localizer.update()






        //send_toall("is open", camera.is_open)
        //send_toall("is valid", result.isValid)
        //send_toall("is not empty", !result.colorResults.isEmpty())
        //send_toall("is in size", result.colorResults[0].targetCorners.size == 4)

        if(!DISABLE_CAM) {
            if (camera.is_open)
                if (result.isValid())
                    if (!result.colorResults.isEmpty())
                        if (result.colorResults[0].targetCorners.size == 4)
                            drawings.draw_sample(canvas, result)
        }
        dashboard.sendTelemetryPacket(tp)
        telemetry.update()
        et.reset()
        control_hub.clearBulkCache()
        expansion_hub.clearBulkCache()
    }
}