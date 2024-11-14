package org.firstinspires.ftc.teamcode.TELEOP

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.EXTENDO_STATE
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.LIFT_STATE
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.WITH_PID
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.derivative
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_target
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.force
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.proportional
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_testing
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_testing
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.intaker_power
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.intaker_power_testing
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.WRAPPERS.CR_SERVO
import kotlin.math.abs
import kotlin.math.sign
import com.qualcomm.robotcore.eventloop.opmode.TeleOp as TeleOp

var PS1: Boolean = false
var targetheading: Double = 0.0
var ep: ElapsedTime = ElapsedTime()
@Photon
@TeleOp
class opTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)

        waitForStart()
        while(!isStopRequested){
            chassis_vars.h_PDF = PDF(chassis_vars.p, chassis_vars.d, chassis_vars.f)

            send_toall("IMEW", imew.yaw)
            send_toall("FORWARDS POWER", gamepad1.left_stick_y)
            send_toall("STRAFE POWER", gamepad1.left_stick_x)
            send_toall("ROTATION POWER", gamepad1.right_stick_x)
            send_toall("SLOWDOWN", gamepad1.left_trigger)

            if(gamepad1.ps && !PS1){
                imew.reset()
                targetheading = imew.yaw
            }
            PS1 = gamepad1.ps

            if(abs(gamepad1.right_stick_x) > 0.005){
                targetheading = imew.yaw
                ep.reset()
            } else {
                while(ep.milliseconds() <= chassis_vars.heading_timeout){
                    targetheading = imew.yaw
                }
            }

            send_toall("TARGETHEADING", targetheading)

            chassis.fc_drive(gamepad1.left_stick_y.toDouble(),  gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x + if(WITH_PID && abs(ang_diff(targetheading, imew.yaw)) >= chassis_vars.angular_tolerance) chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)) else 0.0, gamepad1.left_trigger.toDouble())
            send_toall("diff", ang_diff(targetheading, imew.yaw))
            send_toall("pid value", chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)))

            robot.update()
        }
    }
}

@TeleOp
class servo_resetter: LinearOpMode(){

    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)
        val chub_outtake_servo = robot_vars.hardwareMap.servo.get("CHUB_ARM")
        val ehub_outtake_servo = robot_vars.hardwareMap.servo.get("EHUB_ARM")

        waitForStart()
        while (!isStopRequested){
            chub_outtake_servo.position = 0.0
            ehub_outtake_servo.position = 1.0

            robot.update()
        }
    }

}

@Photon
@TeleOp
class extendo_test: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)

        waitForStart()
        while(!isStopRequested){
            setExtendoTarget(EXTENDO_STATE)
            extendo_pdf = PDF(extendo_vars.proportional, extendo_vars.derivative, extendo_vars.force)

            extendo.chub_rails.power = if(!isExtendoinTolerance()) extendo_pdf.update((extendo_target - extendo.chub_rails.currentpos).toDouble()) else extendo_vars.force * sign((extendo_target - extendo.chub_rails.currentpos).toDouble())
            extendo.ehub_rails.power = if(!isExtendoinTolerance()) extendo_pdf.update((extendo_target - extendo.chub_rails.currentpos).toDouble()) else extendo_vars.force * sign((extendo_target - extendo.chub_rails.currentpos).toDouble())
            robot.update()
        }
    }

}

@Photon
@TeleOp
class lift_test: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)

        waitForStart()
        while(!isStopRequested){
            setLiftTarget(LIFT_STATE)
            lift_pdf = PDF(lift_vars.proportional, lift_vars.derivative, lift_vars.force)

            lift.chub_slides.power = if(!isLiftinTolerance()) lift_pdf.update((lift_target - lift.chub_slides.currentpos).toDouble()) else lift_vars.force * sign((lift_target - lift.chub_slides.currentpos).toDouble())
            lift.ehub_slides.power = if(!isLiftinTolerance()) lift_pdf.update((lift_target - lift.chub_slides.currentpos).toDouble()) else lift_vars.force * sign((lift_target - lift.chub_slides.currentpos).toDouble())
            robot.update()
        }
    }

}