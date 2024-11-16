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
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoPowers
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
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftPowers
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.WRAPPERS.CR_SERVO
import kotlin.math.abs
import kotlin.math.sign
import com.qualcomm.robotcore.eventloop.opmode.TeleOp as TeleOp

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
        robot.base_init(this)
        extendo = Extendo()

        waitForStart()
        while(!isStopRequested){
            setExtendoTarget(EXTENDO_STATE)
            extendo_pdf = PDF(extendo_vars.proportional, extendo_vars.derivative, extendo_vars.force)

            setExtendo()
            send_toall("ERR", extendo_target- extendo.chub_rails.currentpos)
            send_toall("PID VALUE", extendo_pdf.update((extendo_target- extendo.chub_rails.currentpos).toDouble()))
            robot.update()
        }
    }

}

@Photon
@TeleOp
class lift_test: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        lift = Lift()
        waitForStart()

        while(!isStopRequested){
            setLiftTarget(LIFT_STATE)
            lift_pdf = PDF(lift_vars.proportional, lift_vars.derivative, lift_vars.force)
            setLift()
            send_toall("ERR", lift_target-lift.chub_slides.currentpos)
            send_toall("PID VALUE", lift_pdf.update((lift_target-lift.chub_slides.currentpos).toDouble()))
            robot.update()
        }
    }

}

@Photon
@TeleOp
class set_lift_f: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        lift = Lift()

        waitForStart()
        while(!isStopRequested){
            setLiftPowers(lift_vars.force)
            robot.update()
        }
    }
}

@Photon
@TeleOp
class set_extendo_f: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        extendo = Extendo()

        waitForStart()
        while(!isStopRequested){
            setExtendoPowers(extendo_vars.force)
            robot.update()
        }
    }
}

@Photon
@TeleOp
class find_lift_positions: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        lift = Lift()

        waitForStart()
        while(!isStopRequested){
            send_toall("LIFT POSITION", lift.chub_slides.currentpos)
            robot.update()
        }
    }
}

@Photon
@TeleOp
class find_extendo_positions: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        extendo = Extendo()

        waitForStart()
        while(!isStopRequested){
            send_toall("EXTENDO POSITION", extendo.chub_rails.currentpos)
            robot.update()
        }
    }
}