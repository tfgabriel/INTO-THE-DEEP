package org.firstinspires.ftc.teamcode.TELEOP

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.pos_diff
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.READY_FOR_TRANSFER
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.WITH_PID
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.manual_tresh
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setFourbar
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setWrist
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_commands
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_specimen
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_specimen
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_up
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.complex_commands
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.WRAPPERS.CAMERA.Camera
import kotlin.math.abs
import com.qualcomm.robotcore.eventloop.opmode.TeleOp as TeleOp

var PS1: Boolean = false
var targetheading: Double = 0.0
var ep: ElapsedTime = ElapsedTime()
var test_place = false
var test_pickup = false
var test_close = false
var test_open = false
var test_positioner = false
var at_exam = false
var current_command: Command? = null
var transfer = false
var transfer_extendo = false
var outtaking = false
var lift_testy0 = false
var lift_testy3 = false
var lift_testy6 = false
var slay = false
var slay2 = false
var slay3 = false
var intake_specimen = false

var isSpecimen = false
var isLiftDown = false
var isIntakeDown = false
var isExtendoAtExam = false
var isIntaking = false
var intake_sample = false
var snatch_sample = false
var curu = false
var curu2 = false
var curu3 = false
var curu4 = false
var curu5 = false
var curu6 = false
var k = false

@Photon
@TeleOp
class opTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        chassis = Chassis()
        lift = Lift()
        extendo = Extendo()
        outtake = Outtake()
        intake = Intake()
        //setExtendoTarget(1)
        k = true
        intake.wrist.position = wrist_neutral
        outtake.positioner.position = positioner_neutral
       // intake.chub_arm.position = intake_vars.chub_arm_intake
       // intake.ehub_arm.position = intake_vars.ehub_arm_intake

        outtake.chub_arm.position = outtake_vars.chub_arm_pickup
        outtake.ehub_arm.position = outtake_vars.ehub_arm_pickup

        outtake.ehub_claw.position = outtake_vars.ehub_claw_close
        outtake.chub_claw.position = outtake_vars.chub_claw_close

        waitForStart()
        while(!isStopRequested){

            ///imu reset
            if(gamepad1.ps && !PS1){
                imew.reset()
                targetheading = imew.yaw
            }
            PS1 = gamepad1.ps

            ///set target heading
            if(abs(gamepad1.right_stick_x) > 0.005){
                targetheading = imew.yaw
                ep.reset()
            } else {
                while(ep.milliseconds() <= chassis_vars.heading_timeout){
                    targetheading = imew.yaw
                }
            }
            send_toall("TARGETHEADING", targetheading)

            ///chassis
            chassis.fc_drive(gamepad1.left_stick_y.toDouble(),  gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x + if(WITH_PID && abs(ang_diff(targetheading, imew.yaw)) >= chassis_vars.angular_tolerance) chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)) else 0.0, gamepad1.left_trigger.toDouble())
            //chassis.rc_drive(gamepad1.left_stick_y.toDouble(),  gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x + if(WITH_PID && abs(ang_diff(targetheading, imew.yaw)) >= chassis_vars.angular_tolerance) chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)) else 0.0, gamepad1.left_trigger.toDouble())

            send_toall("diff", ang_diff(targetheading, imew.yaw))
            send_toall("pid value", chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)))


            //lift
            if(gamepad1.right_bumper && !lift_testy3){
                isSpecimen = true
                isLiftDown = false

                setLiftTarget(3)
            }
            lift_testy3 = gamepad1.right_bumper

            if(gamepad1.left_bumper && !lift_testy6){
                isSpecimen = false
                isLiftDown = false

                setLiftTarget(6)
            }
            lift_testy6 = gamepad1.right_bumper

            if(gamepad1.square && !lift_testy0){
                setLiftTarget(0)
                current_command = if(isSpecimen)
                    complex_commands.place_specimen()
                else
                    complex_commands.place_sample()

                isLiftDown = true
            }
            lift_testy0 = gamepad1.square


            //outtake, sets the outtake in placing position for the lift
            if(gamepad1.circle && !outtaking){

                current_command = if(isSpecimen)
                    complex_commands.prepare_specimen()
                else
                    complex_commands.prepare_sample()
            }
            outtaking =gamepad1.circle

            if(gamepad1.triangle && !curu5){
                current_command = complex_commands.reset_outtake()
            }
            curu5 = gamepad1.triangle



            //transfer from intake to outtake
            if(gamepad2.triangle && !transfer){
                current_command = setClawState(0)
            }
            transfer = gamepad2.triangle

            //intake but for specimens
            if(gamepad2.cross && !slay2){
                isIntaking = false
                if(extendo.chub_rails.currentpos > 250)
                    current_command = intake_commands.reset_arm_and_intake()
                else
                    current_command = intake_commands.specimen_intake()

            }
            slay2 = gamepad2.cross

            //set intake & extendo to intake
            if(gamepad2.square && !transfer_extendo){
                isIntaking = true
            }
            transfer_extendo = gamepad2.square

            //extendo to transfer
            if(gamepad2.dpad_up && !slay){
                current_command = intake_commands.transfer()
                //setExtendoTarget(2)
            }
            slay = gamepad2.dpad_up

            //intake
            if(gamepad2.dpad_left && !intake_sample ){
                current_command = intake_commands.intake()
            }
            intake_sample = gamepad2.dpad_left

            if(gamepad2.right_bumper && !intake_specimen){
                isIntaking = false
                setExtendoTarget(0)
                current_command = intake_commands.sample_spit()
            }
            intake_specimen = gamepad2.right_bumper

            //stop intake
            if(gamepad2.dpad_down && !at_exam){
               // current_command = setIntakePower(-1)
            }
            at_exam = gamepad2.dpad_down

            if(gamepad2.left_trigger > 0.002 && !curu3){
                if(!pos_diff(intake.wrist.position, 0.0))
                    intake.wrist.position -= 0.1
                else
                    intake.wrist.position = wrist_neutral

            }
            curu3 = gamepad2.left_trigger > 0.002

            if(gamepad2.right_trigger > 0.002 && !curu4){
                if(!pos_diff(intake.wrist.position, 1.0))
                    intake.wrist.position += 0.1
                else
                    intake.wrist.position = wrist_neutral

            }
            curu4 = gamepad2.right_trigger > 0.002

            ///update lift
            setLift()
            ///update extendo with either pid or manual input
            setExtendo(gamepad2.left_stick_y.toDouble())
            ///set wrist according to the camera
            //setWrist()

            ///commandbase
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }

            robot.update()
        }
    }
}

var a = false
var b = false
@Disabled
@TeleOp
class mapispeel: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        val servo2 = hardwareMap.crservo.get("EHUB_INTAKER")
        val servo = hardwareMap.servo.get("fourbar")
        waitForStart()
        while (!isStopRequested){
            if(gamepad2.y && !a){
                servo.position = 0.93
            }
            a = gamepad2.y

            if(gamepad2.x && !b){
                servo2.power = 1.0
            }
            b = gamepad2.x
            robot.update()
        }
    }

}


///modify transfer pos for outtake + transfer pos for intake
//transfer sequence intake spit + grab outtake + lift up
