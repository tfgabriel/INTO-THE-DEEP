package org.firstinspires.ftc.teamcode.TELEOP

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.pos_diff
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.WITH_PID
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.WaitUntilCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.Extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_target
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setArmStateIntake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setClawIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setFourbar
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setWrist
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.claws_closed
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.complex_commands
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_claw_open
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
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
var current_command_secondary: Command? = null
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

var isSpecimen = true
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
var curu2000 = false
var curu2001 = false
var sleepy_time: Double = 0.0

var TRENUL_DE_BUZAU = false
var current_command3 : Command? = null
@TeleOp(name = "我討厭修訂")
class opTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)
        //setExtendoTarget(1)
        k = true
        intake.wrist.position = wrist_neutral
        //outtake.positioner.position = positioner_neutral
       // intake.chub_arm.position = intake_vars.chub_arm_intake
       // intake.ehub_arm.position = intake_vars.ehub_arm_intake

        outtake.chub_arm.position = outtake_vars.chub_arm_pickup
        outtake.ehub_arm.position = outtake_vars.ehub_arm_pickup

        outtake.ehub_claw.position = outtake_vars.ehub_claw_open
        outtake.chub_claw.position = outtake_vars.chub_claw_open

        intake.chub_arm.position = intake_vars.chub_arm_transfer
        intake.ehub_arm.position = intake_vars.ehub_arm_transfer

        intake.claws.position = intake_vars.claws_open

        intake.fourbar.position = intake_vars.fourbar_transfer
        setExtendoTarget(1)
        waitForStart()
        while(!isStopRequested){
            sleepy_time = 0.005 * abs(extendo.chub_rails.currentpos - extendo_target)

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

            ///chassis
            chassis.fc_drive(-gamepad1.left_stick_y.toDouble(),  gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x + if(WITH_PID && abs(ang_diff(targetheading, imew.yaw)) >= chassis_vars.angular_tolerance) chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)) else 0.0, gamepad1.left_trigger.toDouble())

            send_toall("imew yaw", imew.yaw)
            //chassis.rc_drive(gamepad1.left_stick_y.toDouble(),  gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x + if(WITH_PID && abs(ang_diff(targetheading, imew.yaw)) >= chassis_vars.angular_tolerance) chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)) else 0.0, gamepad1.left_trigger.toDouble())

            //lift
            if(gamepad1.right_bumper && !lift_testy3){
                isSpecimen = true
                setLiftTarget(3)
            }
            lift_testy3 = gamepad1.right_bumper

            if(gamepad1.left_bumper && !lift_testy6){
                isSpecimen = false
                setLiftTarget(6)
            }
            lift_testy6 = gamepad1.right_bumper


            if(gamepad1.square && !lift_testy0){
                current_command = if(isSpecimen)
                    SequentialCommand(
                        setArmState(0),
                        SleepCommand(0.2),
                        setClawState(2),
                        SleepCommand(0.2),
                        setClawState(1)
                )
                else
                    setArmState(0)
                setLiftTarget(0)
            }
            lift_testy0 = gamepad1.square



            if(gamepad1.triangle && !curu5){
                current_command  = if(pos_diff(outtake.chub_claw.position, chub_claw_open))
                    setClawState(0)
                else
                    setClawState(1)
            }
            curu5 = gamepad1.triangle



            if (gamepad2.circle && !curu3) {
                isTakingSpecimen = true

                current_command = SequentialCommand(

                    setArmStateIntake(4),
                    setFourbar(4),
                )
            }
            curu3 = gamepad2.circle


            if(gamepad2.square && !transfer_extendo){
                isTransferring = true
                setExtendoTarget(2)
                current_command2 = SequentialCommand(
                    setFourbar(4),
                    setArmStateIntake(3),
                    WaitUntilCommand { isExtendoinTolerance() },
                    SleepCommand(0.2),
                    setClawState(0),
                    SleepCommand(0.2),
                    setClawIntakeState(0),
                )

                intake.wrist.position = wrist_neutral
            }
            transfer_extendo = gamepad2.square


            if(gamepad2.cross && !curu2){
                current_command  = if(pos_diff(intake.claws.position, claws_closed))
                    setClawIntakeState(0)
                else
                    setClawIntakeState(1)
            }
            curu2 = gamepad2.cross

            if(gamepad2.triangle && !curu2001){
                current_command = SequentialCommand(
                    setArmStateIntake(4),
                    setFourbar(4),
                    setArmStateIntake(3),
                )
                intake.wrist.position = wrist_neutral
            }
            curu2001 = gamepad2.triangle

            /*




            if(gamepad2.dpad_up && !slay){
                current_command = SequentialCommand(
                    setArmStateIntake(4),
                    setFourbar(0)
                )
            }
            slay = gamepad2.dpad_up

            if(gamepad2.dpad_down && !intake_sample ){
                current_command = SequentialCommand(
                    setArmStateIntake(1),
                    setFourbar(0)
                )
            }
            intake_sample = gamepad2.dpad_down

             */

            if(gamepad2.right_bumper && !intake_specimen){
                isToExam = true
                setExtendoTarget(0)
                current_command = SequentialCommand(
                    setFourbar(4),
                    setArmStateIntake(3),
                    WaitUntilCommand { isExtendoinTolerance() },
                    setArmStateIntake(0),
                    setFourbar(1),
                    SleepCommand(0.15),
                    setClawIntakeState(0),
                )
            }
            intake_specimen = gamepad2.right_bumper

            if(gamepad2.left_bumper && !curu2000){
                isToIntake = true
                setExtendoTarget(3)
                current_command = SequentialCommand(
                    setArmStateIntake(3),
                    setFourbar(4),
                    WaitUntilCommand { isExtendoinTolerance() },
                )
            }
            curu2000 = gamepad2.left_bumper

            if(gamepad2.left_trigger > 0.002 && !curu3){
                if(!pos_diff(intake.wrist.position, 0.01))
                    intake.wrist.position -= 0.09
                else
                    intake.wrist.position = wrist_neutral

            }
            curu3 = gamepad2.left_trigger > 0.002

            if(gamepad2.right_trigger > 0.002 && !curu4){
                if(!pos_diff(intake.wrist.position, 0.89))
                    intake.wrist.position += 0.09
                else
                    intake.wrist.position = wrist_neutral

            }
            curu4 = gamepad2.right_trigger > 0.002

            if(abs(gamepad2.right_stick_y) > 0.001){
                extendo_target = extendo.chub_rails.currentpos
            }



            if(gamepad1.circle && !outtaking){

                if(isSpecimen) {
                    outtake.chub_arm.position = outtake_vars.chub_arm_place
                    outtake.ehub_arm.position = outtake_vars.ehub_arm_place
                }
                else{
                    outtake.chub_arm.position = outtake_vars.chub_arm_basket
                    outtake.ehub_arm.position = outtake_vars.ehub_arm_basket
                }

            }
            outtaking =gamepad1.circle

            send_toall("pos", extendo.chub_rails.currentpos)


            setExtendo(gamepad2.right_stick_y.toDouble())

            setLift()

            ///commandbase
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }

            if(current_command2 != null){
                if(current_command2!!.run(telemetry_packet)){
                    current_command2 = null
                }
            }
            robot.update()
        }
    }
}

var a = false
var b = false
var c = false
@Disabled
@TeleOp
class mapispeel: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        outtake = Outtake()
        waitForStart()
        while (!isStopRequested){
            if(gamepad2.y && !a){
                outtake.chub_arm.position = outtake_vars.chub_arm_place
                outtake.ehub_arm.position = outtake_vars.ehub_arm_place
            }
            a = gamepad2.y

            if(gamepad2.x && !b){
                outtake.chub_arm.position = outtake_vars.chub_arm_pickup
                outtake.ehub_arm.position = outtake_vars.ehub_arm_pickup
            }
            b = gamepad2.x

            if(gamepad2.a && !c){
                outtake.ehub_arm.position = outtake_vars.ehub_arm_basket
                outtake.chub_arm.position = outtake_vars.chub_arm_basket
            }
            c = gamepad2.a
            robot.update()
        }
    }

}

@Disabled
@TeleOp
class sparcfan: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        val localizer = robot_vars.hardwareMap.get(SparkFunOTOS::class.java, "sparkfun")
        localizer.initialize()
        localizer.resetTracking()
        localizer.calibrateImu()
        localizer.angularUnit = AngleUnit.RADIANS
        localizer.linearUnit = DistanceUnit.CM
        localizer.linearScalar = 1.0054
        localizer.angularScalar = 1.0075
        localizer.offset.set(SparkFunOTOS.Pose2D(0.4,0.1,0.0))

        waitForStart()

        while (!isStopRequested){
            send_toall("x", localizer.position.x)
            send_toall("y", localizer.position.y)
            send_toall("heading", localizer.position.h)

            send_toall("spakrgun", localizer.status.get())
            send_toall("sparkfun err lsm", localizer.status.errorLsm)
            send_toall("sparkfun err paa", localizer.status.errorPaa)
            send_toall("sparkfun warning ot", localizer.status.warnOpticalTracking)

            if(gamepad2.circle && curu2001){
                localizer.resetTracking()
            }
            curu2001 = gamepad2.circle
        }
    }

}


///modify transfer pos for outtake + transfer pos for intake
//transfer sequence intake spit + grab outtake + lift up
