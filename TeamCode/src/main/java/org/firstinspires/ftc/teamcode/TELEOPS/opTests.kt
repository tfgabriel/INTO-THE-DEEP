package org.firstinspires.ftc.teamcode.TELEOPS

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry.Line
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
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
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand

import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.chassis_vars.chassis_f
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinHomeTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_target
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setArmStateIntake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setClawIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setWrist
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.claws_closed
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftPowers
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars
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
var curu9999 = false
var curu2003 = false
var okey = false
var curu1999 = false

var curu666666 = false

var curu77777 = false

var curu90000099 = false

var DISABLE_CAM = true
var curu60 = false
var position = 6
var position2 = 5
var TRENUL_DE_BUZAU = false
var current_command3 : Command? = null
@Disabled
@TeleOp
class ffwheeltest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)
        while (!isStopRequested) {
            chassis.leftfront.power = chassis_f[0]
            chassis.leftback.power = chassis_f[1]
            chassis.rightfront.power = chassis_f[2]
            chassis.rightback.power = chassis_f[3]
        }
    }
}
@TeleOp(name = "我討厭修訂")
class opTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(false)
        robot.start(this)
        DISABLE_CAM = true
        //lift.chub_slides.motor.setCurrentAlert(3.0, CurrentUnit.AMPS)
        k = true
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
            chassis.fc_drive(-gamepad1.left_stick_y.toDouble(),  gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x + if(WITH_PID && abs(ang_diff(
                    targetheading, imew.yaw)) >= chassis_vars.angular_tolerance) chassis_vars.h_PDF.update(ang_diff(
                targetheading, imew.yaw)) else 0.0, gamepad1.left_trigger.toDouble() * 0.65 )

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

            if(gamepad1.dpad_up && !curu77777){
                setLiftTarget(-1)
                setLiftPowers(0.7)
            }
            curu77777 = gamepad1.dpad_up


            if(gamepad1.triangle && !curu5){
                send_toall("isopen trigger", "true")
                current_command  = SequentialCommand(
                    setClawState(1),
                    InstantCommand { send_toall("is actually opened", "yes") }
                )
            }
            curu5 = gamepad1.triangle

            if(gamepad1.cross && !curu60){
                send_toall("isclosed trigger", "true")
                current_command  = SequentialCommand(
                    setClawState(0),
                    InstantCommand { send_toall("is actually closed", "yes") }
                )
            }
            curu60 = gamepad1.cross




            if (gamepad2.circle && !curu9999) {
                isTakingSpecimen = true
                okey = true
                current_command = SequentialCommand(
                    setArmStateIntake(1)
                )
            }
            curu9999 = gamepad2.circle


            if(gamepad2.square && !transfer_extendo){
                isTransferring = true
                setExtendoTarget(0)
                current_command = SequentialCommand(
                    setIntakeState(0),
                    WaitUntilCommand { !isExtendoinHomeTolerance() },
                    SleepCommand(0.55),
                    setClawState(0),
                    SleepCommand(0.2),
                    setClawIntakeState(0),
                )

                intake.wrist.position = wrist_neutral
            }
            transfer_extendo = gamepad2.square


            if(gamepad2.cross && !curu2) {
                current_command =
                    if (pos_diff(intake.claws.position, claws_closed)){
                        if (pos_diff(intake.chub_arm.position, intake_vars.hover))
                            SequentialCommand(
                                setIntakeState(2),
                                SleepCommand(0.2),
                                setClawIntakeState(0)
                            )
                        else
                            SequentialCommand(
                                setClawIntakeState(0)
                            )
                    }
                    else if (pos_diff(intake.chub_arm.position, intake_vars.hover))
                        SequentialCommand(
                            setIntakeState(2),
                            SleepCommand(0.2),
                            setClawIntakeState(1)
                        )
                    else
                        SequentialCommand(
                            setClawIntakeState(1)
                        )

            }
            curu2 = gamepad2.cross

            if(gamepad2.triangle && !curu2001){
                current_command = SequentialCommand(
                    setIntakeState(0)
                )
                intake.wrist.position = wrist_neutral
            }
            curu2001 = gamepad2.triangle

            if(gamepad2.dpad_left && !intake_sample ){
                setExtendoTarget(0)
                current_command = ParallelCommand(
                    setIntakeState(0),
                    setWrist()
                )
            }
            intake_sample = gamepad2.dpad_left

            if(gamepad1.dpad_down && !curu666666){
                lift.chub_slides.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.ehub_slides.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                setLiftTarget(-1)
                setLiftPowers(-1.0)
            }

            if (gamepad1.dpad_left && !curu90000099) {
                setLiftTarget(-1)
                setLiftPowers(0.0)
                lift.chub_slides.motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
                lift.chub_slides.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                lift.ehub_slides.motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
                lift.ehub_slides.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            }
            curu90000099 = gamepad1.dpad_left

            if(gamepad2.right_bumper && !intake_specimen){
                isToExam = true
                setExtendoTarget(2)
                current_command = SequentialCommand(
                    setIntakeState(0),
                    WaitUntilCommand { isExtendoinTolerance() },
                    setIntakeState(1),
                    SleepCommand(0.15),
                    setClawIntakeState(0),
                )
            }
            intake_specimen = gamepad2.right_bumper

            if(gamepad2.left_bumper && !curu2000){
                isToIntake = true
                setExtendoTarget(1)
                current_command = SequentialCommand(
                    setIntakeState(1),
                    WaitUntilCommand { isExtendoinTolerance() },
                )
            }
            curu2000 = gamepad2.left_bumper

            if(gamepad2.left_trigger > 0.5 && !curu3){
                intake.wrist.position = when (position) {
                    9 -> 0.93
                    8 -> 0.82
                    7 -> 0.71
                    6 -> 0.6
                    5 -> 0.49
                    4 -> 0.38
                    3 -> 0.27
                    else -> 0.6
                }
                --position
                if (position == 3) {
                    position = 6
                }
            }
            curu3 = gamepad2.left_trigger > 0.5

            if(gamepad2.right_trigger > 0.5 && !curu4){
                intake.wrist.position = when (position) {
                    9 -> 0.93
                    8 -> 0.82
                    7 -> 0.71
                    6 -> 0.6
                    5 -> 0.49
                    4 -> 0.38
                    3 -> 0.27
                    else -> 0.6
                }
                ++position
                if (position == 9) {
                    position = 6
                }
            }

            curu4 = gamepad2.right_trigger > 0.5

            if(abs(gamepad2.right_stick_y) > 0.001){
                extendo_target = extendo.chub_rails.currentpos
            }



            if(gamepad1.circle && !outtaking){

                if(isSpecimen) {
                    //outtake.chub_arm.position = outtake_vars.chub_arm_place
                    outtake.ehub_arm.position = outtake_vars.ehub_arm_place
                }
                else{
                    //outtake.chub_arm.position = outtake_vars.chub_arm_basket
                    outtake.ehub_arm.position = outtake_vars.ehub_arm_basket
                }

            }
            outtaking =gamepad1.circle

            send_toall("pos", extendo.chub_rails.currentpos)
           // send_toall("KAAAAAAAAAAAAAAAAAAA", lift.chub_slides.motor.isOverCurrent)


            setExtendo(gamepad2.right_stick_y.toDouble())

            setLift()

            ///commandbase
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }


            send_toall("lift pos ehub", lift.ehub_slides.currentpos)
            send_toall("lift pos chub", lift.chub_slides.currentpos)
            send_toall("target", lift_target)

            robot.update()
        }
    }
}

var a = false
var b = false
var c = false
@TeleOp
class mapispeel: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        robot.start(this)
        outtake = Outtake()

        val servo = hardwareMap.servo.get("WRIST_INTAKE")

        servo.position = wrist_neutral
        waitForStart()
        while (!isStopRequested){
            if(gamepad2.y && !a){
                servo.position = 0.5
            }
            a = gamepad2.y

            if(gamepad2.x && !b){
                servo.position = 0.3
            }
            b = gamepad2.x

            if(gamepad2.a && !c){
                servo.position = 0.8
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
