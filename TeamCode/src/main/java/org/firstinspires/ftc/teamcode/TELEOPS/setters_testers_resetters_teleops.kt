package org.firstinspires.ftc.teamcode.TELEOPS

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_to_pos
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.x_distance
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.y_distance
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.EXTENDO_STATE
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.LIFT_STATE
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.control_hub
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.dashboard
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.expansion_hub
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.linearopmode
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.result
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
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoPowers
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_target
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.Intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setArmStateIntake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setClawIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setFourbar
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setWrist
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_testing
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.Lift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftPowers
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.tolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.Outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.complex_commands
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState
import org.firstinspires.ftc.teamcode.Systems.ThreadedIMU
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.CAMERA.Camera
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.Localizer
import org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.MOTOR
import kotlin.math.abs
import com.qualcomm.robotcore.eventloop.opmode.TeleOp as TeleOp

@Disabled
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

@Disabled
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
            send_toall("POWER", extendo.chub_rails.power)
            send_toall("POS", extendo.chub_rails.currentpos)
            send_toall("ERR", extendo_target- extendo.chub_rails.currentpos)
            send_toall("PID VALUE", extendo_pdf.update((extendo_target- extendo.chub_rails.currentpos).toDouble()))
            setExtendo(0.0)
            robot.update()
        }
    }

}

var test_pid = false
@Disabled
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
            send_toall("IS IN TOLERANCE", lift_target-lift.ehub_slides.currentpos < tolerance)
            send_toall("VALUE",  lift_target-lift.ehub_slides.currentpos * lift_vars.proportional)
            send_toall("POS", lift.ehub_slides.currentpos)
            send_toall("ERR", lift_target-lift.ehub_slides.currentpos)
            send_toall("PID VALUE", lift_pdf.update((lift_target-lift.ehub_slides.currentpos).toDouble()))
            robot.update()
        }
    }

}

@Disabled
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

@Disabled
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

@Disabled
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

@Disabled
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

@Disabled
@TeleOp
class are_servos_alive: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)

        val servo1 = hardwareMap.servo.get("POSITIONER")
        val servo2 = hardwareMap.servo.get("CHUB_CLAW")
        val servo3 = hardwareMap.servo.get("EHUB_CLAW")
        val servo4 = hardwareMap.servo.get("CHUB_ARM")
        val servo5 = hardwareMap.servo.get("EHUB_ARM")

        var test1: Boolean = false
        var test2: Boolean = false
        var test3: Boolean = false
        var test4: Boolean = false
        var test5: Boolean = false

        waitForStart()
        while(!isStopRequested){
            if(gamepad1.x && !test1){
                servo1.position = 0.5
            }
            test1 = gamepad1.x

            if(gamepad1.b && !test2) {
                servo2.position = 0.5
            }
            test2 = gamepad1.b
            if(gamepad1.y && !test3) {
                servo3.position = 0.5
            }
            test3 = gamepad1.y

            if(gamepad1.a && !test4) {
                servo4.position = 0.5
            }
            test4 = gamepad1.a

            if(gamepad1.dpad_up && !test5) {
                servo5.position = 0.5
            }
            test5 = gamepad1.dpad_up

            robot.update()
        }
    }

}

@Disabled
@TeleOp
class are_intake_servos_alive: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)

        val servo1 = hardwareMap.servo.get("EHUB_ARM")
        val servo2 = hardwareMap.servo.get("CHUB_ARM")
        val servo3 = hardwareMap.servo.get("WRIST")
        val servo4 = hardwareMap.crservo.get("CHUB_INTAKER")
        val servo5 = hardwareMap.crservo.get("EHUB_INTAKER")

        var test1: Boolean = false
        var test2: Boolean = false
        var test3: Boolean = false
        var test4: Boolean = false
        var test5: Boolean = false
        while (!isStopRequested){
            if(gamepad1.x && !test1){
                servo1.position = 0.5
            }
            test1 = gamepad1.x

            if(gamepad1.b && !test2) {
                servo2.position = 0.5
            }
            test2 = gamepad1.b
            if(gamepad1.y && !test3) {
                servo3.position = 0.5
            }
            test3 = gamepad1.y

            if(gamepad1.a && !test4) {
                servo4.power = 0.5
            }
            test4 = gamepad1.a

            if(gamepad1.dpad_up && !test5) {
                servo5.power = 0.5
            }
            test5 = gamepad1.dpad_up
            robot.update()
        }
    }
}

@Disabled
@Photon
@TeleOp
class slides_testyyyy: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        val chub_slide = MOTOR("CHUB_SLIDE", false, false)
        val ehub_slide = MOTOR("EHUB_SLIDE", true, true)
        var slide_power: Double
        waitForStart()
        while(!isStopRequested){
            send_toall("LEFT STICK Y", gamepad2.left_stick_y.toDouble())
            slide_power = -gamepad2.left_stick_y.toDouble()
            chub_slide.power = slide_power
            ehub_slide.power = slide_power

            robot.update()
        }
    }

}

@Disabled
@TeleOp
class extendo_testyyyy: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        var extendo_motor = hardwareMap.dcMotor.get("EXTENDO_MOTOR")
        var extendo_power: Double
        waitForStart()
        while(!isStopRequested){
            extendo_power = gamepad2.left_stick_y.toDouble()
            extendo_motor.power = extendo_power

            robot.update()
        }
    }

}

@Disabled
@TeleOp
class intake_testyyyy: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        val servo1 = hardwareMap.servo.get("EHUB_ARM")
        val servo2 = hardwareMap.servo.get("CHUB_ARM")
        val servo3 = hardwareMap.servo.get("WRIST")
        val servo4 = hardwareMap.crservo.get("CHUB_INTAKER")
        val servo5 = hardwareMap.crservo.get("EHUB_INTAKER")

        var test1: Boolean = false
        var test2: Boolean = false
        var test3: Boolean = false
        var test4: Boolean = false
        var test5: Boolean = false
        var a: Boolean = false
        waitForStart()
        while(!isStopRequested){
            if(gamepad1.y && !test1){
            }
            test1 = gamepad1.y

            if(gamepad1.a && !test2){
            }
            test2 = gamepad1.a

            if(gamepad1.x && !test4){
                servo4.power = 1.0
                servo5.power = 1.0
                a = true
            }
            test4 = gamepad1.x

            if(gamepad1.b && !test3){
                servo4.power = -1.0
                servo5.power = -1.0
                a = false
            }
            test3 = gamepad1.b

            send_toall("chub_arm position", servo2.position)
            robot.update()
        }
    }

}

@Disabled
@TeleOp
class camera_testyyyy: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        val limelight = hardwareMap.get(Limelight3A::class.java,"limelight")
        limelight.start()
        limelight.pipelineSwitch(0)
        waitForStart()
        while(!isStopRequested){
            send_toall("X_ANGLE_OFFSET", limelight.latestResult.tx)
            send_toall("Y_ANGLE_OFFSET", limelight.latestResult.ty)
            send_toall("COLOR", limelight.latestResult.colorResults)

            robot.update()
        }
    }

}

@Disabled
@TeleOp
class outtake_testyyyy: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        val servo1 = hardwareMap.servo.get("EHUB_ARM")
        val servo2 = hardwareMap.servo.get("CHUB_ARM")
        val servo3 = hardwareMap.servo.get("POSITIONER")
        val servo4 = hardwareMap.servo.get("CHUB_CLAW")
        val servo5 = hardwareMap.servo.get("EHUB_CLAW")

        var test1: Boolean = false
        var test2: Boolean = false
        var test3: Boolean = false
        var test4: Boolean = false
        var test5: Boolean = false
        while(!isStopRequested){
            if(gamepad1.y && !test1) {
                servo2.position = outtake_vars.chub_arm_testing
            }
            test1 = gamepad1.y
            //servo3.position = outtake_vars.chub_arm_testing

            if(gamepad1.b && !test2){
                servo3.position = outtake_vars.positioner_testing
            }
            test2 = gamepad1.b

            if(gamepad1.x && !test3){
                servo4.position = outtake_vars.chub_claw_testing
                servo5.position = outtake_vars.ehub_claw_testing
            }
            test3 = gamepad1.x

            send_toall("chub_arm position", servo2.position)
            robot.update()
        }
    }

}

@Disabled
@TeleOp
class imew_checker: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        chassis = Chassis()
        lift = Lift()

        waitForStart()
        while(!isStopRequested){
            chassis_vars.h_PDF = PDF(chassis_vars.p, chassis_vars.d, chassis_vars.f)

            send_toall("IMEW", imew.yaw)
            send_toall("FORWARDS POWER", gamepad1.left_stick_y)
            send_toall("STRAFE POWER", gamepad1.left_stick_x)
            send_toall("ROTATION POWER", gamepad1.right_stick_x)
            send_toall("SLOWDOWN", gamepad1.left_trigger)

            if(gamepad1.ps && !PS1){
                targetheading = 0.0
                imew.reset()
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

            //chassis.fc_drive(gamepad1.left_stick_y.toDouble(),  gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x + if(WITH_PID && abs(ang_diff(targetheading, imew.yaw)) >= chassis_vars.angular_tolerance) chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)) else 0.0, gamepad1.left_trigger.toDouble())
            send_toall("diff", ang_diff(targetheading, imew.yaw))
            send_toall("pid value", chassis_vars.h_PDF.update(ang_diff(targetheading, imew.yaw)))

            robot.update()
        }
    }

}

@Disabled
@TeleOp
class teletest: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        linearopmode = this
        robot_vars.hardwareMap = linearopmode.hardwareMap

        val lynxModules = robot_vars.hardwareMap.getAll(LynxModule::class.java)
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
        robot_vars.telemetry = dashboard.telemetry
        telemetry_packet = TelemetryPacket()

        waitForStart()
        while(!isStopRequested){

        }
    }

}


var aaaaaa = false

@Disabled
@TeleOp
class servoTEST: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        intake = Intake()
        waitForStart()
        while(!isStopRequested){
            if(gamepad2.y && !test_place) {
                intake.chub_arm.position = 0.0
                intake.ehub_arm.position = 1.0
            }
            test_place = gamepad2.y

        }
    }
}


var bbbbb = false
var cccccc = false
var dddd = false
var eee = false
@TeleOp
class servoTTTT: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        intake = Intake()
        outtake = Outtake()
        lift = Lift()
        extendo = Extendo()
        waitForStart()
        while(!isStopRequested){
            if(gamepad2.y && !test_place) {
            }
            test_place = gamepad2.y

            if(gamepad2.b && !test_pickup) {
                intake.fourbar.position = fourbar_testing
            }
            test_pickup = gamepad2.b

            if(gamepad2.x && !test_open) {
            }
            test_open = gamepad2.x

            if(gamepad2.dpad_down && !test_pid) {
               // intake.chub_intaker.power = 1.0
               // intake.ehub_intaker.power = 1.0
            }
            test_pid = gamepad2.dpad_down

            if(gamepad2.dpad_up && !test_positioner) {
                //outtake.chub_arm.position = outtake_vars.chub_arm_testing
                outtake.ehub_arm.position = outtake_vars.ehub_arm_testing
                //outtake.positioner.position = outtake_vars.positioner_neutral
            }
            test_positioner = gamepad2.dpad_up

            if(gamepad2.dpad_left && !aaaaaa) {
               // intake.chub_intaker.power = 0.0
               // intake.ehub_intaker.power = 0.0
            }
            aaaaaa = gamepad2.dpad_left


            if(gamepad2.left_bumper && !dddd) {
                setLiftTarget(3)
            }
            dddd = gamepad2.left_bumper

            if(gamepad2.a && !eee) {
                setLiftTarget(0)
            }
            eee = gamepad2.a

            extendo.chub_rails.power = gamepad2.left_stick_y.toDouble()
            send_toall("pos", extendo.chub_rails.currentpos)
            setLift()
            robot.update()
        }
    }

}

@Disabled
@TeleOp
class ecstendo: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        extendo = Extendo()
        val servo = hardwareMap.servo.get("wrist")
        waitForStart()
        while(!isStopRequested){
            extendo.chub_rails.power = gamepad2.left_stick_y.toDouble()
            send_toall("pos", extendo.chub_rails.currentpos)


            robot.update()
        }
    }
}

@Disabled
@TeleOp
class outtake_reset: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        outtake = Outtake()
        intake = Intake()
        waitForStart()
        while(!isStopRequested){
            if(gamepad2.circle && !test_positioner) {
            }
            test_positioner = gamepad2.circle

            if(gamepad2.square && !test_open) {
            }
            test_open = gamepad2.square

            if(gamepad2.triangle && !test_pid) {
                intake.fourbar.position = fourbar_transfer
                intake.wrist.position = wrist_neutral
            }
            test_pid = gamepad2.triangle
            robot.update()
        }
    }

}
@TeleOp
class cameruta: LinearOpMode(){

    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        camera = Camera()
        camera.limelight.start()
        camera.limelight.pipelineSwitch(0)
        localizer = Localizer("sparkfun")
        localizer.reset()
        var k: Int = 0
        var mid: Vec2D = Vec2D()

        // LLFieldMap pentru Autonom please :()()()()()
        waitForStart()
        while(!isStopRequested) {
            send_toall("alive", camera.limelight.isRunning)
            send_toall("ahem", camera.limelight.status)
            result = camera.limelight.getLatestResult()

            if (result != null) {
                if (result.isValid()) {
                    //telemetry.addData("tx", result.getTx());
                    //telemetry.addData("ty", result.getTy());
                    //send_toall("mamamama", result.colorResults.isEmpty())

                    if (!result.colorResults.isEmpty()) {
                        send_toall("size", result.colorResults[0].targetCorners.size)
                        send_toall("IS IN SIZE", result.colorResults[0].targetCorners.size == 4)
                        if (result.colorResults[0].targetCorners.size == 4) {
                            send_toall(
                                "",
                                "----------------------- CORNERS ------------------------"
                            )
                            mid = Vec2D()
                            for (point in result.colorResults[0].targetCorners) {
                                val corner = String.format("corner %d", k)
                                send_toall(corner, result.colorResults[0].targetCorners[k])
                                mid += Vec2D(result.colorResults[0].targetCorners[k])
                                if (k < 4)
                                    k++
                                else
                                    break
                            }


                            send_toall(
                                "",
                                "----------------------- MIDPOINT ------------------------"
                            )
                            send_toall("mid", mid / 4.0)

                            send_toall(
                                "",
                                "----------------------- DISTANCES ------------------------"
                            )

                            send_toall("Y DIST", y_distance(camera.ang_Y))
                            send_toall("X DIST", x_distance(camera.ang_X, camera.ang_Y))

                            send_toall(
                                "",
                                "----------------------- ANGLES ------------------------"
                            )

                            send_toall("SAMPLE OFF Y", camera.ang_Y)
                            send_toall("SAMPLE OFF X", camera.ang_X)

                            send_toall(
                                "",
                                "----------------------- WRIST POS ------------------------"
                            )

                            send_toall(
                                "servo pos",
                                ang_to_pos(Vec2D(result.colorResults[0].targetCorners[0]), Vec2D(result.colorResults[0].targetCorners[2]))
                            )

                            //drawings.draw_sample(canvas, result)
                        }
                    }
                }
            }



            robot.update()
            k = 0
        }
    }
}

//intake the whole time + retract arm + put wrist in home position
//go from current pos to homed exam zone on extendo -> after in homed go to max + put 4bar in down position + spit
//lift arm up in intermediary position then set it down + intake to pickup specimen
//retract extendo to transfer zone + set 4bar in up transfer position
//get camera data -> nearest sample orientation / distance
//let camera / driver set extendo for next sample ???????????? if i can't automatically set the extendo pos for the next sample using the limelight
//then i don't really know how i'd give manual access to the driver if the extendo is only set to running with pids or how would the driver
//'guesstimate' how far the arm will land unless they somehow have very good depth perception and are well accustomed to the length of the arm
//


//leftt bumper -> lift to high chamber, right bumper -
var curu7 = false
var curu8 = false
var curu9 = false
var curu10= false
var curu1 = false

var isToExam = false
var isTakingSpecimen = false
var isTransferring = false
var isToIntake = false
var current_command2: Command? = null
@TeleOp
class teser: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        chassis = Chassis()
        intake = Intake()
        outtake = Outtake()
        lift = Lift()
        extendo = Extendo()
        camera = Camera()
        current_command = InstantCommand{}
        setExtendoTarget(1)
        waitForStart()

        while(!isStopRequested) {

            if (gamepad2.triangle && !curu) {
                isToExam = true
                setExtendoTarget(0)
                current_command = SequentialCommand(
                    WaitUntilCommand { isExtendoinTolerance() },
                    setArmStateIntake(2),
                    setFourbar(1)
                )
            }
            curu = gamepad2.triangle

            if (gamepad2.circle && !curu2) {
                current_command = setClawIntakeState(0)
            }
            curu2 = gamepad2.circle

            if (gamepad2.cross && !curu1999) {
                isTakingSpecimen = true


                current_command = SequentialCommand(
                    setFourbar(1),
                    setArmStateIntake(0),
                    SleepCommand(0.2),
                    setClawIntakeState(1),
                    SleepCommand(0.3),
                    setArmStateIntake(3),
                    setFourbar(4),
                    setWrist(),
                )
            }
            curu1999 = gamepad2.cross

            if (gamepad2.square && !curu2003) {
                current_command2 = SequentialCommand(
                    setArmState(0),
                    setClawState(1)
                )
            }
            curu2003 = gamepad2.square

            if(gamepad2.dpad_down && !curu6){
                isTransferring = true
                setExtendoTarget(2)
                current_command2 = SequentialCommand(
                    SleepCommand(0.7),
                    setFourbar(6),
                    SleepCommand(0.2),
                    setClawState(0),
                    SleepCommand(0.4),
                    setClawIntakeState(0)
                )
            }
            curu6 = gamepad2.dpad_down

            if (gamepad2.dpad_right && !curu1) {

            }
            curu1 = gamepad2.dpad_right

            if (gamepad1.circle && !curu7) {
            }
            curu7 = gamepad1.circle

            if (gamepad1.cross && !curu8) {
                intake.fourbar.position = intake_vars.fourbar_testing
            }
            curu8 = gamepad1.cross

            if (gamepad1.square && !curu9) {
            }
            curu9 = gamepad1.square

            if(gamepad2.dpad_up && !curu10){
                current_command2 = if(isSpecimen)
                    complex_commands.prepare_specimen()
                else
                    complex_commands.prepare_sample()
            }
            curu10 = gamepad2.dpad_up

            if(abs(gamepad2.right_stick_y) > 0.001){
                extendo_target = extendo.chub_rails.currentpos
            }

           /* if(extendo.chub_rails.currentpos in home_examination - 150 .. home_submersible + 200){
                if(isToExam) {
                    setExtendoTarget(0)
                    current_command = SequentialCommand(
                        setArmStateIntake(3),
                        setFourbar(4),
                        WaitUntilCommand { isExtendoinTolerance() },
                        setFourbar(1),
                        setArmStateIntake(2)
                    )
                    isToExam = false
                } else if(isTransferring){
                    current_command = SequentialCommand(
                        SleepCommand(0.5),
                        setClawState(0),
                        SleepCommand(0.4),
                        setClawIntakeState(0)
                    )
                    setExtendoTarget(2)
                    isTransferring = false
                }
                else
                    current_command = SequentialCommand(
                        setArmStateIntake(3),
                        setFourbar(4),
                        setWrist()
                    )

            }

            */

            if(gamepad2.right_bumper && !lift_testy3){
                isSpecimen = true

                setLiftTarget(3)
            }
            lift_testy3 = gamepad2.right_bumper

            if(gamepad2.left_bumper && !lift_testy6){
                isSpecimen = false

                setLiftTarget(6)
            }
            lift_testy6 = gamepad2.right_bumper

            if(gamepad2.dpad_left && !lift_testy0){
                current_command2 = if(isSpecimen)
                    complex_commands.place_specimen()
                else
                    complex_commands.place_sample()
                setLiftTarget(0)
            }
            lift_testy0 = gamepad2.dpad_left

            setLift()
            setExtendo(gamepad2.right_stick_y.toDouble())

            if (current_command != null) {
                if (current_command!!.run(telemetry_packet)) {
                    current_command = null
                }
            }

            if (current_command2 != null) {
                if (current_command2!!.run(telemetry_packet)) {
                    current_command2 = null
                }
            }

            send_toall("lkjhgfds", extendo.chub_rails.currentpos)

            robot.update()
        }

    }

}



@TeleOp
class activeintake_test: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(false)
        robot.base_init(this)
        val servo = hardwareMap.crservo.get("CHUB_ARM_INTAKE")
        val servo2 = hardwareMap.crservo.get("EHUB_ARM_INTAKE")

        DISABLE_CAM = true
        waitForStart()
        while(!isStopRequested){
           servo2.power = 1.0
            servo.power = -1.0
            robot.update()
        }
    }

}
