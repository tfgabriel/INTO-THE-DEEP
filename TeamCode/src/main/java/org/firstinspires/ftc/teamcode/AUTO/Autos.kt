package org.firstinspires.ftc.teamcode.AUTO

import android.annotation.SuppressLint
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.pose_set
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.WaitUntilCommand
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.current_pos
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.hPDF
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.path
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.xPDF
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.yPDF
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_p
import org.firstinspires.ftc.teamcode.P2P.red_vars_specimen
import org.firstinspires.ftc.teamcode.P2P.red_vars_specimen.spec0
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.complex_commands
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.TELEOP.current_command
import org.firstinspires.ftc.teamcode.TELEOP.set_extendo_f

@Autonomous
class AutoTest: LinearOpMode() {
    @SuppressLint("DefaultLocale")
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        robot.init_systems()
        localizer.resetTracking()
        waitForStart()
        localizer.resetTracking()
        current_command = SequentialCommand(
            InstantCommand { p2p.followpath(red_vars_specimen.spec0) }
        )
        while (!isStopRequested){

            xPDF = PDF(x_p, x_d, x_f)
            yPDF = PDF(y_p, y_d, y_f)
            hPDF = PDF(h_p, h_d, h_f)

            path = red_vars_specimen.spec0

            p2p.update()

            send_toall("imu", String.format("%.4f",imew.yaw))

            send_toall("endpos", String.format("%.4f",red_vars_specimen.spec0.x) + " " +String.format("%.4f",red_vars_specimen.spec0.y) + " " + String.format("%.4f",red_vars_specimen.spec0.h))

            send_toall("curr pos", String.format("%.4f", current_pos.x) + " " +String.format("%.4f",current_pos.y) + " " + String.format("%.4f", current_pos.h))

            send_toall("angdiff", String.format("%.4f", h_err))

            send_toall("pid x value", String.format("%.4f",xPDF.update(x_err)))
            send_toall("pid y value", String.format("%.4f", yPDF.update(y_err)))
            send_toall("pid h value", String.format("%.4f",hPDF.update(h_err)))

            setLift()
            setExtendo(0.0)

            send_toall("is in tolerance", p2p.isBotinTolerance())
            robot.update()
        }
    }
}

@Autonomous
class RedSample: LinearOpMode(){
    @SuppressLint("DefaultLocale")
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        robot.init_systems()
        robot.init_positions(true)
        localizer.resetTracking()
        err = Pose()
        path = spec0
        waitForStart()
        localizer.resetTracking()
        setExtendoTarget(1)
        current_command = SequentialCommand(
            InstantCommand { p2p.followpath(Pose(0.0, 34.5, 0.0)) },
            setArmState(1),
            InstantCommand { setLiftTarget(3) },
            WaitUntilCommand { p2p.isBotinTolerance()},
            WaitUntilCommand { isLiftinTolerance() },
            SleepCommand(2.0),
            InstantCommand { setLiftTarget(0) },
            SleepCommand(0.25),
            complex_commands.place_specimen(),
            SleepCommand(1.0),
            InstantCommand { p2p.followpath(Pose(80.0, 8.0, 0.0))}

        )
        while (!isStopRequested){
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }



            p2p.update()
            setLift()
            setExtendo(0.0)

            robot.update()

            send_toall("imu", String.format("%.4f",imew.yaw))

            send_toall("is in tolerance", p2p.isBotinTolerance())

            send_toall("endpos", String.format("%.4f",red_vars_specimen.spec0.x) + " " +String.format("%.4f",red_vars_specimen.spec0.y) + " " + String.format("%.4f",red_vars_specimen.spec0.h))

            send_toall("curr pos", String.format("%.4f", current_pos.x) + " " +String.format("%.4f",current_pos.y) + " " + String.format("%.4f", current_pos.h))

            send_toall("angdiff", String.format("%.4f", h_err))

            send_toall("pid x value", String.format("%.4f",xPDF.update(x_err)))
            send_toall("pid y value", String.format("%.4f", yPDF.update(y_err)))
            send_toall("pid h value", String.format("%.4f",hPDF.update(h_err)))

        }}

}

@Autonomous
class RedSpecimen: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        robot.init_systems()
        robot.init_positions(true)
        localizer.resetTracking()
        err = Pose()
        path = spec0
        waitForStart()
        localizer.resetTracking()
        setExtendoTarget(1)
        current_command = SequentialCommand(
            InstantCommand { p2p.followpath(red_vars_specimen.spec0) },
            setArmState(1),
            InstantCommand { setLiftTarget(3) },
            WaitUntilCommand { p2p.isBotinTolerance()},
            WaitUntilCommand { isLiftinTolerance() },
            InstantCommand { setLiftTarget(0) },
            complex_commands.place_specimen(),
            SleepCommand(1.0),
            InstantCommand { p2p.followpath(Pose(0.0, 35.0, 0.0))}

        )
        while (!isStopRequested){
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }



            p2p.update()
            setLift()
            setExtendo(0.0)

            robot.update()

            send_toall("imu", String.format("%.4f",imew.yaw))

            send_toall("is in tolerance", p2p.isBotinTolerance())

            send_toall("endpos", String.format("%.4f",red_vars_specimen.spec0.x) + " " +String.format("%.4f",red_vars_specimen.spec0.y) + " " + String.format("%.4f",red_vars_specimen.spec0.h))

            send_toall("curr pos", String.format("%.4f", current_pos.x) + " " +String.format("%.4f",current_pos.y) + " " + String.format("%.4f", current_pos.h))

            send_toall("angdiff", String.format("%.4f", h_err))

            send_toall("pid x value", String.format("%.4f",xPDF.update(x_err)))
            send_toall("pid y value", String.format("%.4f", yPDF.update(y_err)))
            send_toall("pid h value", String.format("%.4f",hPDF.update(h_err)))

        }
    }

}

@Autonomous
class BlueSample: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        robot.init_systems()
        robot.init_positions(true)
        localizer.resetTracking()
        err = Pose()
        path = spec0
        waitForStart()
        localizer.resetTracking()
        setExtendoTarget(1)
        current_command = SequentialCommand(
            InstantCommand { p2p.followpath(Pose(0.0, 35.0, 0.0)) },
            setArmState(1),
            InstantCommand { setLiftTarget(3) },
            WaitUntilCommand { p2p.isBotinTolerance()},
            WaitUntilCommand { isLiftinTolerance() },
            SleepCommand(2.0),
            InstantCommand { setLiftTarget(0) },
            SleepCommand(0.25),
            complex_commands.place_specimen(),
            SleepCommand(1.0),
            InstantCommand { p2p.followpath(Pose(90.0, 2.0, 0.0))},
            )
        while (!isStopRequested){
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }



            p2p.update()
            setLift()
            setExtendo(0.0)

            robot.update()

            send_toall("imu", String.format("%.4f",imew.yaw))

            send_toall("is in tolerance", p2p.isBotinTolerance())

            send_toall("endpos", String.format("%.4f",red_vars_specimen.spec0.x) + " " +String.format("%.4f",red_vars_specimen.spec0.y) + " " + String.format("%.4f",red_vars_specimen.spec0.h))

            send_toall("curr pos", String.format("%.4f", current_pos.x) + " " +String.format("%.4f",current_pos.y) + " " + String.format("%.4f", current_pos.h))

            send_toall("angdiff", String.format("%.4f", h_err))

            send_toall("pid x value", String.format("%.4f",xPDF.update(x_err)))
            send_toall("pid y value", String.format("%.4f", yPDF.update(y_err)))
            send_toall("pid h value", String.format("%.4f",hPDF.update(h_err)))

        }
    }

}

@Autonomous
class BlueSpecimen: LinearOpMode(){
    override fun runOpMode() {

        val robot = robot(true, true, false)
        robot.start(this)
        robot.init_systems()
        robot.init_positions(true)
        localizer.resetTracking()
        err = Pose()
        path = spec0
        waitForStart()
        localizer.resetTracking()
        setExtendoTarget(1)
        current_command = SequentialCommand(
            InstantCommand { p2p.followpath(red_vars_specimen.spec0) },
            setArmState(1),
            InstantCommand { setLiftTarget(3) },
            WaitUntilCommand { p2p.isBotinTolerance()},
            WaitUntilCommand { isLiftinTolerance() },
            InstantCommand { setLiftTarget(0) },
            complex_commands.place_specimen(),
            SleepCommand(1.0),
            InstantCommand { p2p.followpath(Pose(0.0, 35.0, 0.0))}

        )
        while (!isStopRequested){
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }



            p2p.update()
            setLift()
            setExtendo(0.0)

            robot.update()

            send_toall("imu", String.format("%.4f",imew.yaw))

            send_toall("is in tolerance", p2p.isBotinTolerance())

            send_toall("endpos", String.format("%.4f",red_vars_specimen.spec0.x) + " " +String.format("%.4f",red_vars_specimen.spec0.y) + " " + String.format("%.4f",red_vars_specimen.spec0.h))

            send_toall("curr pos", String.format("%.4f", current_pos.x) + " " +String.format("%.4f",current_pos.y) + " " + String.format("%.4f", current_pos.h))

            send_toall("angdiff", String.format("%.4f", h_err))

            send_toall("pid x value", String.format("%.4f",xPDF.update(x_err)))
            send_toall("pid y value", String.format("%.4f", yPDF.update(y_err)))
            send_toall("pid h value", String.format("%.4f",hPDF.update(h_err)))

        }
    }

}