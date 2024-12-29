package org.firstinspires.ftc.teamcode.AUTO

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.moving
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.test_pose
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.p_s
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.p_t
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.park
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_1
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_11
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_2
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_7
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_8
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_9
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_offset
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_offset_2
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_start
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wait_move
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.pose_set
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.vel
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.WaitUntilCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_p
import org.firstinspires.ftc.teamcode.P2P.red_vars_specimen
import org.firstinspires.ftc.teamcode.P2P.red_vars_specimen.spec0
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.hPDF
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.xPDF
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.yPDF
import org.firstinspires.ftc.teamcode.SYSTEMS.CHASSIS.Chassis
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setArmStateIntake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setClawIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setFourbar
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.complex_commands
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.TELEOP.DISABLE_CAM

import org.firstinspires.ftc.teamcode.TELEOP.current_command
import org.firstinspires.ftc.teamcode.TELEOP.set_extendo_f
import kotlin.math.PI


@Config
object AutoTestVars {
    @JvmField
    var moving = true

    @JvmField
    var test_pose = Pose()
}

@Config
object SpecimenVars {
    @JvmField
    var score_preload = Pose(0.0, -68.0, 0.0,  0.4)

    @JvmField
    var park = Pose(-70.0, -10.0, 0.0, 0.8)

    @JvmField
    var sleep_preload = 0.00

    @JvmField
    var slow = 0.4

    @JvmField
    var sleep_start = 0.42

    @JvmField
    var p_t = Pose(-85.0, -130.0, 0.0, 0.4)
    @JvmField
    var p_s = Pose(-45.0, -60.0, 0.0, 0.7)

    @JvmField
    var s_1 = Pose(-95.0, -125.0, 0.0, 0.9)

    @JvmField
    var s_offset = Pose(-15.0, 0.0, 0.0, 0.0)

    @JvmField
    var s_2 = Pose(-85.0, -20.0, 0.0, 0.5)

    @JvmField
    var s_11 = Pose(-100.0, -125.0, 0.0, 0.4 )

    @JvmField
    var s_7 = Pose(-140.0, -20.0, 0.0, 0.7)

    @JvmField
    var s_offset_2 = Pose(-15.0, 0.0, 0.0, 0.0)

    @JvmField
    var s_8 = Pose(-80.0, -50.0, PI, 0.6)

    @JvmField
    var s_9 = Pose(-80.0, -16.0, PI, 0.5)

    @JvmField
    var wait_move = 0.5
}


@Autonomous
class AutoTest: LinearOpMode() {
    @SuppressLint("DefaultLocale")
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        localizer.reset()
        waitForStart()
        var zerounu = 0
        current_command = InstantCommand { p2p.followpath(test_pose) }
        while (!isStopRequested){

            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }

            p2p.update()
            robot.update()
        }


    }
}

@Autonomous
class Specimen: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            //score preload
            InstantCommand {setExtendoTarget(0) },
            InstantCommand {send_toall("step", "0") },
            SequentialCommand(
                setArmState(1),
                InstantCommand { setLiftTarget(3)},
                SleepCommand(sleep_start),
                InstantCommand { p2p.followpath(score_preload)},
            ),
            WaitUntilCommand { p2p.isBotinTolerance()},
            //WaitUntilCommand { isLiftinTolerance() },
            SleepCommand(sleep_preload),
            InstantCommand {send_toall("step", "1") },
            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                setArmState(0),
                SleepCommand(0.15),
                setClawState(2),
                SleepCommand(0.2),
                setClawState(1),

            ),

            SequentialCommand(
                InstantCommand { p2p.followpath(p_s)},
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.isBotinTolerance()},
                InstantCommand {send_toall("step", "2.5") },
            ),

            SequentialCommand(
                InstantCommand { p2p.followpath(p_t)},
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.isBotinTolerance()},
                InstantCommand {send_toall("step", "2.75") },
            ),

            //first sample
            SequentialCommand(
                InstantCommand { p2p.followpath(s_1+Pose(-10.0, 0.0, 0.0, 0.0))},
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.isBotinTolerance()},
                InstantCommand {send_toall("step", "3") },
            ),


            SequentialCommand(
                InstantCommand { p2p.followpath(s_2)},
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.isBotinTolerance()},
                InstantCommand {send_toall("step", "4") },
            ),

            SequentialCommand(
            //second sample
                InstantCommand { p2p.followpath(s_1+Pose(10.0, 0.0, 0.0, 0.0))},
                WaitUntilCommand { p2p.isBotinTolerance()},
                SleepCommand(wait_move+0.5),
                InstantCommand {send_toall("step", "5") },
            ),

            SequentialCommand(
                InstantCommand { p2p.followpath(s_11+ s_offset+Pose(-15.0,0.0,0.0,0.0))},
                WaitUntilCommand { p2p.isBotinTolerance()},
                SleepCommand(wait_move),
                InstantCommand {send_toall("step", "5.5") },
            ),

            SequentialCommand(
                InstantCommand { p2p.followpath(s_2-Pose(0.0, -10.0, 0.0, 0.0))},
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.isBotinTolerance()},
                SleepCommand(wait_move),
                InstantCommand {send_toall("step", "6") },
            ),

            //third sample
            SequentialCommand(
                //second sample
                InstantCommand { p2p.followpath(s_11+ s_offset)},
                WaitUntilCommand { p2p.isBotinTolerance()},
                SleepCommand(wait_move),
                InstantCommand {send_toall("step", "5.5") },
            ),

            SequentialCommand(
                //third sample
                InstantCommand { p2p.followpath(s_11 + s_offset + s_offset_2)},
                WaitUntilCommand { p2p.isBotinTolerance()},
                SleepCommand(wait_move),
                InstantCommand {send_toall("step", "7") },
            ),


            SequentialCommand(
                InstantCommand { p2p.followpath(s_7) },
                WaitUntilCommand { p2p.isBotinTolerance()},
                SleepCommand(wait_move),
                InstantCommand {send_toall("step", "8") },
            ),


            SleepCommand(0.5),
            SequentialCommand(
                InstantCommand { p2p.followpath(s_8)},
                SleepCommand(0.2),
                InstantCommand {send_toall("step", "11.5") },
                setArmState(1),
                WaitUntilCommand { p2p.isBotinTolerance()},
            ),

            SequentialCommand(
                InstantCommand { p2p.followpath(s_9)},
                SleepCommand(0.2),
                InstantCommand {send_toall("step", "12") },
                WaitUntilCommand { p2p.isBotinTolerance()},
                setClawState(0)
                ),

            SequentialCommand(
                InstantCommand{ setLiftTarget(3) },
                SleepCommand(0.3),
                InstantCommand { p2p.followpath(score_preload+Pose(20.0, 0.0, 0.0, 0.0))},
            ),

            WaitUntilCommand { p2p.isBotinTolerance()},
            //WaitUntilCommand { isLiftinTolerance() },
            SleepCommand(sleep_preload),
            InstantCommand {send_toall("step", "1") },
            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                setArmState(0),
                SleepCommand(0.15),
                setClawState(2),
                SleepCommand(0.2),
                setClawState(1),

                ),


            //park and reset
            /*WaitUntilCommand { p2p.isBotinTolerance() },
            InstantCommand { setLiftTarget(0) },
            InstantCommand { setExtendoTarget(0) },
            InstantCommand { p2p.followpath(park)},
            InstantCommand {send_toall("step", "13") },

             */


            )

        waitForStart()
        val elap: ElapsedTime = ElapsedTime()
        while(!isStopRequested){
            send_toall("time", elap.seconds())

            if(elap.seconds() >= 27.0)
                current_command = SequentialCommand(
                    InstantCommand { setLiftTarget(0) },
                    InstantCommand { setExtendoTarget(0) },
                    InstantCommand { p2p.followpath(park)},
                    InstantCommand {send_toall("step", "10000000000000000000") },
                    )

            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }

            setLift()
            setExtendo(0.0)
            p2p.update()
            robot.update()
        }
    }

}