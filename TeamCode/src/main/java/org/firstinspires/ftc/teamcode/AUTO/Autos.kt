package org.firstinspires.ftc.teamcode.AUTO

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.clamp
import org.firstinspires.ftc.teamcode.ALGORITHMS.Path
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.Trajectory
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec4D
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.test_pose
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.alexia_offset
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.alexia_wait
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.from_preload_to_samples
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.matei_clumsy
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.matei_sleepy
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.rotate
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_8
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_9
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_s
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_offset
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_with_rotation
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.second_sample
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_score
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleepy_extend_from_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleepy_extend_third_impact
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.spinny_baby
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.spinny_baby2
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.spinny_baby3
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.testp
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.the_third_children
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wait_move
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wait_take
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wait_theydontloveyoulikeiloveyou
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wrist_one
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wrist_three
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wrist_two
import org.firstinspires.ftc.teamcode.AUTO.auto_commands.goto_chamber
import org.firstinspires.ftc.teamcode.AUTO.auto_commands.place_specimen
import org.firstinspires.ftc.teamcode.AUTO.auto_commands.retract
import org.firstinspires.ftc.teamcode.AUTO.auto_commands.take
import org.firstinspires.ftc.teamcode.AUTO.auto_commands.transfer
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.dunk
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.first_offet
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.pp
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.WaitUntilCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pure_pursuit
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.park2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.park3
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_1
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_three
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.second_offset
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sleepBIG
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sleep_startS
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.third_offset
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.wait_takeS
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.waitaminute
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.linearopmode
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinHomeTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTargetLinear
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.max_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setClawIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setFourbar
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setWrist
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.TELEOPS.DISABLE_CAM

import org.firstinspires.ftc.teamcode.TELEOPS.current_command
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isSpecimenScored
import kotlin.math.PI


@Config
object AutoTestVars {
    @JvmField
    var moving = true

    @JvmField
    var test_pose = Pose()
}

@Config
object auto_commands{
    fun transfer(): Command{
        return SequentialCommand(
            setIntakeState(0),
            WaitUntilCommand { isExtendoinHomeTolerance() },
            SleepCommand(0.55),
            setClawState(0),
            SleepCommand(0.2),
            setClawIntakeState(0),
            SleepCommand(0.2),
            InstantCommand { send_toall("is", "transferring") },
        )
    }

    fun retract(): Command{
        return SequentialCommand(
            setIntakeState(0),
            setWrist(),
            InstantCommand { setExtendoTarget(0) },
            InstantCommand { send_toall("is", "retracting") }
        )
    }

    fun place_specimen(): Command{
        return SequentialCommand(
            InstantCommand { setLiftTarget(0) },
            SleepCommand(0.1),
            setArmState(0),
            SleepCommand(0.15),
            setClawState(2),
            SleepCommand(0.2),
            setClawState(1),
            InstantCommand { send_toall("is", "placing specimen") }
        )
    }

    fun place_sample( pose1: Pose, pose2: Pose): Command{
        return SequentialCommand(
            WaitUntilCommand { isLiftinTolerance() },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(pose1) },
            WaitUntilCommand { p2p.done},
            SleepCommand(0.1),
            setClawState(1),
            InstantCommand { p2p.followpath(pose2)},
            SleepCommand(0.3),
            InstantCommand { setLiftTarget(0) },
            setArmState(0),
            InstantCommand { send_toall("is", "placing sample") }
        )
    }

    fun cycle_specimen(offset: Pose): Command{
        return SequentialCommand(
            InstantCommand { p2p.followpath(alexia_offset + offset) },
            WaitUntilCommand { p2p.done },
            InstantCommand { setExtendoTarget(2)},
            setIntakeState(1),
            setWrist(),
            setClawIntakeState(0),
            SleepCommand(alexia_wait),
            InstantCommand { p2p.followpath(matei_clumsy + offset) },
            WaitUntilCommand { p2p.done},
            setIntakeState(2),
            SleepCommand(matei_sleepy),
            setClawIntakeState(1),
            SleepCommand(0.2),
            InstantCommand { send_toall("is", "cycling specimen") }
        )
    }

    fun cycle_specimen_rotation(pose1: Pose, pose2: Pose, offset1: Pose, offset2: Pose): Command{
        return SequentialCommand(
            ParallelCommand (
                InstantCommand { p2p.followpath(pose1 + offset1)},
                setArmState(1),
                setClawState(1)
            ),

            WaitUntilCommand { p2p.done },
            setClawState(0),
            SleepCommand(0.5),

            SleepCommand(1.0),

            SequentialCommand(
                InstantCommand { setLiftTarget(3) },
                WaitUntilCommand { isLiftinTolerance() },
                InstantCommand { p2p.followpath(pose2 + offset2)},
            ),

            WaitUntilCommand { p2p.done  && isLiftinTolerance() },

            SleepCommand(1.0),

            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                WaitUntilCommand { isSpecimenScored() },
                setClawState(1)
            )

        )
    }
    fun cycle_sample(): Command{
        return SequentialCommand(
            InstantCommand { setExtendoTarget(2) },
            WaitUntilCommand { p2p.done },
            setIntakeState(1),
            setWrist(),
            setClawIntakeState(0),
            InstantCommand { setLiftTarget(0) },
            InstantCommand { send_toall("is", "cycling sample") }
        )
    }


    fun goto_chamber(offset: Pose): Command{
        return SequentialCommand(
            setArmState(1),
            InstantCommand { setLiftTarget(3) },
            SleepCommand(sleep_score),
            InstantCommand { p2p.followpath(score_with_rotation + offset) },
            InstantCommand { send_toall("is", "going to chamber") }
        )
    }

    fun goto_basket(offset: Pose): Command{
        return ParallelCommand(
            InstantCommand { setLiftTarget(6) },
            setArmState(2),
            SleepCommand(0.3),
            InstantCommand { p2p.followpath(dunk + offset)},
            InstantCommand { send_toall("is", "going to basket") }
        )
    }

    fun take(pos: Double, isSample: Boolean): Command{

        val  wristcommand = if(pos != 0.0)
            InstantCommand { intake.wrist.position = pos }
        else
            setWrist()


        return SequentialCommand(

            wristcommand,
            setIntakeState(1),
            ParallelCommand(
                setIntakeState(1),
                setClawIntakeState(0),
            ),
            SleepCommand(if(!isSample) wait_take else wait_takeS),
            setIntakeState(2),
            SleepCommand(if(!isSample) wait_take else wait_takeS),
            setClawIntakeState(1),
            SleepCommand(if(!isSample) wait_take else wait_takeS),
            setIntakeState(1),
            InstantCommand { send_toall("is", "taking") }
        )
    }

    fun take(pos: Double, isSample: Boolean, secondary_wait: Double): Command{

        val  wristcommand = if(pos != 0.0)
            InstantCommand { intake.wrist.position = pos }
        else
            setWrist()


        return SequentialCommand(
            ParallelCommand(
                setIntakeState(1),
                wristcommand,
                setClawIntakeState(0),
            ),
            SleepCommand(if(!isSample) wait_take else wait_takeS + secondary_wait),
            setIntakeState(2),
            SleepCommand(if(!isSample) wait_take else wait_takeS),
            setClawIntakeState(1),
            SleepCommand(if(!isSample) wait_take else wait_takeS),
            setIntakeState(1),
            InstantCommand { send_toall("is", "taking") }
        )
    }
}

@Config
object SpecimenVars {
    @JvmField
    var score_preload = Pose(0.0, -74.0, 0.0, Vec2D(50.0, 55.0), 25.0, Vec4D(20.0, 0.4, 200.0, 200.0))
    @JvmField
    var sleep_preload = 0.00
    @JvmField
    var sleep_score = 0.3


    @JvmField
    var wait_theydontloveyoulikeiloveyou = Pose(0.0, -64.0, 2.2, Vec2D(0.0, 0.0), 0.0, Vec4D(50.0, 0.5, 200.0, 200.0))
    @JvmField
    var s_8 = Pose(-73.0, -40.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(200.0, 0.4, 200.0, 200.0))

    @JvmField
    var s_s = Pose(-73.0, -40.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(80.0, 20.0, 200.0, 200.0))

    @JvmField
    var s_9 = Pose(-73.0, -13.0, PI, Vec2D(10.0, 15.0), 10.0)

    @JvmField
    var wait_move = 0.00

    @JvmField
    var from_preload_to_samples = Pose(-50.0, -60.0, 2.2, Vec2D(20.0, 21.0), 20.0)

    @JvmField
    var second_sample = Pose(-76.0, -61.0, 2.2, Vec2D(14.0, 15.0), 0.0)

    @JvmField
    var wrist_one = 0.1

    @JvmField
    var wrist_two = 0.1

    @JvmField
    var wrist_three = 0.9

    @JvmField
    var spinny_baby = Pose(-50.0, -60.0,0.5, Vec2D(0.0, 0.0), 0.0, Vec4D(10.0, 0.3, 200.0, 200.0))

    @JvmField
    var spinny_baby2 = Pose(-76.0, -61.0,0.5, Vec2D(0.0, 0.0), 0.0, Vec4D(10.0, 0.3, 200.0, 200.0))

    @JvmField
    var wait_take = 0.15

    @JvmField
    var the_third_children = Pose(-85.0, -95.0, Math.toRadians(90.0), Vec2D(8.0, 10.0), 7.0)

    @JvmField
    var spinny_baby3 = Pose(-76.0, -61.0,0.5, Vec2D(0.0, 0.0), 0.0, Vec4D(10.0, 0.3, 200.0, 200.0))

    @JvmField
    var matei_clumsy = Pose(-20.0, -55.0, 0.7, 1.0)

    @JvmField
    var alexia_offset = Pose(-15.0, -65.0, 0.7, 1.0)

    @JvmField
    var matei_sleepy = 0.2

    @JvmField
    var score_offset = Pose(-1.0, 1.0, 0.0, 0.6)

    @JvmField
    var sleepy_extend_from_preload = 0.2

    @JvmField
    var sleepy_extend_third_impact = 0.6

    @JvmField
    var alexia_wait = 0.2

    @JvmField
    var score_with_rotation = Pose(20.0, -66.0, 0.0, Vec2D(10.0, 15.0), 15.0, Vec4D(50.0, 0.2, 200.0, 200.0))

    @JvmField
    var rotate = Pose(-70.0, -40.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(34.0, 1.1, 200.0, 200.0))

    @JvmField
    var testp = Pose(-80.0, 60.0, PI, 1.0)

}

@Autonomous
class SpecimenPrime: LinearOpMode() {
    var cdist = 10.0
    var ep = ElapsedTime()
    fun extMove(): Boolean {
        setExtendoTargetLinear(cdist.toInt())
        cdist += linearopmode.gamepad1.left_stick_x  * 50 / ep.seconds()
        cdist = clamp(cdist, max_examination.toDouble(), 0.0)
        send_toall("ExtDist", cdist)

        return linearopmode.gamepad1.a
    }

    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            //score preload
            InstantCommand { setExtendoTarget(0) },
            SequentialCommand(
                InstantCommand { setLiftTarget(3) },
                setArmState(1),
                SleepCommand(0.27),
                InstantCommand { p2p.followpath(score_preload) },
            ),
            WaitUntilCommand { p2p.done },

            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                setArmState(0),
                SleepCommand(0.2),
                setClawState(2),
                SleepCommand(0.2),
                setClawState(1),
                InstantCommand { p2p.followpath(wait_theydontloveyoulikeiloveyou) },
                WaitUntilCommand { p2p.done },
                InstantCommand { p2p.followpath(from_preload_to_samples) },
            ),
            WaitUntilCommand { p2p.done },
            WaitUntilCommand { extMove() },
            SleepCommand(sleepy_extend_from_preload),
            InstantCommand { setExtendoTarget(2) },
            SleepCommand(0.05),
            //pickup first sample
            SequentialCommand(
                ParallelCommand(
                    setIntakeState(1),
                    InstantCommand { intake.wrist.position = wrist_one},
                    setClawIntakeState(0),
                ),
                WaitUntilCommand { isExtendoinTolerance() && p2p.done},
                SleepCommand(wait_take),
                setIntakeState(2),
                SleepCommand(wait_take),
                setClawIntakeState(1),
                SleepCommand(wait_take),
                setIntakeState(1),
                InstantCommand { send_toall("is", "taking") }
            ),

            //dropoff first sample
            SequentialCommand(
                InstantCommand { p2p.followpath(spinny_baby) },
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.done },
                setClawIntakeState(0),
                setIntakeState(1)
            ),

            //pickup second sample
            InstantCommand { p2p.followpath(second_sample) },
            WaitUntilCommand { p2p.done },
            take(wrist_two, false),

            //dropoff second sample
            SequentialCommand(
                InstantCommand { p2p.followpath(spinny_baby2) },
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.done },
                setClawIntakeState(0),
                setIntakeState(1),
            ),

            //pickup third sample
            InstantCommand { setExtendoTarget(1) },
            InstantCommand { p2p.followpath(the_third_children) },
            InstantCommand { intake.wrist.position = 0.9},
            SleepCommand(sleepy_extend_third_impact),
            InstantCommand { setExtendoTarget(2) },
            InstantCommand { intake.wrist.position = 0.9},
            WaitUntilCommand { p2p.done && isExtendoinTolerance() },
            take(wrist_three, false),

            //dropoff third sample
            SequentialCommand(
                setIntakeState(1),
                setWrist(),
                InstantCommand { p2p.followpath(spinny_baby3) },
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.done },
                setClawIntakeState(0)
            ),

            //take wall specimen
            retract(),
            SequentialCommand(
                setArmState(1),
                InstantCommand { p2p.followpath(s_8) },
                WaitUntilCommand { p2p.done },
                InstantCommand { p2p.followpath(s_9) },
                WaitUntilCommand { p2p.done },
                SleepCommand(0.2),
                setClawState(0),
                SleepCommand(0.2),
            ),

            //score wall specimen

            goto_chamber(score_offset),
            WaitUntilCommand { p2p.done },
            SleepCommand(sleep_preload),
            place_specimen(),

            InstantCommand { p2p.followpath(rotate) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(s_s) },
            WaitUntilCommand { p2p.done },

            SequentialCommand(
                setArmState(1),
                setClawState(1),
                InstantCommand { p2p.followpath(s_9) },
                WaitUntilCommand { p2p.done },

                setArmState(1),
                setArmState(1),
                setArmState(1),
                setArmState(1),
                setArmState(1),
                setArmState(1),
                SleepCommand(0.2),
                setClawState(0),
                SleepCommand(0.2),
            ),

            //score wall specimen
            goto_chamber(score_offset),
            WaitUntilCommand { p2p.done },
            SleepCommand(sleep_preload),
            place_specimen(),

            InstantCommand { p2p.followpath(rotate) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(s_s) },
            WaitUntilCommand { p2p.done },
            SequentialCommand(
                setArmState(1),
                setClawState(1),
                InstantCommand { p2p.followpath(s_9) },
                WaitUntilCommand { p2p.done },
                SleepCommand(0.2),
                setClawState(0),
                SleepCommand(0.2),
            ),

            //score wall specimen
            goto_chamber(score_offset),
            WaitUntilCommand { p2p.done },
            SleepCommand(sleep_preload),
            place_specimen(),


            InstantCommand { p2p.followpath(rotate) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(s_s) },
            WaitUntilCommand { p2p.done },
            SequentialCommand(
                setArmState(1),
                setClawState(1),
                InstantCommand { p2p.followpath(s_9) },
                WaitUntilCommand { p2p.done },
                SleepCommand(0.2),
                setArmState(1),
                setClawState(0),
                SleepCommand(0.2),
            ),

            //score wall specimen
            goto_chamber(score_offset),
            WaitUntilCommand { p2p.done },
            SleepCommand(sleep_preload),
            place_specimen(),

            InstantCommand { p2p.followpath(rotate) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(s_s) },
            WaitUntilCommand { p2p.done },
            //park
            SequentialCommand(
                setArmState(1),
                setClawState(1),
                InstantCommand { p2p.followpath(s_9) },
                WaitUntilCommand { p2p.done },
                SleepCommand(0.2),
                setClawState(1),
                SleepCommand(0.2),
            ),

            //score wall specimen
            goto_chamber(score_offset),
            WaitUntilCommand { p2p.done },
            SleepCommand(sleep_preload),
            place_specimen(),

        )

        waitForStart()
        val elap: ElapsedTime = ElapsedTime()
        while (!isStopRequested) {
            send_toall("time", elap.seconds())

            if (current_command != null) {
                if (current_command!!.run(telemetry_packet)) {
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


@Config
object sample_vars{
    @JvmField
    var dunk = Pose(-23.0, -44.0, Math.toRadians(45.0), 0.5)

    @JvmField
    var sample_1 = Pose(-23.3, -30.7, 1.65, 0.6)

    @JvmField
    var sample_2 = Pose(-22.6, -59.2, 1.58, 0.6)
    @JvmField
    var sample_three = Pose(-92.0, -12.5, Math.toRadians(180.0), 0.7)

    @JvmField
    var wait_takeS = 0.27

    @JvmField
    var score = Pose(-19.0, -40.0, Math.toRadians(35.0), 0.9)

    @JvmField
    var waitaminute = 0.3
    @JvmField
    var sleep_startS = 0.85

    @JvmField
    var offset = Pose()

    @JvmField
    var sleepBIG = 0.9

    @JvmField
    var park2 = Pose(-140.0, 0.0, 3.3, 1.0)

    @JvmField
    var park3 = Pose(-140.0, 35.0, 3.3, 1.0)

    @JvmField
    var first_offet =  Pose(-2.0, -0.0, -0.03, 0.0)

    @JvmField
    var second_offset = Pose(-3.1, -3.5, -0.03, 0.0)

    @JvmField
    var third_offset = Pose(-2.0, -2.5, -0.03 , 0.0)
}

@Autonomous
class Sample: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            InstantCommand { setExtendoTarget(0) },
            SequentialCommand(
                InstantCommand { setLiftTarget(6) },
                setArmState(2),
                SleepCommand(sleepBIG),
                InstantCommand { p2p.followpath(dunk)}
            ),

            WaitUntilCommand { p2p.done  && isLiftinTolerance() },
            SleepCommand(0.2),
            setClawState(1),
            SleepCommand(0.4),

            InstantCommand { p2p.followpath(sample_1)},
            InstantCommand { setExtendoTarget(2) },

            SleepCommand(waitaminute),
            ParallelCommand(
                InstantCommand {setLiftTarget(0) },
                setArmState(0)
            ),

            WaitUntilCommand { p2p.done  && isExtendoinTolerance() },
            take(0.0, true),

            retract(),
            transfer(),

            InstantCommand { setLiftTarget(6) },
            setArmState(2),
            SleepCommand(sleep_startS),
            InstantCommand { p2p.followpath(dunk + first_offet)},
            WaitUntilCommand { p2p.done},

            WaitUntilCommand { p2p.done  && isLiftinTolerance() },
            SleepCommand(0.2),
            setClawState(1),
            SleepCommand(0.4),

            ParallelCommand(
                InstantCommand { p2p.followpath(sample_2)},
                InstantCommand { setExtendoTarget(2) },
            ),

            SleepCommand(waitaminute),
            ParallelCommand(
                InstantCommand {setLiftTarget(0) },
                setArmState(0)

            ),

            WaitUntilCommand { p2p.done  && isExtendoinTolerance() },
            take(0.0, true),

            retract(),
            transfer(),

            InstantCommand { setLiftTarget(6) },
            setArmState(2),
            SleepCommand(sleep_startS),

            InstantCommand { p2p.followpath(dunk + second_offset)},
            WaitUntilCommand { p2p.done },

            WaitUntilCommand { p2p.done  && isLiftinTolerance() },
            SleepCommand(0.2),
            setClawState(1),
            SleepCommand(0.4),

            ParallelCommand(
                InstantCommand { p2p.followpath(sample_three)},
            ),

            SleepCommand(waitaminute),
            ParallelCommand(
                InstantCommand {setLiftTarget(0) },
                setArmState(0)
            ),

            WaitUntilCommand { p2p.done },
            InstantCommand { setExtendoTarget(2) },
            SleepCommand(0.3),
            setIntakeState(1),
            setFourbar(1),
            WaitUntilCommand { isExtendoinTolerance() },
            take(0.09, true, 0.2),

            retract(),
            transfer(),

            InstantCommand { setLiftTarget(6) },
            setArmState(2),
            SleepCommand(sleep_startS),
            InstantCommand { p2p.followpath(dunk + third_offset)},
            WaitUntilCommand { p2p.done && isLiftinTolerance() },

            setClawState(1),
            SleepCommand(0.4),
            InstantCommand { p2p.followpath(park2) },
            SleepCommand(0.2),
            setArmState(0),
            InstantCommand { setLiftTarget(2) },
            InstantCommand { p2p.followpath(park3) },

            )
        waitForStart()
        val elap: ElapsedTime = ElapsedTime()
        while (!isStopRequested) {
            send_toall("time", elap.seconds())

            if (elap.seconds() >= 27.0)
                current_command = ParallelCommand(
                    InstantCommand { p2p.followpath(park2) },
                    SleepCommand(0.2),
                    setArmState(0),
                    InstantCommand { setLiftTarget(2) },
                    InstantCommand { p2p.followpath(park3) },

                    InstantCommand { send_toall("step", "10000000000000000000") },
                )

            if (current_command != null) {
                if (current_command!!.run(telemetry_packet)) {
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





@Autonomous
class testy: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(true)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            InstantCommand { p2p.followpath(testp)},
            InstantCommand { setLiftTarget(3) }
            //InstantCommand { setLiftTarget(3) }
        )

        waitForStart()
        while(!isStopRequested){
            if (current_command != null) {
                if (current_command!!.run(telemetry_packet)) {
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

@Config
object pp_test{
    @JvmField
    var test_pose = Pose()
}


@Autonomous
class PPTest: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        pp = pure_pursuit()
        current_command = InstantCommand { pp.followpath(Trajectory(Path(Pose(), test_pose)))}

        waitForStart()
        val ep = ElapsedTime()
        while(!isStopRequested){
            if(current_command != null){
                if(current_command!!.run(telemetry_packet)){
                    current_command = null
                }
            }

            setLift()
            setExtendo(0.0)
            pp.update()
            robot.update()
        }
    }

}