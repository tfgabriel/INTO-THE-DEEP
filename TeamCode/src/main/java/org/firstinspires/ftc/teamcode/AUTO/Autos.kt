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
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.from_preload_to_samples
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.rotate
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_8
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_9
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_s
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_offset
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_with_rotation
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.second_sample
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_preload
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
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.dunk2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.park2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.park3
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.rotate1
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.rotate2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.rotate3
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.rotatemid
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample2_examination
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_1
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_three
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sleepExtendoThird
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.wait_takeS
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.waitaminute
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.isAuto
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.linearopmode
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.pp
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.WaitUntilCommand
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pure_pursuit
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinHomeTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.isExtendoinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoPowers
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTargetLinear
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.max_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setClawIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setIntakeState
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setWrist
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_yummy
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinMaxTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLift
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.setLiftTarget
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setOuttake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setOuttakeFourbar
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import org.firstinspires.ftc.teamcode.TELEOPS.DISABLE_CAM
import org.firstinspires.ftc.teamcode.TELEOPS.current_command
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
            setOuttake(1),
            setClawState(1),

            WaitUntilCommand { !isExtendoinHomeTolerance() },
            SleepCommand(0.55),

            InstantCommand { intake.fourbar.position = fourbar_yummy },
            SleepCommand(0.1),

            setOuttake(0),
            SleepCommand(0.1),
            setClawState(0),
            SleepCommand(0.2),
            setClawIntakeState(0),
            setOuttake(1)
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
            SleepCommand(0.3),
            setClawState(1),
            SleepCommand(0.1),
            setOuttake(1),
            InstantCommand { send_toall("is", "placing specimen") }
        )
    }



    fun goto_chamber(offset: Pose): Command{
        return SequentialCommand(
            setOuttake(2),
            InstantCommand { setLiftTarget(3) },
            InstantCommand { p2p.followpath(score_with_rotation + offset) },
            InstantCommand { send_toall("is", "going to chamber") }
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
                setOuttake(2),
                InstantCommand { p2p.followpath(score_preload) },
            ),
            WaitUntilCommand { p2p.done },

            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                SleepCommand(0.2),
                setClawState(1),
                SleepCommand(0.2),
                setOuttake(1),
                InstantCommand { p2p.followpath(wait_theydontloveyoulikeiloveyou) },
                WaitUntilCommand { p2p.done },
                InstantCommand { p2p.followpath(from_preload_to_samples) },
            ),
            setOuttake(4),
            WaitUntilCommand { p2p.done },
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

            setOuttake(4),
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
            setOuttake(4),
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


            setOuttake(4),
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
            setOuttake(4),
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

@Autonomous
class Sample: LinearOpMode(){
    override fun runOpMode() {
        isAuto= true
//        liftl_first_open.opened = false
//        liftr_first_open.opened = false
//        extendo_first_open.opened = false
        val robot = robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            InstantCommand { setExtendoTarget(0) },
            SequentialCommand(
                InstantCommand { setLiftTarget(6) },
                InstantCommand { p2p.followpath(rotate1)},
                WaitUntilCommand { p2p.done},
                InstantCommand { p2p.followpath(dunk)}
            ),

            WaitUntilCommand { p2p.done  && isLiftinMaxTolerance() },
            setOuttake(3),
            setOuttakeFourbar(3),
            SleepCommand(0.3),
            setClawState(1),
            SleepCommand(0.2),
            setOuttake(1),

            InstantCommand { p2p.followpath(rotatemid)},
            WaitUntilCommand{p2p.done},
            InstantCommand { setLiftTarget(0) },
            InstantCommand { p2p.followpath(sample_1)},
            InstantCommand { setExtendoTarget(2) },
            setWrist(),
            SleepCommand(0.15),
            setIntakeState(1),
            setClawIntakeState(0),

            SleepCommand(waitaminute),

            WaitUntilCommand { p2p.done  && isExtendoinTolerance() },
            SequentialCommand(
                setWrist(),
                ParallelCommand(
                    setIntakeState(1),
                    setClawIntakeState(0),
                ),
                SleepCommand(wait_takeS + 0.2),
                setIntakeState(2),
                SleepCommand(wait_takeS),
                setClawIntakeState(1),
                SleepCommand(wait_takeS),
                setIntakeState(1),
                setIntakeState(0),
                InstantCommand { send_toall("is", "taking") }
            ),

            InstantCommand { p2p.followpath(rotate1)},
            setIntakeState(0),
            SleepCommand(0.2),
            retract(),
            transfer(),

            InstantCommand { setLiftTarget(6) },
            WaitUntilCommand { p2p.done  && isLiftinMaxTolerance()},

            InstantCommand { p2p.followpath(dunk)},
            WaitUntilCommand { p2p.done  && isLiftinMaxTolerance() },

            setOuttake(3),
            SleepCommand(0.3),
            setClawState(1),
            SleepCommand(0.2),
            setOuttake(1),

            InstantCommand { p2p.followpath(rotatemid)},
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(sample_2)},
            InstantCommand { setExtendoTargetLinear(sample2_examination) },
            SleepCommand(waitaminute),
            InstantCommand {setLiftTarget(0) },

            WaitUntilCommand { p2p.done  && isExtendoinTolerance() },
            SequentialCommand(

                setWrist(),
                ParallelCommand(
                    setIntakeState(1),
                    setClawIntakeState(0),
                ),
                SleepCommand(wait_takeS + 0.2),
                setIntakeState(2),
                SleepCommand(wait_takeS),
                setClawIntakeState(1),
                SleepCommand(wait_takeS),
                setIntakeState(1),
                setIntakeState(0),
                InstantCommand { send_toall("is", "taking") }
            ),


            InstantCommand { p2p.followpath(rotate2)},
            setIntakeState(0),
            SleepCommand(0.2),
            retract(),
            transfer(),

            InstantCommand { setLiftTarget(6) },
            WaitUntilCommand { p2p.done  && isLiftinMaxTolerance() },

            InstantCommand { p2p.followpath(dunk)},
            WaitUntilCommand { p2p.done  && isLiftinMaxTolerance() },

            setOuttake(3),
            SleepCommand(0.3),
            setClawState(1),
            SleepCommand(0.3),
            setOuttake(1),

            ParallelCommand(
                InstantCommand { p2p.followpath(sample_three)},
            ),

            SleepCommand(waitaminute),
            InstantCommand {setLiftTarget(0) },

            WaitUntilCommand { p2p.done },
            InstantCommand { setExtendoTarget(2) },
            SleepCommand(0.2),
            setIntakeState(1),
            InstantCommand { intake.wrist.position = 0.9},
            SleepCommand(sleepExtendoThird),
            InstantCommand { setExtendoPowers(1.0) },
            SequentialCommand(

                ParallelCommand(
                    setIntakeState(1),
                    setClawIntakeState(0),
                ),
                SleepCommand(wait_takeS + 0.2),
                setIntakeState(2),
                SleepCommand(wait_takeS),
                setClawIntakeState(1),
                SleepCommand(wait_takeS),
                setIntakeState(1),
                setIntakeState(0),
                InstantCommand { send_toall("is", "taking") }
            ),

            InstantCommand { p2p.followpath(rotate1)},
            SleepCommand(0.2),
            InstantCommand { setExtendoPowers(0.0) },
            retract(),
            transfer(),

            InstantCommand { setLiftTarget(6) },
            WaitUntilCommand { p2p.done && isLiftinMaxTolerance() },

            InstantCommand { p2p.followpath(dunk2)},
            WaitUntilCommand { p2p.done && isLiftinMaxTolerance() },

            setOuttake(3),
            SleepCommand(0.3),
            setClawState(1),
            SleepCommand(0.2),

            InstantCommand { p2p.followpath(rotate3)},
            InstantCommand { setLiftTarget(0) },
            WaitUntilCommand { p2p.done },

            InstantCommand { p2p.followpath(park2) },
            WaitUntilCommand { p2p.done},
            InstantCommand { p2p.followpath(park3) },
            WaitUntilCommand { p2p.done }

            )
        waitForStart()
        val elap: ElapsedTime = ElapsedTime()
        while (!isStopRequested) {
            send_toall("time", elap.seconds())

            /*if (elap.seconds() >= 28.5)
                current_command = ParallelCommand(
                    InstantCommand { p2p.followpath(park2) },
                    SleepCommand(0.2),
                    setArmState(0),
                    InstantCommand { setLiftTarget(2) },
                    InstantCommand { p2p.followpath(park3) },

                    InstantCommand { send_toall("step", "10000000000000000000") },
                )

             */

            if (current_command != null) {
                if (current_command!!.run(telemetry_packet)) {
                    current_command = null
                }
            }

            send_toall("lift pos ehub", lift.ehub_slides.currentpos)
            send_toall("lift pos chub", lift.chub_slides.currentpos)
            send_toall("lift ehub amps", lift.ehub_slides.amps)
            send_toall("lift chub amps", lift.ehub_slides.amps)
            send_toall("lift target", lift_target)

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
class PPTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        pp = pure_pursuit()
        current_command = InstantCommand { pp.followpath(Trajectory(Path(Pose(), test_pose))) }

        waitForStart()
        val ep = ElapsedTime()
        while (!isStopRequested) {
            if (current_command != null) {
                if (current_command!!.run(telemetry_packet)) {
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
