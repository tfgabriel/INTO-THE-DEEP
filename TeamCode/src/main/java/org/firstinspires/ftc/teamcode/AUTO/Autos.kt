package org.firstinspires.ftc.teamcode.AUTO

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.clamp
import org.firstinspires.ftc.teamcode.ALGORITHMS.Path
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.Trajectory
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.test_pose
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.bomboclaat
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.bomboclaat_0
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.co1
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.co2
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.co3
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.extension_0
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.rotate
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_8
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.s_9
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_gen
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_offset
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_steal
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleepy_extend_from_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleepy_extend_third_impact
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.spinny_baby
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.spinny_baby3
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.steal_withff
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.testp
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.the_third_children
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wait_move
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wait_take
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wait_theydontloveyoulikeiloveyou
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wo1
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wo2
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.wo3
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
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.dunkmid
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.park2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.park3
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.rotate1
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.rotate2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.rotatemid
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample2_examination
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_1
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_2
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sample_three
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.sleepExtendoThird
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.wait_takeS
import org.firstinspires.ftc.teamcode.AUTO.sample_vars.waitaminute
import org.firstinspires.ftc.teamcode.BOT_CONFIG.Robot
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
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.setCol
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
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinLooseMaxTolerance
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


@Config
object AutoTestVars {
    @JvmField
    var moving = true

    @JvmField
    var test_pose = Pose()
}

@Config
object auto_commands {
    fun transfer(): Command {
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

    fun retract(): Command {
        return SequentialCommand(
            setIntakeState(0),
            setWrist(),
            InstantCommand { setExtendoTarget(0) },
            InstantCommand { send_toall("is", "retracting") }
        )
    }

    fun place_specimen(): Command {
        return SequentialCommand(
            InstantCommand { setLiftTarget(0) },
            SleepCommand(0.4),
            setClawState(1),
            SleepCommand(0.1),
            setOuttake(1),
            InstantCommand { send_toall("is", "placing specimen") }
        )
    }


    fun goto_chamber(offset: Pose): Command {
        return SequentialCommand(
            setOuttake(2),
            InstantCommand { setLiftTarget(3) },
            InstantCommand { p2p.followpath(score_gen + offset)},
            InstantCommand { send_toall("is", "going to chamber") }
        )
    }


    fun take(pos: Double, isSample: Boolean): Command {

        val wristcommand = if (pos != 0.0)
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
            SleepCommand(if (!isSample) wait_take else wait_takeS),
            setIntakeState(2),
            SleepCommand(if (!isSample) wait_take else wait_takeS),
            setClawIntakeState(1),
            SleepCommand(if (!isSample) wait_take else wait_takeS),
            setIntakeState(1),
            InstantCommand { send_toall("is", "taking") }
        )
    }
}

@Autonomous
class SpecimenPrime : LinearOpMode() {
    var cdist = 10.0
    var ep = ElapsedTime()
    fun extMove(): Boolean {
        setExtendoTargetLinear(cdist.toInt())
        cdist += linearopmode.gamepad1.left_stick_x * 50 / ep.seconds()
        cdist = clamp(cdist, max_examination.toDouble(), 0.0)
        send_toall("ExtDist", cdist)

        return linearopmode.gamepad1.a
    }

    fun cycle(offsetscore: Pose, offsetTake: Pose): Command{
        return SequentialCommand(

            // adjusting pose
            InstantCommand { p2p.followpath(s_8) },
            WaitUntilCommand { p2p.done },

            //steal pose
            InstantCommand { p2p.followpath(steal_withff + offsetTake) },
            WaitUntilCommand { p2p.done },

            //steal
            SleepCommand(0.2),
            setClawState(0),
            SleepCommand(0.2),


            //goto chamber
            setOuttake(2),
            InstantCommand { setLiftTarget(3) },
            SleepCommand(sleep_steal),
            InstantCommand { p2p.followpath(score_gen + offsetscore)},
            WaitUntilCommand { p2p.done },

            //place specimen
            InstantCommand { setLiftTarget(0) },
            SleepCommand(0.4),
            setClawState(1),
            SleepCommand(0.1),
            setOuttake(1),

            //rotate and prepare to leave
            InstantCommand { p2p.followpath(rotate) },
            WaitUntilCommand { p2p.done },

            //reset arm for steal
            setOuttake(4),
        )
    }

    override fun runOpMode() {
        isAuto = true
        val robot = Robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            //score preload
            InstantCommand { setExtendoTarget(0) },
            SequentialCommand(
                InstantCommand { setLiftTarget(3) },
                SleepCommand(0.2),
                setOuttake(2),
                InstantCommand { p2p.followpath(score_preload) },
            ),
            WaitUntilCommand { p2p.done },

            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                SleepCommand(0.4),
                setClawState(1),
                SleepCommand(0.2),
                setOuttake(1),
                InstantCommand { p2p.followpath(wait_theydontloveyoulikeiloveyou) },
                WaitUntilCommand { p2p.done },
                InstantCommand { p2p.followpath(bomboclaat_0) },
            ),
            setOuttake(4),
            WaitUntilCommand { p2p.done },
            SleepCommand(sleepy_extend_from_preload),
            InstantCommand { setExtendoTargetLinear(extension_0) },
            SleepCommand(0.05),

            setIntakeState(1),
            //pickup first sample
            SequentialCommand(
                ParallelCommand(
                    setIntakeState(1),
                    InstantCommand { intake.wrist.position = wrist_one },
                    setClawIntakeState(0),
                ),
                WaitUntilCommand { isExtendoinTolerance() && p2p.done },
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
                InstantCommand { setExtendoTarget(2) },
                InstantCommand { p2p.followpath(spinny_baby) },
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.done && isExtendoinTolerance() },
                setClawIntakeState(0),
                setIntakeState(1)
            ),

            //pickup second sample
            InstantCommand { p2p.followpath(bomboclaat) },

            setIntakeState(1),
            WaitUntilCommand { p2p.done },
            take(wrist_two, false),

            //dropoff second sample
            SequentialCommand(
                InstantCommand { p2p.followpath(spinny_baby) },
                SleepCommand(wait_move),
                WaitUntilCommand { p2p.done },
                setClawIntakeState(0),
                setIntakeState(1),
            ),

            //pickup third sample
            InstantCommand { intake.wrist.position = 0.9 },
            InstantCommand { setExtendoTarget(1) },
            InstantCommand { p2p.followpath(the_third_children) },
            InstantCommand { intake.wrist.position = 0.9 },
            WaitUntilCommand { p2p.done },
            InstantCommand { setExtendoTarget(2) },
            InstantCommand { setExtendoPowers(1.0) },
            setIntakeState(1),
            InstantCommand { intake.wrist.position = 0.9 },
            WaitUntilCommand { p2p.done },
            SleepCommand(sleepy_extend_third_impact),
            take(wrist_three, false),
            InstantCommand { setExtendoPowers(0.0) },

            InstantCommand { setExtendoTarget(2) },
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
            place_specimen(),

            InstantCommand { p2p.followpath(rotate) },
            WaitUntilCommand { p2p.done },
            setOuttake(4),

            //cycle 1
            cycle(co1, wo1),

            //cycle 2
            cycle(co2, wo2),

            //cycle 3
            cycle(co3, wo3),
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
class Sample : LinearOpMode() {
    private fun waitPExst() =
        SequentialCommand(
            WaitUntilCommand {p2p.done},
            InstantCommand {setCol(Color.rgb(0, 255, 0)) } ,
            WaitUntilCommand { isExtendoinTolerance() },
            InstantCommand {setCol(Color.rgb(255, 255, 255)) }
        )

    private fun doDunk(pose: Pose) =
        SequentialCommand(
            InstantCommand { p2p.followpath(pose) },
            WaitUntilCommand { isLiftinLooseMaxTolerance() },
            setOuttake(3),
            setOuttakeFourbar(3),

            setOuttake(3),
            setOuttakeFourbar(3),
            setOuttake(3),
            setOuttakeFourbar(3),
            setOuttake(3),
            setOuttakeFourbar(3),
            WaitUntilCommand { p2p.done && isLiftinMaxTolerance() },
            setOuttake(3),
            setOuttakeFourbar(3),
            SleepCommand(0.3),
            setClawState(1),
            SleepCommand(0.4),
            setOuttake(1)
        )

    private fun doGrab(isThird: Boolean = false) =
        SequentialCommand(
            if (isThird) InstantCommand { intake.wrist.position = 0.9 } else setWrist(),
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
        )

    override fun runOpMode() {
        isAuto = true
//        liftl_first_open.opened = false
//        liftr_first_open.opened = false
//        extendo_first_open.opened = false
        val robot = Robot(true, true, false)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            InstantCommand { setExtendoTarget(0) },
            SequentialCommand(
                /// Start to dunk preload
                InstantCommand { setLiftTarget(6) },
                InstantCommand { p2p.followpath(rotate1) },
                WaitUntilCommand { p2p.done },
                doDunk(dunk),
            ),

            SequentialCommand(
                /// Preload to sample 1
                InstantCommand { p2p.followpath(rotatemid) },
                WaitUntilCommand { p2p.done },
                InstantCommand { setLiftTarget(0) },
                InstantCommand { p2p.followpath(sample_1) },

                InstantCommand { setExtendoTarget(2) },

                setWrist(),
                SleepCommand(0.15),
                setIntakeState(1), setClawIntakeState(0),
                SleepCommand(waitaminute),

                waitPExst(),
                doGrab(),
            ),

            SequentialCommand(
                /// Grab 1 to Dunk 1
                InstantCommand { p2p.followpath(rotate1) },
                setIntakeState(0),
                SleepCommand(0.2),
                retract(), transfer(),

                InstantCommand { setLiftTarget(6) },
                WaitUntilCommand { p2p.done && isLiftinMaxTolerance() },

                doDunk(dunk),
            ),

            SequentialCommand(
                /// Dunk 1 to Grab 2
                InstantCommand { p2p.followpath(rotatemid) },
                WaitUntilCommand { p2p.done },
                InstantCommand { p2p.followpath(sample_2) },
                InstantCommand { setExtendoTargetLinear(sample2_examination) },
                SleepCommand(waitaminute),

                InstantCommand { setLiftTarget(0) },
                waitPExst(),
                doGrab(),
            ),

            SequentialCommand(
                /// Grab 2 to Dunk 2
                InstantCommand { p2p.followpath(rotate2) },
                setIntakeState(0),
                SleepCommand(0.2),
                retract(),
                transfer(),

                InstantCommand { setLiftTarget(6) },
                WaitUntilCommand { p2p.done && isLiftinMaxTolerance() },

                doDunk(dunkmid),
            ),

            SequentialCommand(
                /// Dunk 2 to Grab 3
                InstantCommand { p2p.followpath(sample_three) },

                SleepCommand(waitaminute),
                InstantCommand { setLiftTarget(0) },

                WaitUntilCommand { p2p.done },
                InstantCommand { setExtendoTarget(2) },
                SleepCommand(0.1),
                setIntakeState(1),
                InstantCommand { intake.wrist.position = 0.9 },
                SleepCommand(sleepExtendoThird),
                InstantCommand { setExtendoPowers(1.0) },
                doGrab(true),
            ),

            SequentialCommand(
                /// Grab 3 to Dunk 3
               // InstantCommand { p2p.followpath(rotate1) },
                InstantCommand { setExtendoPowers(0.0) },
                retract(),
                InstantCommand { p2p.followpath(dunk2)},
                transfer(),

                InstantCommand { setLiftTarget(6) },
                WaitUntilCommand { p2p.done  },

                doDunk(dunk2), // to pose Dunk2
            ),

            SequentialCommand( /// Park
                InstantCommand { p2p.followpath(park2) },
                SleepCommand(0.1),
                InstantCommand { setLiftTarget(0) },
                setOuttake(3),
                WaitUntilCommand { p2p.done },
                InstantCommand { p2p.followpath(park3) },
                WaitUntilCommand { p2p.done }
            )
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
class testy : LinearOpMode() {
    fun getc() = testp
    override fun runOpMode() {
        val robot = Robot(true)
        robot.start(this)
        DISABLE_CAM = true
        localizer.reset()
        current_command = SequentialCommand(
            InstantCommand { p2p.followpath(getc()) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(getc()*2.0) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(getc()*3.0) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(getc()*4.0) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(getc()*3.0) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(getc()*2.0) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(getc()*1.0) },
            WaitUntilCommand { p2p.done },
            InstantCommand { p2p.followpath(getc()*0.0) },
            WaitUntilCommand { p2p.done },
        )

        waitForStart()
        while (!isStopRequested) {
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
object pp_test {
    @JvmField
    var test_pose = Pose()
}


@Autonomous
class PPTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(true, true, false)
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
