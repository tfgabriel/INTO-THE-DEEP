package org.firstinspires.ftc.teamcode.AUTO

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.moving
import org.firstinspires.ftc.teamcode.AUTO.AutoTestVars.targetPos
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.first_sample
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.from_preload_to_samples
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.h_off
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.park
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.position_for_specimens
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sample_three
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sample_two
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.score_second_sample
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.second_sample
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_preload
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.sleep_start
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.specimen_wall
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.spinny_baby
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.spinny_baby2
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.take_specimen
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.third_sample
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars.waitforhuman
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.p2p
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.pose_set
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry_packet
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

import org.firstinspires.ftc.teamcode.TELEOP.current_command
import org.firstinspires.ftc.teamcode.TELEOP.set_extendo_f
import kotlin.math.PI


@Config
object AutoTestVars {
    @JvmField
    var targetPos = Pose(0.0, 0.0, 0.0)
    @JvmField
    var moving = true
}

@Config
object SpecimenVars {
    @JvmField
    var score_preload = Pose(0.0, -68.0, 0.0)

    @JvmField
    var park = Pose(-70.0, -10.0, 0.0)

    @JvmField
    var from_preload_to_samples = Pose(-46.0, -55.0, 2.2)

    @JvmField
    var sample_0 = Pose(-90.0, -90.0, 0.0)

    @JvmField
    var drop_samples = Pose(-90.0, 10.0, 0.0)

    @JvmField
    var spinny_baby = Pose(-53.7, -53.5, 0.8)

    @JvmField
    var sleep_preload = 0.00

    @JvmField
    var slow = 0.4

    @JvmField
    var sleep_start = 0.42

    @JvmField
    var first_sample = 0.25

    @JvmField
    var second_sample = 0.25

    @JvmField
    var third_sample = 0.25

    @JvmField
    var sample_two = Pose(-67.5, -62.0, 2.1)

    @JvmField
    var sample_three = Pose(-102.7, -58.0, 1.95)

    @JvmField
    var specimen_wall = Pose(-45.0, 10.0, 3.14159)

    @JvmField
    var spinny_baby2 = Pose(-53.7, -50.5, 0.8)

    @JvmField
    var score_second_sample = Pose(15.0, -69.5, 0.0)

    @JvmField
    var position_for_specimens = Pose(10.0, -70.0, 0.0)

    @JvmField
    var take_specimen = Pose(-65.0, -60.0, 0.8)

    @JvmField
    var waitforhuman = 0.5

    @JvmField
    var h_off = -0.2
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
        p2p.followpath(targetPos)
        while (!isStopRequested){
            if (p2p.done) {
                if (zerounu == 0) {
                    p2p.followpath(Pose())
                    zerounu = 1
                } else {
                    p2p.followpath(targetPos)
                    zerounu = 0
                }
            }

            if (moving) {
                    p2p.update()
            }


                robot.update()
        }


    }
}

@Autonomous
class Specimen: LinearOpMode(){
    override fun runOpMode() {
        val robot = robot(true, true, false)
        robot.start(this)
        localizer.reset()
        current_command = SequentialCommand(
            InstantCommand {setExtendoTarget(1) },
            SequentialCommand(
                setArmState(1),
                InstantCommand { setLiftTarget(3)},
                SleepCommand(sleep_start),
                InstantCommand { p2p.followpath(score_preload)},
            ),
            WaitUntilCommand { p2p.isBotinTolerance()},
            WaitUntilCommand { isLiftinTolerance() },
            SleepCommand(sleep_preload),
            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                setArmState(0),
                SleepCommand(0.15),
                setClawState(2),
                SleepCommand(0.2),
                setClawState(1),
                InstantCommand { p2p.followpath(from_preload_to_samples)}
            ),
            WaitUntilCommand { p2p.isBotinTolerance() },

            InstantCommand { setExtendoTarget(0) },


            SequentialCommand(
                setIntakeState(0),
                WaitUntilCommand { isExtendoinTolerance() },
                setIntakeState(1),
                SleepCommand(0.15),
                setClawIntakeState(0),
                InstantCommand { intake.wrist.position = first_sample},
            ),
            SequentialCommand(
                SleepCommand(0.5),
                setIntakeState(2),
                SleepCommand(0.5),
                setClawIntakeState(1),
                SleepCommand(0.5),
                ),
            InstantCommand { p2p.followpath(spinny_baby)},
            WaitUntilCommand { p2p.isBotinTolerance() },
            SleepCommand(0.7),
            setClawIntakeState(0),

            InstantCommand { p2p.followpath(sample_two)},
            SequentialCommand(
                setIntakeState(0),
                WaitUntilCommand { isExtendoinTolerance() },
                setIntakeState(1),
                SleepCommand(0.15),
                setClawIntakeState(0),
                InstantCommand { intake.wrist.position = first_sample},
            ),
            SequentialCommand(
                SleepCommand(0.5),
                setIntakeState(2),
                SleepCommand(0.5),
                setClawIntakeState(1),
                SleepCommand(0.5),
            ),
            InstantCommand { p2p.followpath(spinny_baby2)},
            WaitUntilCommand { p2p.isBotinTolerance() },
            SleepCommand(0.7),

            setClawIntakeState(0),


            InstantCommand { p2p.followpath(sample_three)},
            SequentialCommand(
                setIntakeState(0),
                WaitUntilCommand { isExtendoinTolerance() },
                setIntakeState(1),
                SleepCommand(0.15),
                SleepCommand(0.3),
                setClawIntakeState(0),
                InstantCommand { intake.wrist.position = third_sample},
            ),
            SequentialCommand(
                SleepCommand(0.5),
                setIntakeState(2),
                SleepCommand(0.5),
                setClawIntakeState(1),
                SleepCommand(0.5),
            ),
            InstantCommand { p2p.followpath(spinny_baby2 + Pose(0.0, 0.0, h_off))},
            WaitUntilCommand { p2p.isBotinTolerance() },
            SleepCommand(0.7),
            setClawIntakeState(0),

            ParallelCommand(
                setIntakeState(1),
                InstantCommand { intake.wrist.position = wrist_neutral},
                InstantCommand{ setExtendoTarget(2) },
            ),
            WaitUntilCommand { isExtendoinTolerance() },

            InstantCommand { p2p.followpath(take_specimen)},
            InstantCommand { p2p.isBotinTolerance() },

            SleepCommand(waitforhuman),
            InstantCommand { setExtendoTarget(0) },


            SequentialCommand(
                setIntakeState(0),
                WaitUntilCommand { isExtendoinTolerance() },
                setIntakeState(1),
                SleepCommand(0.15),
                setClawIntakeState(0),
                InstantCommand { intake.wrist.position = wrist_neutral},
            ),
            SequentialCommand(
                SleepCommand(0.5),
                setIntakeState(2),
                SleepCommand(0.5),
                setClawIntakeState(1),
                SleepCommand(0.5),
            ),

            InstantCommand{ setExtendoTarget(2) },
            SequentialCommand(
                setIntakeState(0),
                WaitUntilCommand { isExtendoinTolerance() },
                SleepCommand(0.2),
                setClawState(0),
                SleepCommand(0.2),
                setClawIntakeState(0),
            ),

            SequentialCommand(
                setArmState(1),
                InstantCommand { setLiftTarget(3)},
                SleepCommand(sleep_start),
                InstantCommand { p2p.followpath(position_for_specimens - Pose(0.0, -20.0, 0.0))},
                WaitUntilCommand { p2p.isBotinTolerance()},
                InstantCommand { p2p.followpath(position_for_specimens)},
            ),
            WaitUntilCommand { p2p.isBotinTolerance()},
            WaitUntilCommand { isLiftinTolerance() },
            SleepCommand(sleep_preload),
            SequentialCommand(
                InstantCommand { setLiftTarget(0) },
                setArmState(0),
                SleepCommand(0.15),
                setClawState(2),
                SleepCommand(0.2),
                setClawState(1),
                //InstantCommand { p2p.followpath(take_specimen)}
            ),
            InstantCommand { setLiftTarget(0) },
            WaitUntilCommand { p2p.isBotinTolerance() },
            InstantCommand { setExtendoTarget(1) },
            InstantCommand { p2p.followpath(park)},
            )
        waitForStart()
        var elap: ElapsedTime = ElapsedTime()
        while(!isStopRequested){

            if(elap.seconds() >= 27.0)
                current_command = SequentialCommand(
                    InstantCommand { setLiftTarget(0) },
                    InstantCommand { setExtendoTarget(1) },
                    InstantCommand { p2p.followpath(park)},
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