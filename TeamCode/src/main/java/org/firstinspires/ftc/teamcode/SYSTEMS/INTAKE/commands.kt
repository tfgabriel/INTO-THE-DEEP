package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import org.firstinspires.ftc.teamcode.ALGORITHMS.Array
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_to_pos
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTargetCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.home_extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setArmStateIntake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setFourbar
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.claws_closed
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.claws_open
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_hover
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_third
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.intake_time
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.spit_time
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.transverse_time
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral

object commands {

    fun setArmStateIntake(state: Int): Command {
        val states = when (state) {
            0 -> Array(intake_vars.transfer)
            1 -> Array(intake_vars.hover)
            else -> Array(intake_vars.intake)
        }


        return ParallelCommand(
            InstantCommand{ intake.chub_arm.position = states[0] },
            InstantCommand{ intake.ehub_arm.position = states[0] }
        )
    }


    fun setFourbar(state: Int): Command {
        val states = when (state) {
            0 -> Array(fourbar_transfer)
            1 -> Array(fourbar_hover)
            2 -> Array(fourbar_intake)
            else -> Array(fourbar_third)
        }


        return InstantCommand { intake.fourbar.position = states[0] }

    }

    fun setIntakeState(state: Int): Command {
        return ParallelCommand(
            setFourbar(state),
            setArmStateIntake(state)
        )
    }

    //if i have my cam open and my arm is going down, set the wrist according to the camera's feed, else, set it ready for transfer
    //the else if is there just not to constantly set the servo position even if it's already in the right position
    fun setWrist(): Command {
        return InstantCommand{ intake.wrist.position = wrist_neutral}
    }

    fun setWristCommand(): Command {
        val pos = if(camera.is_open && extendo.chub_rails.currentpos > home_extendo)
            ang_to_pos(Vec2D(camera.corners()!!.lld[0]), Vec2D(camera.corners()!!.lld[2]))
        else
            wrist_neutral

        return InstantCommand{
            intake.wrist.position = pos
        }
    }

    // 0 - open, 1 - closed
    fun setClawIntakeState(state: Int): Command {
        val states = when(state){
            0 -> Array(claws_open)
            else -> Array(claws_closed)
        }

        return ParallelCommand(
            InstantCommand { intake.claws.position = states[0] }
        )
    }
}

object intake_commands{

    fun setup_intake_for_specimen(): Command {
        return SequentialCommand(
            setArmStateIntake(0),
            SleepCommand(transverse_time),
            setFourbar(2),
            SleepCommand(spit_time),
            //setIntakePower(-1),
            SleepCommand(0.2),
            //setIntakePower(0)
        )
    }

    fun intake_specimen(): Command {
        return SequentialCommand(
            //setIntakePower(1),
            ParallelCommand(
                setArmStateIntake(0),
                setFourbar(2)
            ),
            SleepCommand(intake_time),
            go_for_specimen()
        )
    }

    fun intake_sample(): Command {
        return SequentialCommand(
            setArmStateIntake(0),
            //setIntakePower(1),
            )
    }

    fun go_for_specimen(): Command {
        return SequentialCommand(
            setArmStateIntake(2),
            setFourbar(1),
        )
    }

    fun setup_intake_for_transfer(): Command {
        return SequentialCommand(
            setArmStateIntake(2),
            setFourbar(1),
            SleepCommand(0.2),
            setArmStateIntake(3),
            setFourbar(3)
        )
    }

    fun sample_spit(): Command {
        return SequentialCommand(
            ParallelCommand(
                setArmStateIntake(2),
                setFourbar(1)
            ),
            SleepCommand(transverse_time),
            //setIntakePower(-1)
        )
    }

    fun specimen_intake(): Command {
        return SequentialCommand(
            ParallelCommand(
                setFourbar(2),
                setArmStateIntake(0)
            ),
            //setIntakePower(1)
        )
    }

    fun transfer(): Command {
        return SequentialCommand(
            setArmStateIntake(3),
            setFourbar(4),
            SleepCommand(0.3),
            setExtendoTargetCommand(2),
            SleepCommand(0.15),
            setFourbar(5)
            )
    }

    fun intake(): Command {
        return SequentialCommand(
            setArmStateIntake(0),
            setFourbar(0),
            SleepCommand(0.2),
            setArmStateIntake(1),
            //setIntakePower(1)
        )
    }

    fun reset_arm_and_intake(): Command {
        return SequentialCommand(
            SleepCommand(0.5),
            ParallelCommand(
                setArmStateIntake(2),
                setFourbar(1)
            ),
            SleepCommand(transverse_time),
            //setIntakePower(-1)
        )
    }
}