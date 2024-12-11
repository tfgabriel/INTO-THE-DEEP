package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import org.firstinspires.ftc.teamcode.ALGORITHMS.Array
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_to_pos
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.pos_diff
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.commands.setExtendoTargetCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setArmStateIntake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setFourbar
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_specimen
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.chub_arm_transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.claws_closed
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.claws_open
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_specimen
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.ehub_arm_transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_deaaa
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_mid
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.fourbar_up
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.intake_time
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.intaker_power
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.intaker_spit_power
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.spit_time
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.transverse_time
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars.wrist_neutral
import org.firstinspires.ftc.teamcode.TELEOP.isIntaking

object commands {
    ///0 = neutral, 1 = intake, 2 = specimen, 3 = transfer
    fun setArmStateIntake(state: Int): Command {
        val states = when (state) {
            0 -> Array(chub_arm_neutral, ehub_arm_neutral)
            1 -> Array(chub_arm_intake, ehub_arm_intake)
            2 -> Array(chub_arm_specimen, ehub_arm_specimen)
            else -> Array(chub_arm_transfer, ehub_arm_transfer)
        }


        return SequentialCommand(
            InstantCommand{ intake.chub_arm.position = states[0] },
            InstantCommand{ intake.ehub_arm.position = states[1] }
        )
    }

    ///-1 = spit, 0 = stop, 1 = intake
    /*fun setIntakePower(state: Int): Command{
        val states = when (state) {
            1 -> Array(intaker_power)
            -1 -> Array(intaker_spit_power)
            else -> Array()
        }

        return ParallelCommand(
            InstantCommand { intake.chub_intaker.power = states[0] },
            InstantCommand { intake.ehub_intaker.power = states[0] }
        )
    }

     */

    //0 - intake, 1 - up, 2 - mid, 3 - transfer
    fun setFourbar(state: Int): Command{
        val states = when (state) {
            0 -> Array(fourbar_intake)
            1 -> Array(fourbar_up)
            2 -> Array(fourbar_mid)
            4-> Array(fourbar_transfer)
            else -> Array(fourbar_deaaa)
        }


        return ParallelCommand(
            InstantCommand { intake.fourbar.position = states[0] }
        )
    }

    //if i have my cam open and my arm is going down, set the wrist according to the camera's feed, else, set it ready for transfer
    //the else if is there just not to constantly set the servo position even if it's already in the right position
    fun setWrist(){
        if(camera.is_open && isIntaking)
            intake.wrist.position = ang_to_pos(camera.corners().p1, camera.corners().p2)
        else if(!pos_diff(intake.wrist.position, wrist_neutral))
            intake.wrist.position = wrist_neutral
    }

    fun setClawIntakeState(state: Int): Command{
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

    fun setup_intake_for_specimen(): Command{
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

    fun intake_specimen(): Command{
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

    fun intake_sample(): Command{
        return SequentialCommand(
            setArmStateIntake(0),
            //setIntakePower(1),
            )
    }

    fun go_for_specimen():Command{
        return SequentialCommand(
            setArmStateIntake(2),
            setFourbar(1),
        )
    }

    fun setup_intake_for_transfer(): Command{
        return SequentialCommand(
            setArmStateIntake(2),
            setFourbar(1),
            SleepCommand(0.2),
            setArmStateIntake(3),
            setFourbar(3)
        )
    }

    fun sample_spit(): Command{
        return SequentialCommand(
            ParallelCommand(
                setArmStateIntake(2),
                setFourbar(1)
            ),
            SleepCommand(transverse_time),
            //setIntakePower(-1)
        )
    }

    fun specimen_intake(): Command{
        return SequentialCommand(
            ParallelCommand(
                setFourbar(2),
                setArmStateIntake(0)
            ),
            //setIntakePower(1)
        )
    }

    fun transfer(): Command{
        return SequentialCommand(
            setArmStateIntake(3),
            setFourbar(4),
            SleepCommand(0.3),
            setExtendoTargetCommand(2),
            SleepCommand(0.15),
            setFourbar(5)
            )
    }

    fun intake(): Command{
        return SequentialCommand(
            setArmStateIntake(0),
            setFourbar(0),
            SleepCommand(0.2),
            setArmStateIntake(1),
            //setIntakePower(1)
        )
    }

    fun reset_arm_and_intake(): Command{
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