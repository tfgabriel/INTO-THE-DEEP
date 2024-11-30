package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import org.firstinspires.ftc.teamcode.ALGORITHMS.Array
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.commands.setIntakePower
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_arm_basket
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_arm_pickup
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_arm_place
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_claw_close
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_claw_intermed
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_claw_open
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_arm_basket
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_arm_pickup
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_arm_place
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_claw_close
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_claw_intermed
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_claw_open
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.place_time
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_chub
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_ehub
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.prepare_time_sample
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.prepare_time_specimen
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState

object simple_commands {
    /// O = close, 1 = open, 2 - intermediary
    fun setClawState(state: Int): Command {
        val states: Array = if(state == 0)
            Array(chub_claw_close, ehub_claw_close)
        else if(state == 1)
            Array(chub_claw_open, ehub_claw_open)
        else
            Array(chub_claw_intermed, ehub_claw_intermed)

        return ParallelCommand(
            InstantCommand{ outtake.chub_claw.position = states[0]},
            InstantCommand{ outtake.ehub_claw.position = states[1]}
        )
    }

    /// -1 = chub, 0 = neutral, 1 = ehub
    fun setPositionerState(state: Int): Command{
        val states: Array = if(state == 0)
            Array(positioner_neutral)
        else if(state == -1)
            Array(positioner_chub)
        else
            Array(positioner_ehub)

        return SequentialCommand(
            InstantCommand{ outtake.positioner.position = states[0] }
        )
    }

    /// 0 = pickup, 1 = chamber, 2 = basket
    fun setArmState(state: Int): Command{
        val states: Array = if(state == 0)
            Array(chub_arm_pickup, ehub_arm_pickup)
        else if(state == 1)
            Array(chub_arm_place, ehub_arm_place)
        else
            Array(chub_arm_basket, ehub_arm_basket)

        return ParallelCommand(
            InstantCommand{ outtake.chub_arm.position = states[0] },
            InstantCommand{ outtake.ehub_arm.position = states[1] }
        )
    }

    /*
    /// NOT IN USE
    /// 0 = neutral, 1 = intake
    fun setWristState(state: Int): Command{
        val states: Array = if(state == 0)
            Array(wrist_neutral)
        else
            Array(wrist_intake)

        return SequentialCommand(
           // InstantCommand{ outtake.wrist.position = states[0] }
        )
    }
     */
}

object complex_commands{

    fun transfer(): Command{
        return SequentialCommand(
            setIntakePower(-1),
            setClawState(0),
            setIntakePower(0)
        )
    }

    //place and return
    fun place_specimen(): Command{
        return SequentialCommand(
            SleepCommand(place_time),
            setClawState(2),
            SleepCommand(0.1),
            setClawState(1),
            SleepCommand(0.2),
            setArmState(0)
        )
    }

    fun place_sample(): Command{
        return SequentialCommand(
            SleepCommand(0.1),
            setClawState(1),
            SleepCommand(0.2),
            setArmState(0)
        )
    }

    //prepare to place
    fun prepare_specimen(): Command{
        return SequentialCommand(
            SleepCommand(prepare_time_specimen),
            setArmState(1)
        )
    }

    fun prepare_sample(): Command{
        return SequentialCommand(
            SleepCommand(prepare_time_sample),
            setArmState(2)
        )
    }

}