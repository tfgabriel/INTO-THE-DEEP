package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import org.firstinspires.ftc.teamcode.ALGORITHMS.Array
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_arm_pickup
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_arm_place
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_claw_close
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.chub_claw_open
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_arm_pickup
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_arm_place
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_claw_close
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.ehub_claw_open
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_chub
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.positioner_ehub
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.wrist_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.wrist_neutral

object simple_commands {
    /// O = open, 1 = close
    fun setClawState(state: Int): Command {
        val states: Array = if(state == 0)
            Array(chub_claw_close, ehub_claw_close)
        else
            Array(chub_claw_open, ehub_claw_open)

        return ParallelCommand(
            InstantCommand{ outtake.ehub_claw.position = states[0]},
            InstantCommand{ outtake.chub_claw.position = states[1]}
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

    /// 0 = pickup, 1 = chamber
    fun setArmState(state: Int): Command{
        val states: Array = if(state == 0)
            Array(chub_arm_pickup, ehub_arm_pickup)
        else
            Array(chub_arm_place, ehub_arm_place)

        return ParallelCommand(
            InstantCommand{ outtake.chub_arm.position = states[0] },
            InstantCommand{ outtake.ehub_arm.position = states[1] }
        )
    }

    /// 0 = neutral, 1 = intake
    fun setWristState(state: Int): Command{
        val states: Array = if(state == 0)
            Array(wrist_neutral)
        else
            Array(wrist_intake)

        return SequentialCommand(
            InstantCommand{ outtake.wrist.position = states[0] }
        )
    }
}