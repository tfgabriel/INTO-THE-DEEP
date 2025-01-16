package org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE

import org.firstinspires.ftc.teamcode.ALGORITHMS.Array
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.outtake
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SleepCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.claw_close
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.claw_open
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.fb_score
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.fb_steal
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.fb_transfer
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.idle
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.score_basket
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.score_specimen
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.steal
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.outtake_vars.transfer_outtake
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setArmState
import org.firstinspires.ftc.teamcode.SYSTEMS.OUTTAKE.simple_commands.setClawState
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall

object simple_commands {
    /// O = close, 1 = open, 2 - intermediary
    fun setClawState(state: Int): Command {
        val states: Array = when (state) {
            0 -> Array(claw_close)
            else -> Array(claw_open)
        }

        return ParallelCommand(
            InstantCommand{ outtake.claw.position = states[0]},
        )
    }



    /// 0 = pickup, 1 = chamber, 2 = basket. 3 = steal
    fun setArmState(state: Int): Command {
        val states: Array = when (state) {
            0 -> Array(transfer_outtake)
            1 -> Array(idle)
            2 -> Array(score_specimen)
            3 -> Array(score_basket)
            else-> Array(steal)
        }

        return ParallelCommand(
            InstantCommand{ outtake.chub_arm.position = states[0] },
            InstantCommand{ outtake.ehub_arm.position = states[0] },
            InstantCommand{ send_toall("outtake state", state)}
        )
    }

    fun setOuttakeFourbar(state: Int): Command{
        val states: Array = when (state) {
            0 -> Array(fb_transfer)
            1 -> Array(fb_transfer)
            2 -> Array(fb_score)
            3 -> Array(fb_score)
            else -> Array(fb_steal)
        }

        return ParallelCommand(
            InstantCommand{ outtake.fourbar.position = states[0] },
            InstantCommand{ send_toall(" state", state)}
        )
    }

    fun setOuttake(state: Int): Command{
        return ParallelCommand(
            setArmState(state),
            setOuttakeFourbar(state)
        )
    }

}

object complex_commands{

    fun transfer(): Command {
        return SequentialCommand(
            setClawState(0),
        )
    }

    //place and return
    fun place_specimen(): Command {
        return SequentialCommand(
            SleepCommand(0.3),
            setClawState(2),
            SleepCommand(0.1),
            setClawState(1)
        )
    }

    fun place_sample(): Command {
        return SequentialCommand(
            setClawState(1),
            SleepCommand(0.2),
            setArmState(0)
        )
    }

    //prepare to place
    fun prepare_specimen(): Command {
        return SequentialCommand(
            setArmState(1)
        )
    }

    fun prepare_sample(): Command {
        return SequentialCommand(
            setArmState(2)
        )
    }


    fun reset_outtake(): Command {
        return SequentialCommand(
            setArmState(0),
            setClawState(1)
        )
    }


}
