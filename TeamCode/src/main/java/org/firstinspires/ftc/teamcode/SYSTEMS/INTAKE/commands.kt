package org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE

import org.firstinspires.ftc.teamcode.ALGORITHMS.stateArray
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.RunUntilCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.SequentialCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.chub_arm_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.chub_arm_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.ehub_arm_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.ehub_arm_neutral
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.intaker_power
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.intaker_spit_power
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.wrist_intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.vars.wrist_neutral

object commands {

    ///0 = neutral, 1 = intake
    fun setArmState(state: Int): Command {
        val states = if(state == 0)
            stateArray(chub_arm_neutral, ehub_arm_neutral)
        else
            stateArray(chub_arm_intake, ehub_arm_intake)

        return SequentialCommand(
            InstantCommand{ intake.chub_arm.position = states[0] },
            InstantCommand{ intake.ehub_arm.position = states[1] }
        )
    }

    ///0 = neutral, 1 = intake
    fun setWristState(state: Int): Command{
        val states = if(state == 0)
            stateArray(wrist_neutral)
        else
            stateArray(wrist_intake)

        return SequentialCommand(
            InstantCommand{ intake.wrist.position = states[0] }
        )
    }

    ///-1 = spit, 0 = stop, 1 = intake
    fun setIntakePower(state: Int, isIntaken: Boolean): Command{
        val states = if(state == 1)
            stateArray(intaker_power)
        else if(state == -1)
            stateArray(intaker_spit_power)
        else
            stateArray(0.0)

        return ParallelCommand(
            InstantCommand { intake.chub_intaker.power = states[0] },
            InstantCommand { intake.ehub_intaker.power = states[0] }
        )
    }
}