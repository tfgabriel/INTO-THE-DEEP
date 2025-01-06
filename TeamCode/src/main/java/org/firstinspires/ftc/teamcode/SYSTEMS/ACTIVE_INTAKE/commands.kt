package org.firstinspires.ftc.teamcode.SYSTEMS.ACTIVE_INTAKE

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.active_intake
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.intake
import org.firstinspires.ftc.teamcode.SYSTEMS.INTAKE.intake_vars
import org.firstinspires.ftc.teamcode.ALGORITHMS.Array
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand

object commands {
    fun setIntakePower(power: Double): Command {
        return ParallelCommand(
            InstantCommand { active_intake.chub_intaker.power = power },
            InstantCommand { active_intake.ehub_intaker.power = power }
        )
    }

    fun setArmStateIntake(state: Int): Command {
        val states = when (state) {
            0 -> Array(active_intake_vars.transfer)
            1 -> Array(active_intake_vars.hover)
            else -> Array(active_intake_vars.intake)
        }

        return ParallelCommand(
            InstantCommand{ intake.chub_arm.position = states[0] },
            InstantCommand{ intake.ehub_arm.position = states[0] }
        )
    }
}