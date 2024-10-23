package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFL
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFLCoef
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.RunUntilCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.derivative
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.extendo_pdfl
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.force
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.home_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.home_submersible
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.lower_limit
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.max_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.max_submersible
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.proportional
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.vars.tolerance

object commands {
    ///0 - max_examination, 1 - home_examination , 2 - home_submersible, 3 - max_submersible
    fun setLiftState(state: Int): Command {
        extendo_pdfl = if(state == 0)
            PDFL(PDFLCoef(0.0, 0.0, lower_limit, 0.0))
        else if(state != -1)
            PDFL(PDFLCoef(proportional, derivative, force, 0.0))
        else
            PDFL()

        val target = if(state == 0)
            max_examination
        else if(state == 1)
            home_examination
        else if(state == 2)
            home_submersible
        else
            max_submersible

        return ParallelCommand(
            RunUntilCommand(
                InstantCommand{ extendo.chub_rails.power = extendo_pdfl.update((target - extendo.chub_rails.currentpos).toDouble(), tolerance )},
                InstantCommand { target - extendo.chub_rails.currentpos < tolerance }
            ),
            RunUntilCommand(
                InstantCommand{ extendo.ehub_rails.power = extendo_pdfl.update((target - extendo.chub_rails.currentpos).toDouble(), tolerance )},
                InstantCommand { target - extendo.chub_rails.currentpos < tolerance }
            )
        )
    }
}