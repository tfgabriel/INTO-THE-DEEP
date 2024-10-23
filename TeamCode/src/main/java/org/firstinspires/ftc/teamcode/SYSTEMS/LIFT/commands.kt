package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFL
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFLCoef
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.RunUntilCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.derivative
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.force
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.high_chamber
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.high_rung
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.home
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.lift_pdfl
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.low_chamber
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.low_rung
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.lower_limit
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.max_extension
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.proportional
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.vars.tolerance

object commands
{
    ///0 - homed, 1 - low chamber, 2 - low rung, 3 - high chamber, 4 - high rung, 5 - max extension, -1 - emergency stop
    fun setLiftState(state: Int): Command {
        lift_pdfl = if(state == 0)
            PDFL(PDFLCoef(0.0, 0.0, lower_limit, 0.0))
        else if(state != -1)
            PDFL(PDFLCoef(proportional, derivative, force, 0.0))
        else
            PDFL()

        val target = if(state == 0)
            home
        else if(state == 1)
            low_chamber
        else if(state == 2)
            low_rung
        else if(state == 3)
            high_chamber
        else if(state == 4)
            high_rung
        else
            max_extension

        return ParallelCommand(
            RunUntilCommand(
                InstantCommand{ lift.chub_slides.power = lift_pdfl.update((target-lift.chub_slides.currentpos, tolerance )},
                InstantCommand { target-lift.chub_slides.currentpos < tolerance}
            ),
            RunUntilCommand(
                InstantCommand{ lift.ehub_slides.power = lift_pdfl.update(target-lift.chub_slides.currentpos, tolerance )},
                InstantCommand { target-lift.chub_slides.currentpos < tolerance}
            )
        )
    }

}