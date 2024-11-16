package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.RunUntilCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.WaitUntilCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.derivative
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_target
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.force
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.home_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.home_submersible
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.max_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.max_submersible
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.proportional
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.tolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.commands.isLiftinTolerance
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.home
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import kotlin.math.abs
import kotlin.math.sign

object commands {
    ///0 - max_examination, 1 - home_examination , 2 - home_submersible, 3 - max_submersible
    fun setExtendoTarget(state: Int) {

        if(state != -1)
            PDF(proportional, derivative, force)
        else
            PDF()

        extendo_target = if(state == 0)
            max_examination
        else if(state == 1)
            home_examination
        else if(state == 2)
            home_submersible
        else
            max_submersible

    }

    fun isExtendoinTolerance() = extendo_target - extendo.chub_rails.currentpos < tolerance


    fun setExtendoPowers(pwr1: Double, pwr2: Double){
        extendo.chub_rails.power = pwr1
        extendo.ehub_rails.power = pwr2
    }

    fun setExtendoPowers(pwr1: Double){
        extendo.chub_rails.power = pwr1
        extendo.ehub_rails.power = pwr1
    }

    fun setExtendo(){
        val err = extendo_target - extendo.chub_rails.currentpos

        if(!isExtendoinTolerance())
            setExtendoPowers(extendo_pdf.update(err.toDouble()))
        else
            setExtendoPowers(-force * sign(err.toDouble()))
    }



    fun setExtendoState(): Command{
        val err = extendo_target - extendo.chub_rails.currentpos
        //if i'm not inside the tolerance, i'm running the pdf, else i'm just running the f
        return if (abs(err) > tolerance)
            ParallelCommand(
                RunUntilCommand(
                    InstantCommand { extendo.chub_rails.power = extendo_pdf.update(err.toDouble())},
                    InstantCommand { err < tolerance }
                ),
                RunUntilCommand(
                    InstantCommand { extendo.ehub_rails.power = extendo_pdf.update(err.toDouble())},
                    InstantCommand { err < tolerance }
                ), WaitUntilCommand { err < tolerance }
            )
        else
            ParallelCommand(
                InstantCommand { extendo.chub_rails.power = sign(err.toDouble()) * force },
                InstantCommand { extendo.ehub_rails.power = sign(err.toDouble()) * force }
            )
    }
}