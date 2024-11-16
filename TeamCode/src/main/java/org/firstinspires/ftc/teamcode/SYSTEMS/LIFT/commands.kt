package org.firstinspires.ftc.teamcode.SYSTEMS.LIFT

import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.lift
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.RunUntilCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.ParallelCommand
import org.firstinspires.ftc.teamcode.COMMANDBASE.WaitUntilCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.derivative
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.force
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.high_basket
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.high_chamber
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.high_rung
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.home
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lift_target
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.low_basket
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.low_chamber
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.low_rung
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.max_extension
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.proportional
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.tolerance
import kotlin.math.abs
import kotlin.math.sign

object commands
{
    ///0 - homed, 1 - low chamber, 2 - low rung, 3 - high chamber, 4 - high rung, 5 - low basket, 6 - high basket, 7 - max extension, -1 - emergency stop
    fun setLiftTarget(state: Int) {

        //emergency stop in case i start a campfire
        if(state != -1)
            PDF(proportional, derivative, force)
        else
            PDF()

        lift_target = if(state == 0)
            home
        else if(state == 1)
            low_chamber
        else if(state == 2)
            low_rung
        else if(state == 3)
            high_chamber
        else if(state == 4)
            high_rung
        else if(state == 5)
            low_basket
        else if(state == 6)
            high_basket
        else
            max_extension
    }

    fun isLiftinTolerance() = lift_target-lift.chub_slides.currentpos < tolerance

    fun setLiftPowers(pwr1: Double){
        lift.chub_slides.power = pwr1
        lift.ehub_slides.power = pwr1
    }

    fun setLift(){
        val err = lift_target-lift.chub_slides.currentpos

        if(!isLiftinTolerance())
            setLiftPowers(lift_pdf.update(err.toDouble()))
        else if(lift_target == home)
            setLiftPowers(0.0)
        else
            setLiftPowers(-force * sign(err.toDouble()))
    }



    //not sure if i wanna run my lifts on commandbase
    fun setLiftState(): Command{
        val err = lift_target-lift.chub_slides.currentpos

        //if i'm not inside the tolerance, i'm running the pdf, else i'm just running the f
        return if (abs(err) > tolerance)

            ParallelCommand(
                RunUntilCommand(
                    InstantCommand { lift.chub_slides.power = lift_pdf.update(err.toDouble())},
                    InstantCommand { err < tolerance}
                ),
                RunUntilCommand(
                    InstantCommand { lift.ehub_slides.power = lift_pdf.update(err.toDouble())},
                    InstantCommand { err < tolerance }
                ), WaitUntilCommand { err < tolerance}
            )
        else
            ParallelCommand(
                InstantCommand { lift.chub_slides.power = sign(err.toDouble()) * force },
                InstantCommand { lift.ehub_slides.power = sign(err.toDouble()) * force }
            )
    }

}