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
import org.firstinspires.ftc.teamcode.SYSTEMS.LIFT.lift_vars.lazy_coef
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
        lift_pdf = if(state != -1)
            PDF(proportional, derivative, force)
        else
            PDF()

        lift_target = when (state) {
            0 -> home
            1 -> low_chamber
            2 -> low_rung
            3 -> high_chamber
            4 -> high_rung
            5 -> low_basket
            6 -> high_basket
            else -> max_extension
        }
    }

    fun setLiftTargetCommand(state: Int): Command{
        //emergency stop in case i start a campfire
        lift_pdf = if(state != -1)
            PDF(proportional, derivative, force)
        else
            PDF()

        return InstantCommand {
            lift_target = when (state) {
                0 -> home
                1 -> low_chamber
                2 -> low_rung
                3 -> high_chamber
                4 -> high_rung
                5 -> low_basket
                6 -> high_basket
                else -> max_extension
            }
        }
    }

    fun isLiftinTolerance() = abs(lift_target-lift.ehub_slides.currentpos) < tolerance
    fun isLiftinHomeTolerance() = abs(lift_target - lift.ehub_slides.currentpos) < 40.0

    fun setLiftPowers(pwr1: Double){
        lift.chub_slides.power = pwr1 + if(abs(lift.chub_slides.currentpos - lift.ehub_slides.power) < 4 || isLiftinTolerance()) 0.0 else (lift.chub_slides.currentpos - lift.ehub_slides.power) * lift_vars.rebepe
        lift.ehub_slides.power = pwr1
    }

    fun setLift(){
        val err = lift_target-lift.ehub_slides.currentpos

        if(!isLiftinTolerance()){
            if(lift_target != home)
                setLiftPowers(lift_pdf.update(err.toDouble()))
            else if(!isLiftinHomeTolerance())
                setLiftPowers(-0.9)
            else
                setLiftPowers(0.0)
        }
        else if(lift_target == home)
            setLiftPowers(0.0)
        else
            setLiftPowers(force * sign(err.toDouble()))
    }

}