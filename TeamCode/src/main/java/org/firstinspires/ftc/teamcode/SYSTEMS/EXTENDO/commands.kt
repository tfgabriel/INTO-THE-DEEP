package org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO

import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.extendo
import org.firstinspires.ftc.teamcode.COMMANDBASE.Command
import org.firstinspires.ftc.teamcode.COMMANDBASE.InstantCommand
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.derivative
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_pdf
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.extendo_target
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.force
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.home_extendo
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.max_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.mid_examination
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.proportional
import org.firstinspires.ftc.teamcode.SYSTEMS.EXTENDO.extendo_vars.tolerance
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import kotlin.math.abs
import kotlin.math.sign

object commands {
    ///0 - home, 1 - mid , 2 - max, 3 - max_submersible
    fun setExtendoTarget(state: Int) {
        extendo_pdf = if (state != -1)
            PDF(proportional, derivative, force)
        else
            PDF()

        extendo_target = if (state == 0)
            home_extendo
        else if (state == 1)
            mid_examination
        else
            max_examination

    }

    fun setExtendoTargetLinear(dist: Int) {
        extendo_pdf = PDF(proportional, derivative, force)
        extendo_target = dist
    }

    fun setExtendoTargetCommand(state: Int): Command {

        extendo_pdf = if (state != -1)
            PDF(proportional, derivative, force)
        else
            PDF()

        return InstantCommand {
            extendo_target = if (state == 0)
                home_extendo
            else if (state == 1)
                mid_examination
            else
                max_examination
        }

    }

    fun isExtendoinTolerance() = abs(extendo_target - extendo.chub_rails.currentpos) < tolerance
    fun isExtendoinHomeTolerance() = abs(extendo_target - extendo.chub_rails.currentpos) < 25.0


    /*
    fun setExtendoPowers(pwr1: Double, pwr2: Double){
        extendo.chub_rails.power = pwr1
        //extendo.ehub_rails.power = pwr2
    }
    */

    fun setExtendoPowers(pwr1: Double) {
        extendo.chub_rails.power = pwr1
        //extendo.ehub_rails.power = pwr1
    }

    fun setExtendo(gamepad_power: Double) {
        val err = extendo_target - extendo.chub_rails.currentpos


        if (!isExtendoinTolerance()) {
            if (extendo_target != home_extendo) {
                setExtendoPowers(extendo_pdf.update(err.toDouble()))
                send_toall("extendo is", "going elsewhere")
            } else if (!isExtendoinHomeTolerance()) {
                setExtendoPowers(1.0)
                send_toall("extendo is", "going home")
            } else {
                setExtendoPowers(0.0)
                send_toall("extendo is", "home")
            }
        } else {
            setExtendoPowers(force * sign(err.toDouble()) + gamepad_power * 0.5)
            send_toall("extendo is", "idling")

        }
    }


    /*fun setExtendoState(): Command{
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

     */
}