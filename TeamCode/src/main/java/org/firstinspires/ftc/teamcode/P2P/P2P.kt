package org.firstinspires.ftc.teamcode.P2P

import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFL
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDFLCoef
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.P2P.vars.angular_tolerance
import org.firstinspires.ftc.teamcode.P2P.vars.current_pos
import org.firstinspires.ftc.teamcode.P2P.vars.err
import org.firstinspires.ftc.teamcode.P2P.vars.hPDFL
import org.firstinspires.ftc.teamcode.P2P.vars.h_d
import org.firstinspires.ftc.teamcode.P2P.vars.h_err
import org.firstinspires.ftc.teamcode.P2P.vars.h_f
import org.firstinspires.ftc.teamcode.P2P.vars.h_p
import org.firstinspires.ftc.teamcode.P2P.vars.path
import org.firstinspires.ftc.teamcode.P2P.vars.slow
import org.firstinspires.ftc.teamcode.P2P.vars.tolerance
import org.firstinspires.ftc.teamcode.P2P.vars.xPDFL
import org.firstinspires.ftc.teamcode.P2P.vars.x_d
import org.firstinspires.ftc.teamcode.P2P.vars.x_err
import org.firstinspires.ftc.teamcode.P2P.vars.x_f
import org.firstinspires.ftc.teamcode.P2P.vars.x_p
import org.firstinspires.ftc.teamcode.P2P.vars.yPDFL
import org.firstinspires.ftc.teamcode.P2P.vars.y_d
import org.firstinspires.ftc.teamcode.P2P.vars.y_err
import org.firstinspires.ftc.teamcode.P2P.vars.y_f
import org.firstinspires.ftc.teamcode.P2P.vars.y_p
import kotlin.math.cos
import kotlin.math.sin

class P2P {
    var done: Boolean = false
        get(){
            return err.distance() < tolerance && h_err < angular_tolerance
        }
        set(value){
            field = value
        }

    fun followpath(current_path: Pose){
        xPDFL = PDFL(PDFLCoef( x_p, x_d, x_f, 0.0))
        yPDFL = PDFL(PDFLCoef(y_p, y_d, y_f, 0.0))
        hPDFL = PDFL(PDFLCoef(h_p, h_d, h_f, 0.0))

        path = current_path
    }

    fun update(){
        current_pos = localizer.pos
        err = path - current_pos

        x_err = cos(imew.yaw) * (err.x - tolerance) - sin(imew.yaw) * (err.y - tolerance)
        y_err = sin(imew.yaw) * (err.x - tolerance) + cos(imew.yaw) * (err.y - tolerance)
        h_err = ang_diff(current_pos.h, imew.yaw)

        chassis.fc_drive(xPDFL.update(x_err, tolerance) * slow, yPDFL.update(y_err, tolerance) * slow, hPDFL.update(h_err, angular_tolerance), 0.0)
    }
}